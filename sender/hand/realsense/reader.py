"""RealSense D405 hand reader for realsense_sender.

Wraps RSCamera + HandDetector + DepthKeypointConverter into a single
plain class with the same lifecycle as ManusReader (connect/disconnect/
get_data). Detection runs in a background daemon thread; get_data() is
non-blocking and always returns the latest result.

Output: HandKeypoints with keypoints_3d=(21,3) wrist-centered, MANO frame.

Usage:
    reader = RealSenseReader(hand_side="right")
    reader.connect()
    while True:
        data = reader.get_data()
        if data is not None:
            print(data.keypoints_3d.shape)  # (21, 3)
    reader.disconnect()
"""

import logging
import threading
import time
from typing import Optional

import numpy as np

from sender.hand.realsense.constants import RS_FPS, RS_HEIGHT, RS_WIDTH
from sender.hand.realsense.depth_keypoint_converter import DepthKeypointConverter
from sender.hand.realsense.hand_detector import HandDetection, HandDetector
from sender.hand.realsense.hand_keypoints import HandKeypoints
from sender.hand.realsense.rs_camera import RSCamera

logger = logging.getLogger(__name__)


class RealSenseReader:
    """RealSense D405 → MANO 21 hand keypoints.

    Mirrors ManusReader's connect/disconnect/get_data interface so that
    realsense_sender can use the same main-loop pattern as manus_sender.

    Parameters
    ----------
    hand_side : str
        "left" or "right". Selects which detected hand is returned.
    serial : str or None
        D405 device serial. None = auto-detect first connected device.
    width, height, fps : int
        Color + depth stream config (D405 default 640x480 @ 30fps).
    num_hands : int
        Number of hands MediaPipe should detect. Set 1 for single-hand mode.
    model_path : str or None
        Custom MediaPipe .task model path. None = bundled model under
        ``sender/hand/realsense/models/hand_landmarker.task``.
    """

    def __init__(
        self,
        hand_side: str = "right",
        serial: Optional[str] = None,
        width: int = RS_WIDTH,
        height: int = RS_HEIGHT,
        fps: int = RS_FPS,
        num_hands: int = 1,
        model_path: Optional[str] = None,
    ):
        self._hand_side = hand_side.lower()
        self._camera = RSCamera(serial=serial, width=width, height=height, fps=fps)
        self._detector = HandDetector(model_path=model_path, num_hands=num_hands)
        self._converter: Optional[DepthKeypointConverter] = None  # created on connect

        # Timing
        self._start_time_ms = 0
        self._detection_time_ms = 0.0

        # Latest results (protected by lock)
        self._lock = threading.Lock()
        self._latest_keypoints: Optional[HandKeypoints] = None
        self._latest_frame: Optional[np.ndarray] = None

        # Detection thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── Lifecycle (manus_reader-style aliases) ───────────────────

    def connect(self) -> None:
        """Open camera, start detector, spawn detection thread."""
        self._camera.start()
        self._detector.start()
        # MANO transform applied inside DepthKeypointConverter
        self._converter = DepthKeypointConverter(
            self._camera.intrinsics, hand_type=self._hand_side,
        )
        self._start_time_ms = int(time.monotonic() * 1000)
        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("RealSenseReader started (hand=%s)", self._hand_side)
        print(f"[RealSenseReader] Started (hand={self._hand_side})")

    def disconnect(self) -> None:
        """Stop detection thread, close detector, stop camera."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._detector.close()
        self._camera.stop()
        logger.info("RealSenseReader stopped.")
        print("[RealSenseReader] Stopped")

    # ── Data access (non-blocking) ───────────────────────────────

    def get_data(self) -> Optional[HandKeypoints]:
        """Return the latest HandKeypoints, or None if no hand detected yet."""
        with self._lock:
            return self._latest_keypoints

    def get_frame(self) -> Optional[np.ndarray]:
        """Return the BGR color frame corresponding to the latest detection."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    # ── Properties ───────────────────────────────────────────────

    @property
    def detection_time_ms(self) -> float:
        return self._detection_time_ms

    @property
    def hand_side(self) -> str:
        return self._hand_side

    # ── Background detection thread ──────────────────────────────

    def _detect_loop(self) -> None:
        """Grab frames → run MediaPipe → convert via depth → store latest."""
        while self._running:
            ok, color, depth = self._camera.read()
            if not ok or color is None or depth is None:
                time.sleep(0.001)
                continue

            ts_ms = int(time.monotonic() * 1000) - self._start_time_ms
            h, w = color.shape[:2]

            t0 = time.perf_counter()
            detections = self._detector.detect(color, ts_ms)
            self._detection_time_ms = (time.perf_counter() - t0) * 1000

            keypoints: Optional[HandKeypoints] = None
            if detections:
                best = self._select_hand(detections)
                if best is not None:
                    kp_3d = self._converter.convert(best, depth, w, h)
                    kp_2d = DepthKeypointConverter.extract_2d(best)
                    keypoints = HandKeypoints(
                        keypoints_3d=kp_3d,
                        keypoints_2d=kp_2d,
                        handedness=best.handedness.lower(),
                        confidence=best.handedness_score,
                        timestamp=time.time(),
                        source="realsense",
                    )

            with self._lock:
                self._latest_keypoints = keypoints
                self._latest_frame = color

    def _select_hand(self, detections: list[HandDetection]) -> Optional[HandDetection]:
        """Pick the detected hand matching self._hand_side with highest confidence."""
        target = self._hand_side.capitalize()
        candidates = [d for d in detections if d.handedness == target]
        if not candidates:
            candidates = detections
        return max(candidates, key=lambda d: d.handedness_score)
