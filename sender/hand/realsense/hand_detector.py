"""MediaPipe HandLandmarker wrapper.

Uses the modern MediaPipe Tasks API (not the legacy mp.solutions.hands).
Provides both 2D image landmarks and 3D world landmarks.

The .task model file is bundled at ``models/hand_landmarker.task``
(same directory as this module). No network download required.
"""

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# Defaults (previously in phone/config.py, now self-contained)
MP_NUM_HANDS = 1
MP_MIN_DETECTION_CONFIDENCE = 0.5
MP_MIN_TRACKING_CONFIDENCE = 0.5
MP_MODEL_PATH = None  # None → use bundled model

logger = logging.getLogger(__name__)

# Bundled model — no network access needed
_BUNDLED_MODEL = Path(__file__).parent / "models" / "hand_landmarker.task"


@dataclass
class HandDetection:
    """Raw detection output from MediaPipe."""

    landmarks_2d: np.ndarray       # (21, 3) normalized image coords [x, y, z]
    world_landmarks: np.ndarray    # (21, 3) meters, hand-geometric-center origin
    handedness: str                # "Left" or "Right"
    handedness_score: float        # confidence [0, 1]


class HandDetector:
    """MediaPipe HandLandmarker with VIDEO running mode."""

    def __init__(
        self,
        model_path: str | None = MP_MODEL_PATH,
        num_hands: int = MP_NUM_HANDS,
        min_detection_confidence: float = MP_MIN_DETECTION_CONFIDENCE,
        min_tracking_confidence: float = MP_MIN_TRACKING_CONFIDENCE,
    ):
        self._model_path = self._ensure_model(model_path)
        self._num_hands = num_hands
        self._min_det = min_detection_confidence
        self._min_track = min_tracking_confidence
        self._landmarker = None

    def start(self) -> None:
        """Create the HandLandmarker instance."""
        import mediapipe as mp

        base_options = mp.tasks.BaseOptions(
            model_asset_path=str(self._model_path),
        )
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=self._num_hands,
            min_hand_detection_confidence=self._min_det,
            min_hand_presence_confidence=self._min_det,
            min_tracking_confidence=self._min_track,
        )
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
        logger.info("HandDetector started (model: %s)", self._model_path)

    def detect(
        self, frame_bgr: np.ndarray, timestamp_ms: int
    ) -> list[HandDetection]:
        """Run hand detection on a BGR frame.

        Args:
            frame_bgr: OpenCV BGR image.
            timestamp_ms: Monotonically increasing timestamp in milliseconds.

        Returns:
            List of HandDetection (may be empty if no hands detected).
        """
        import mediapipe as mp

        if self._landmarker is None:
            raise RuntimeError("HandDetector not started. Call start() first.")

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

        detections: list[HandDetection] = []
        for i, landmarks in enumerate(result.hand_landmarks):
            # 2D normalized image coordinates
            lm_2d = np.array(
                [[lm.x, lm.y, lm.z] for lm in landmarks], dtype=np.float32
            )
            # 3D world landmarks (meters)
            wl = result.hand_world_landmarks[i]
            lm_3d = np.array(
                [[lm.x, lm.y, lm.z] for lm in wl], dtype=np.float32
            )
            # Handedness
            h = result.handedness[i][0]
            detections.append(
                HandDetection(
                    landmarks_2d=lm_2d,
                    world_landmarks=lm_3d,
                    handedness=h.category_name,       # "Left" or "Right"
                    handedness_score=h.score,
                )
            )

        return detections

    def close(self) -> None:
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
            logger.info("HandDetector closed.")

    # ── Model management ──────────────────────────────────

    @staticmethod
    def _ensure_model(model_path: str | None) -> Path:
        """Return path to the .task model file.

        Priority: explicit path → bundled model in ``models/``.
        """
        if model_path is not None:
            p = Path(model_path)
            if p.exists():
                return p
            raise FileNotFoundError(f"Model file not found: {model_path}")

        if _BUNDLED_MODEL.exists():
            return _BUNDLED_MODEL

        raise FileNotFoundError(
            f"Bundled model not found at {_BUNDLED_MODEL}. "
            "Download hand_landmarker.task from "
            "https://storage.googleapis.com/mediapipe-models/"
            "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task "
            f"and place it at {_BUNDLED_MODEL}"
        )
