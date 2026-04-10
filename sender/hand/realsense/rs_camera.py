"""RealSense D405 camera wrapper using pyrealsense2.

Provides aligned color + depth frames and camera intrinsics
for pixel-to-3D deprojection.
"""

import logging

import numpy as np

from sender.hand.realsense.constants import RS_FPS, RS_HEIGHT, RS_WIDTH

logger = logging.getLogger(__name__)


class RSCamera:
    """pyrealsense2 pipeline wrapper for color + depth streaming."""

    def __init__(
        self,
        serial: str | None = None,
        width: int = RS_WIDTH,
        height: int = RS_HEIGHT,
        fps: int = RS_FPS,
    ):
        self._serial = serial
        self._width = width
        self._height = height
        self._fps = fps

        self._pipeline = None
        self._align = None
        self._depth_scale = 0.001  # default, updated in start()
        self._intrinsics = None

    def start(self) -> None:
        """Open the RealSense pipeline and start streaming."""
        import pyrealsense2 as rs

        self._pipeline = rs.pipeline()
        config = rs.config()

        if self._serial:
            config.enable_device(self._serial)

        config.enable_stream(
            rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps,
        )
        config.enable_stream(
            rs.stream.depth, self._width, self._height, rs.format.z16, self._fps,
        )

        profile = self._pipeline.start(config)

        # Depth scale (D405 default: 0.0001m per unit, i.e. 0.1mm)
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        # Align depth → color coordinate frame
        self._align = rs.align(rs.stream.color)

        # Camera intrinsics for deprojection
        color_profile = profile.get_stream(rs.stream.color)
        self._intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

        logger.info(
            "RSCamera started: %dx%d @%dfps (depth_scale=%.6f, serial=%s)",
            self._width, self._height, self._fps, self._depth_scale,
            self._serial or "auto",
        )

    def read(self) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        """Get the latest aligned color + depth frames.

        Returns:
            (ok, color_bgr, depth_meters)
            - color_bgr: uint8 BGR image (H, W, 3)
            - depth_meters: float32 depth in meters (H, W)
        """
        if self._pipeline is None:
            return False, None, None

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
        except Exception:
            return False, None, None

        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return False, None, None

        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self._depth_scale

        return True, color, depth_m

    @property
    def intrinsics(self):
        """Camera intrinsics (pyrealsense2.intrinsics) for rs2_deproject_pixel_to_point."""
        return self._intrinsics

    @property
    def depth_scale(self) -> float:
        return self._depth_scale

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            logger.info("RSCamera stopped.")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
