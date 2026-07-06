"""RealSense D405 컬러 전용 카메라 래퍼 (pyrealsense2).

sender/hand/realsense/rs_camera.py 를 모델로 하되 depth/align 을 제거한
스트리밍 전용 버전. robot/ 은 sender/ 를 import 하지 않는다는 규칙 때문에
별도 구현 (depth 가 없어 USB 대역폭도 절반).
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)


def list_realsense_serials() -> list:
    """연결된 RealSense 장치의 (serial, name) 리스트."""
    import pyrealsense2 as rs

    ctx = rs.context()
    out = []
    for dev in ctx.query_devices():
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        out.append((serial, name))
    return out


class RSColorCamera:
    """pyrealsense2 컬러 전용 파이프라인. read() -> (ok, bgr)."""

    def __init__(
        self,
        serial: str = "",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        self._serial = serial or None
        self._width = width
        self._height = height
        self._fps = fps
        self._pipeline = None

    def start(self) -> None:
        import pyrealsense2 as rs

        self._pipeline = rs.pipeline()
        config = rs.config()
        if self._serial:
            config.enable_device(self._serial)
        config.enable_stream(
            rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps,
        )
        self._pipeline.start(config)
        logger.info(
            "RSColorCamera started: %dx%d @%dfps (serial=%s)",
            self._width, self._height, self._fps, self._serial or "auto",
        )

    def read(self):
        """최신 컬러 프레임. Returns (ok, bgr uint8 (H,W,3) | None)."""
        if self._pipeline is None:
            return False, None
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
        except Exception:
            return False, None
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        return True, np.asanyarray(color_frame.get_data())

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            logger.info("RSColorCamera stopped.")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
