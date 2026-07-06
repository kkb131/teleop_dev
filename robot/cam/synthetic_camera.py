"""합성 카메라 — D405 없이 파이프라인 테스트용.

RSColorCamera 와 동일한 start()/read()/stop() 인터페이스. 무빙 컬러바 +
프레임 카운터 + 벽시계 텍스트를 그려서 스트림 구분 / 프레임 갱신 / 육안
glass-to-glass 지연 측정이 가능하다.
"""

import time

import cv2
import numpy as np

# 컬러바 팔레트 (BGR)
_BARS = [
    (255, 255, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0),
    (255, 0, 255), (0, 0, 255), (255, 0, 0), (32, 32, 32),
]


class SyntheticCamera:
    """read() 호출 시점의 합성 프레임 생성 (fps 페이싱은 CaptureWorker 담당)."""

    def __init__(self, name: str = "syn", width: int = 640, height: int = 480,
                 fps: int = 30):
        self._name = name
        self._width = width
        self._height = height
        self._fps = fps
        self._count = 0

    def start(self) -> None:
        pass

    def read(self):
        w, h = self._width, self._height
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # 무빙 컬러바 (프레임마다 오른쪽으로 이동)
        bar_w = max(w // len(_BARS), 1)
        shift = (self._count * 4) % w
        for i, color in enumerate(_BARS):
            x0 = (i * bar_w + shift) % w
            x1 = min(x0 + bar_w, w)
            frame[:, x0:x1] = color
            if x0 + bar_w > w:  # wrap
                frame[:, 0:(x0 + bar_w) % w] = color

        # 이름 / 카운터 / 벽시계
        now = time.time()
        clock = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d}"
        cv2.rectangle(frame, (0, h // 2 - 42), (w, h // 2 + 42), (0, 0, 0), -1)
        cv2.putText(frame, self._name, (10, h // 2 - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(frame, f"#{self._count}  {clock}", (10, h // 2 + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        self._count += 1
        return True, frame

    def stop(self) -> None:
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
