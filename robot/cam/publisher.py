"""ZMQ PUB publisher + 카메라별 capture worker.

CamZmqPublisher
    PUB 소켓 1개를 여러 CaptureWorker 스레드가 공유. ZMQ 소켓은 thread-unsafe
    라 publish() 를 lock 으로 보호한다. ≤3캠 × 30fps × ~50KB 면 lock 경합은
    무시 가능 — teleimager 의 queue+publisher-thread 구조보다 홉 하나가 적다.

    SNDHWM=8: 구독자별 파이프 깊이. **1로 두면 안 됨** — HWM 은 topic 구분 없이
    파이프 전체에 걸리는데, 워커 2개가 같은 lock 으로 back-to-back publish 하면
    (위상이 겹치는 구간) 두 번째 topic 이 io 스레드가 첫 메시지를 빼내기 전에
    도착해 매 사이클 drop → 한 카메라만 0fps 로 통째 기아. 실카메라는 클럭
    drift 로 위상이 이동하며 수십 분씩 이 상태가 지속될 수 있다.
    최신 프레임만 쓰는 목표는 수신측 LatestSlot (drain 후 topic 별 최신 유지)
    이 담당하므로 HWM 은 topic 수 대비 여유만 있으면 된다.
    CONFLATE 은 multipart / 다중 topic 과 비호환이라 쓰지 않는다.

CaptureWorker
    카메라당 1 스레드: capture → cv2.imencode(JPEG) → publish.
    pyrealsense2 wait_for_frames 가 파이프라인별 blocking 이라 스레드 분리는
    필수. monotonic 고정 주기 페이싱 + fps/bytes 통계.
"""

import threading
import time

import cv2
import zmq

from protocol.cam_protocol import pack_frame


class CamZmqPublisher:
    """PUB 소켓 소유 + thread-safe publish."""

    def __init__(self, bind_host: str = "0.0.0.0", port: int = 9873):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 8)   # 1이면 다중 topic 기아 — docstring 참조
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(f"tcp://{bind_host}:{port}")
        self._lock = threading.Lock()
        self.endpoint = f"tcp://{bind_host}:{port}"

    def publish(self, name: str, seq: int, ts: float, width: int, height: int,
                fps: int, quality: int, jpeg: bytes) -> None:
        parts = pack_frame(name, seq, ts, width, height, fps, quality, jpeg)
        with self._lock:
            self._sock.send_multipart(parts)

    def close(self) -> None:
        with self._lock:
            self._sock.close(linger=0)


class CaptureWorker(threading.Thread):
    """카메라 1대 캡처 → JPEG 인코딩 → publish 루프."""

    def __init__(self, camera, name: str, width: int, height: int, fps: int,
                 publisher: CamZmqPublisher, jpeg_quality: int,
                 stop_event: threading.Event):
        super().__init__(name=f"cam-{name}", daemon=True)
        self._camera = camera
        self._cam_name = name
        self._width = width
        self._height = height
        self._fps = fps
        self._publisher = publisher
        self._quality = jpeg_quality
        # threading.Thread 내부 메서드 _stop() 을 가리면 join() 이
        # TypeError('Event' object is not callable) 로 죽는다 — 이름 충돌 금지.
        self._stop_event = stop_event

        self._seq = 0
        # 통계 (get_stats 로 조회, 2초 주기 리셋)
        self._stats_lock = threading.Lock()
        self._stat_frames = 0
        self._stat_bytes = 0
        self._stat_fail = 0
        self._stat_t0 = time.monotonic()

    def run(self) -> None:
        period = 1.0 / max(self._fps, 1)
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._quality]
        next_t = time.monotonic()

        while not self._stop_event.is_set():
            ok, frame = self._camera.read()
            if ok and frame is not None:
                ok_enc, buf = cv2.imencode(".jpg", frame, encode_params)
                if ok_enc:
                    jpeg = buf.tobytes()
                    self._publisher.publish(
                        self._cam_name, self._seq, time.time(),
                        self._width, self._height, self._fps, self._quality, jpeg,
                    )
                    self._seq += 1
                    with self._stats_lock:
                        self._stat_frames += 1
                        self._stat_bytes += len(jpeg)
                else:
                    with self._stats_lock:
                        self._stat_fail += 1
            else:
                with self._stats_lock:
                    self._stat_fail += 1

            # 고정 주기 페이싱 (실제 카메라는 wait_for_frames 가 이미 페이싱하므로
            # sleep 이 0 에 수렴; 합성 카메라는 이 sleep 이 fps 를 만든다)
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                next_t = time.monotonic()   # 밀리면 리셋 (버스트 방지)

    def get_stats(self) -> dict:
        with self._stats_lock:
            dt = max(time.monotonic() - self._stat_t0, 1e-6)
            stats = {
                "name": self._cam_name,
                "fps": self._stat_frames / dt,
                "kbps": self._stat_bytes / dt / 1024.0,
                "fail": self._stat_fail,
                "seq": self._seq,
            }
            self._stat_frames = 0
            self._stat_bytes = 0
            self._stat_fail = 0
            self._stat_t0 = time.monotonic()
        return stats
