"""ZMQ SUB 수신 스레드 — 카메라별 최신 프레임 슬롯.

로봇 PC 의 robot/cam PUB(tcp 9873) 를 구독해 카메라별 LatestSlot 에
최신 JPEG 만 유지한다. poll 후 NOBLOCK drain + LatestSlot 으로 topic 별
최신만 반영하므로 큐잉 지연 없음. RCVHWM 은 **1로 두면 안 됨** — HWM 은
topic 구분 없는 파이프 전체 한도라 다중 카메라에서 특정 topic 기아를
유발할 수 있다 (robot/cam/publisher.py docstring 의 SNDHWM 설명 참조).
CONFLATE 은 multipart/다중 topic 과 비호환이라 미사용.
"""

import threading
import time
from typing import Dict, List, Optional, Tuple

import zmq

from protocol.cam_protocol import unpack_frame


class LatestSlot:
    """카메라 1대의 최신 프레임 (bytes 는 immutable 이라 핸드오프 안전)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._seq: int = -1
        self._ts: float = 0.0
        self._jpeg: Optional[bytes] = None
        self._recv_count = 0        # 통계용 (2초 주기 리셋)
        self._recv_bytes = 0

    def put(self, seq: int, ts: float, jpeg: bytes) -> None:
        with self._lock:
            self._seq = seq
            self._ts = ts
            self._jpeg = jpeg
            self._recv_count += 1
            self._recv_bytes += len(jpeg)

    def get(self) -> Tuple[int, float, Optional[bytes]]:
        with self._lock:
            return self._seq, self._ts, self._jpeg

    def pop_stats(self) -> Tuple[int, int, float]:
        """(count, bytes, latest_ts) 반환 후 카운터 리셋."""
        with self._lock:
            c, b, ts = self._recv_count, self._recv_bytes, self._ts
            self._recv_count = 0
            self._recv_bytes = 0
        return c, b, ts


class CamReceiver(threading.Thread):
    """SUB + Poller 루프. slots[name] 에 최신 프레임 유지."""

    def __init__(self, robot_ip: str, port: int, cameras: List[str]):
        super().__init__(name="CamReceiver", daemon=True)
        self._endpoint = f"tcp://{robot_ip}:{port}"
        self._cameras = list(cameras)
        self.slots: Dict[str, LatestSlot] = {name: LatestSlot() for name in cameras}
        self._stop = threading.Event()
        self._stats_t0 = time.monotonic()

    def run(self) -> None:
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVHWM, 8)   # 1이면 다중 topic 기아 — 모듈 docstring 참조
        sub.setsockopt(zmq.LINGER, 0)
        sub.connect(self._endpoint)
        for name in self._cameras:
            sub.setsockopt_string(zmq.SUBSCRIBE, name)
        print(f"[CamReceiver] SUB connected: {self._endpoint} "
              f"cameras={self._cameras}")

        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        try:
            while not self._stop.is_set():
                if not poller.poll(timeout=200):
                    continue
                # drain: 읽을 수 있는 만큼 다 읽고 topic별 최신만 반영
                while True:
                    try:
                        parts = sub.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    frame = unpack_frame(parts)
                    if frame is None:
                        continue
                    slot = self.slots.get(frame.name)
                    if slot is not None:
                        slot.put(frame.seq, frame.ts, frame.jpeg)
        finally:
            sub.close(linger=0)

    def stop(self) -> None:
        self._stop.set()

    def get_stats(self) -> List[dict]:
        """카메라별 fps / KB/s / latency(수신시각 − capture ts)."""
        dt = max(time.monotonic() - self._stats_t0, 1e-6)
        self._stats_t0 = time.monotonic()
        now = time.time()
        out = []
        for name in self._cameras:
            count, nbytes, ts = self.slots[name].pop_stats()
            out.append({
                "name": name,
                "fps": count / dt,
                "kbps": nbytes / dt / 1024.0,
                # 로봇/조종 PC 시계 차가 포함된 근사값 (NTP 동기 가정)
                "latency_ms": (now - ts) * 1000.0 if ts > 0 else -1.0,
            })
        return out
