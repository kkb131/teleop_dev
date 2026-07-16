#!/usr/bin/env python3
"""다중 카메라 topic 기아(starvation) 회귀 테스트 (카메라/네트워크 불필요).

배경: CamZmqPublisher 가 SNDHWM=1 이던 시절, 워커 2개가 같은 lock 으로
back-to-back publish 하면 (두 카메라의 프레임 위상이 겹치는 구간) 두 번째
메시지가 io 스레드 drain 전에 도착해 매 사이클 drop → 한 topic 만 0fps.
실기 증상: 로봇PC stats 는 두 캠 30fps 정상, 조종PC 는 한 캠만 0fps.

이 테스트는 최악 케이스(위상 완전 정렬)를 시뮬레이션한다:
  - 워커 스레드 2개가 매 사이클 Barrier 로 랑데부 후 같은 lock 을 잡고
    연달아 publish (실제 CaptureWorker + CamZmqPublisher._lock 구조와 동일)
  - SUB 는 poll + NOBLOCK drain (CamReceiver 와 동일 패턴)
  - SNDHWM=8 (수정값): 두 topic 모두 수신돼야 PASS
  - SNDHWM=1 (버그값): 참고용 카운트만 출력 (드랍 여부는 io 스레드 타이밍
    의존이라 assert 하지 않음)

Usage: python3 -m robot.cam.tests.test_multicam_starvation
"""

import sys
import threading
import time

import zmq

from protocol.cam_protocol import pack_frame, unpack_frame

PORT = 19875           # 다른 selftest 포트(19873/19874)와 충돌 회피
CYCLES = 60            # 30Hz × 2초
PAYLOAD = b"\xff\xd8" + b"\x00" * 30_000   # 실제 JPEG 크기 근사 (~30KB)
TOPICS = ("camA", "camB")


def _run_scenario(sndhwm: int) -> dict:
    """위상 정렬된 2-topic publish 시나리오 실행, topic별 수신 카운트 반환."""
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, sndhwm)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f"tcp://127.0.0.1:{PORT}")

    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 8)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(f"tcp://127.0.0.1:{PORT}")
    for t in TOPICS:
        sub.setsockopt_string(zmq.SUBSCRIBE, t)
    time.sleep(0.3)  # slow-joiner: 구독 전파 대기

    lock = threading.Lock()          # CamZmqPublisher._lock 역할
    barrier = threading.Barrier(len(TOPICS))
    stop = threading.Event()

    def worker(topic: str):
        for seq in range(CYCLES):
            if stop.is_set():
                return
            try:
                barrier.wait(timeout=2.0)   # 위상 완전 정렬 (최악 케이스)
            except threading.BrokenBarrierError:
                return
            with lock:                       # back-to-back publish
                pub.send_multipart(pack_frame(
                    topic, seq, time.time(), 640, 480, 30, 80, PAYLOAD))
            time.sleep(1.0 / 30)

    threads = [threading.Thread(target=worker, args=(t,), daemon=True)
               for t in TOPICS]
    for th in threads:
        th.start()

    counts = {t: 0 for t in TOPICS}
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    deadline = time.monotonic() + CYCLES / 30 + 2.0
    while time.monotonic() < deadline and any(th.is_alive() for th in threads):
        if not poller.poll(timeout=100):
            continue
        while True:
            try:
                parts = sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            frame = unpack_frame(parts)
            if frame is not None:
                counts[frame.name] += 1

    stop.set()
    barrier.abort()
    for th in threads:
        th.join(timeout=1.0)
    sub.close(linger=0)
    pub.close(linger=0)
    return counts


def main() -> int:
    print("=" * 56)
    print("  Multi-cam topic starvation regression (SNDHWM)")
    print("=" * 56)

    # 참고: 버그값 재현 시도 (타이밍 의존 — assert 없음)
    c1 = _run_scenario(sndhwm=1)
    print(f"[ref ] SNDHWM=1: {c1}  (불균형/기아 가능 — 참고용)")

    # 수정값: 두 topic 모두 충분히 수신돼야 함
    c8 = _run_scenario(sndhwm=8)
    print(f"[test] SNDHWM=8: {c8}")
    threshold = CYCLES // 2
    ok = all(c8[t] >= threshold for t in TOPICS)
    if ok:
        print(f"[TEST] 두 topic 모두 >= {threshold} 프레임 수신 ... PASS")
        return 0
    print(f"[TEST] FAIL — topic 기아 잔존: {c8} (threshold={threshold})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
