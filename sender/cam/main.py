"""조종 PC 카메라 수신 + 브라우저/VR 서빙 진입점.

로봇 PC robot/cam 의 ZMQ 스트림을 구독해:
  * http://localhost:8014/  — 브라우저 그리드 뷰
  * ws://localhost:8014/ws  — WebXR 텔레옵 세션 (webxr_to_pose.html) 용 프레임 push

사용:
    python3 -m sender.cam.main --robot-ip 192.168.0.2
    python3 -m sender.cam.main --robot-ip 127.0.0.1 --cameras syn0 syn1   # 합성 테스트
    python3 -m sender.cam.main --selftest

VR 통합: sender/xr_common/config.yaml 의 cam.enabled: true 로 설정하면
webxr_to_pose.html 이 이 서버의 /config, /ws 에 접속한다.
헤드셋 USB 모드: adb reverse tcp:8014 tcp:8014 (8013 과 별개로 추가).

포트 역할 (이 프로세스는 8013 을 열지 않는다 — 단독 실행 시 8013 미접속은 정상):
  * 8014 (이 프로세스)      — 브라우저 그리드 뷰 + 프레임 WS + /config
  * 8013 (BridgePoseStore) — XR 페이지(webxr_to_pose.html) 서빙 + pose WS.
    sender.hand.xr_hand_sender / sender.arm.xr_arm_sender / scripts/run_xr_teleop.py
    가 띄운다. 헤드셋 XR 테스트는 이 중 하나와 함께 실행할 것.
"""

import argparse
import signal
import sys
import threading
import time

from sender.cam.config import SenderCamConfig
from sender.cam.http_server import CamHttpServer
from sender.cam.receiver import CamReceiver


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="camera stream receiver + HTTP/WS server")
    p.add_argument("--config", default=None, help="YAML config 경로")
    p.add_argument("--robot-ip", default=None, help="로봇 PC IP")
    p.add_argument("--port", type=int, default=None, help="ZMQ SUB port (default 9873)")
    p.add_argument("--http-port", type=int, default=None, help="HTTP/WS port (default 8014)")
    p.add_argument("--cameras", nargs="+", default=None, metavar="NAME",
                   help="구독 카메라 이름 목록 (robot 측 name 과 일치)")
    p.add_argument("--mode", choices=["head_locked", "world_locked"], default=None,
                   help="VR plane 고정 모드")
    p.add_argument("--selftest", action="store_true",
                   help="인라인 PUB + 전체 경로 (slot/config/snapshot/ws) 검증")
    return p.parse_args()


def build_config(args: argparse.Namespace) -> SenderCamConfig:
    cfg = SenderCamConfig.load(args.config)
    if args.robot_ip is not None:
        cfg.zmq.robot_ip = args.robot_ip
    if args.port is not None:
        cfg.zmq.port = args.port
    if args.http_port is not None:
        cfg.http.port = args.http_port
    if args.cameras is not None:
        cfg.zmq.cameras = args.cameras
    if args.mode is not None:
        cfg.vr.mode = args.mode
    return cfg


def run(cfg: SenderCamConfig) -> int:
    receiver = CamReceiver(cfg.zmq.robot_ip, cfg.zmq.port, cfg.zmq.cameras)
    receiver.start()
    server = CamHttpServer(cfg, receiver)
    server.start()

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    print(f"[sender.cam] vr.mode={cfg.vr.mode}  브라우저 뷰: "
          f"http://localhost:{server.port}/")
    while not stop_event.is_set():
        stop_event.wait(2.0)
        if stop_event.is_set():
            break
        line = "  ".join(
            f"{s['name']}: {s['fps']:.1f}fps {s['kbps']:.0f}KB/s "
            f"lat={s['latency_ms']:.0f}ms"
            for s in receiver.get_stats()
        )
        print(f"[sender.cam] {line}", flush=True)

    receiver.stop()
    print("[sender.cam] stopped.")
    return 0


def _selftest() -> int:
    """인라인 ZMQ PUB → receiver/config/snapshot/ws 전체 경로 검증.

    robot.cam 을 import 하지 않는다 (robot/↔sender/ import 금지 규칙) —
    최소 PUB 를 여기서 직접 만든다.
    """
    import json
    import urllib.request

    import cv2
    import numpy as np
    import zmq

    from protocol.cam_protocol import pack_frame, unpack_ws_frame

    zmq_port = 19874   # 셀프테스트 전용 포트
    http_port = 18014

    # ── 인라인 PUB (테스트 프레임 30fps) ────────────────────────────────
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 1)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f"tcp://127.0.0.1:{zmq_port}")
    stop = threading.Event()

    def pub_loop():
        seq = 0
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        while not stop.is_set():
            img[:] = (seq * 5) % 255
            cv2.putText(img, f"selftest #{seq}", (10, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            _, buf = cv2.imencode(".jpg", img)
            pub.send_multipart(pack_frame(
                "head", seq, time.time(), 640, 480, 30, 80, buf.tobytes()))
            seq += 1
            stop.wait(1.0 / 30)

    pub_thread = threading.Thread(target=pub_loop, daemon=True)
    pub_thread.start()

    # ── 수신 + 서버 기동 ────────────────────────────────────────────────
    cfg = SenderCamConfig()
    cfg.zmq.robot_ip = "127.0.0.1"
    cfg.zmq.port = zmq_port
    cfg.zmq.cameras = ["head"]
    cfg.http.port = http_port

    receiver = CamReceiver(cfg.zmq.robot_ip, cfg.zmq.port, cfg.zmq.cameras)
    receiver.start()
    server = CamHttpServer(cfg, receiver)
    server.start()

    rc = 0
    try:
        # 1) slot 채워짐
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            seq, _ts, jpeg = receiver.slots["head"].get()
            if jpeg is not None:
                break
            time.sleep(0.05)
        assert jpeg is not None, "5초 내 slot 미채움"
        print(f"[selftest] receiver slot OK (seq={seq}, {len(jpeg)}B)")

        # 2) /config + CORS 헤더
        with urllib.request.urlopen(
                f"http://127.0.0.1:{http_port}/config", timeout=2.0) as r:
            assert r.headers.get("Access-Control-Allow-Origin") == "*", "CORS 헤더 없음"
            c = json.loads(r.read())
        assert c["cameras"] == ["head"] and "vr" in c, c
        print(f"[selftest] /config OK (CORS, vr.mode={c['vr']['mode']})")

        # 3) /snapshot JPEG magic
        with urllib.request.urlopen(
                f"http://127.0.0.1:{http_port}/snapshot/head", timeout=2.0) as r:
            body = r.read()
        assert body[:2] == b"\xff\xd8", "JPEG magic 불일치"
        print(f"[selftest] /snapshot/head OK ({len(body)}B)")

        # 4) WS binary ≥3프레임 unpack
        import asyncio

        import aiohttp

        async def ws_check():
            n = 0
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(
                        f"http://127.0.0.1:{http_port}/ws") as ws:
                    async for msg in ws:
                        if msg.type != aiohttp.WSMsgType.BINARY:
                            continue
                        parsed = unpack_ws_frame(msg.data)
                        assert parsed is not None, "unpack_ws_frame 실패"
                        cam_index, _seq, _ts, jpeg = parsed
                        assert cam_index == 0 and jpeg[:2] == b"\xff\xd8"
                        n += 1
                        if n >= 3:
                            return n
            return n

        n = asyncio.new_event_loop().run_until_complete(
            asyncio.wait_for(ws_check(), timeout=5.0))
        print(f"[selftest] /ws OK ({n} binary frames)")
        print("[selftest] PASS")
    except Exception as e:
        print(f"[selftest] FAIL: {e}")
        rc = 1
    finally:
        stop.set()
        pub_thread.join(timeout=2.0)
        receiver.stop()
        pub.close(linger=0)
    return rc


def main() -> int:
    args = parse_args()
    if args.selftest:
        return _selftest()
    return run(build_config(args))


if __name__ == "__main__":
    sys.exit(main())
