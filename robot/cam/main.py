"""로봇 PC D405 다중 카메라 스트리머 — ZMQ PUB (JPEG).

D405 1~3대의 컬러 프레임을 JPEG 인코딩해 ZMQ PUB(tcp 9873)로 송신.
operator PC 의 `python3 -m sender.cam.main` 이 구독한다.

사용:
    # 기본 config (robot/cam/config/default.yaml)
    python3 -m robot.cam.main

    # 시리얼 직접 지정 (이름은 cam0, cam1, ... 자동 부여)
    python3 -m robot.cam.main --serials 218622270123 218622270456

    # D405 없이 합성 카메라 2대 (테스트)
    python3 -m robot.cam.main --synthetic 2

    # 연결된 장치 확인 / 셀프테스트
    python3 -m robot.cam.list_cameras
    python3 -m robot.cam.main --selftest
"""

import argparse
import signal
import sys
import threading
import time

from robot.cam.config import CameraEntry, RobotCamConfig
from robot.cam.publisher import CamZmqPublisher, CaptureWorker
from robot.cam.synthetic_camera import SyntheticCamera

MAX_CAMERAS = 3


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D405 multi-camera ZMQ streamer")
    p.add_argument("--config", default=None, help="YAML config 경로")
    p.add_argument("--port", type=int, default=None, help="ZMQ PUB port (default 9873)")
    p.add_argument("--jpeg-quality", type=int, default=None, help="JPEG quality 1-100")
    p.add_argument("--width", type=int, default=None, help="모든 카메라 width override")
    p.add_argument("--height", type=int, default=None, help="모든 카메라 height override")
    p.add_argument("--fps", type=int, default=None, help="모든 카메라 fps override")
    p.add_argument("--serials", nargs="+", default=None, metavar="S",
                   help="D405 시리얼 목록 (config cameras 대체, 이름 cam0..N)")
    p.add_argument("--synthetic", type=int, default=0, metavar="N",
                   help="합성 카메라 N대 (D405 불필요, 테스트용)")
    p.add_argument("--list-cameras", action="store_true", help="연결 장치 나열 후 종료")
    p.add_argument("--selftest", action="store_true", help="합성 2캠 + 인라인 SUB 검증")
    return p.parse_args()


def build_config(args: argparse.Namespace) -> RobotCamConfig:
    cfg = RobotCamConfig.load(args.config)

    if args.serials:
        cfg.cameras = [CameraEntry(name=f"cam{i}", serial=s)
                       for i, s in enumerate(args.serials)]
    if args.synthetic > 0:
        cfg.cameras = [CameraEntry(name=f"syn{i}", serial="")
                       for i in range(args.synthetic)]

    if args.port is not None:
        cfg.stream.port = args.port
    if args.jpeg_quality is not None:
        cfg.stream.jpeg_quality = args.jpeg_quality
    for cam in cfg.cameras:
        if args.width is not None:
            cam.width = args.width
        if args.height is not None:
            cam.height = args.height
        if args.fps is not None:
            cam.fps = args.fps

    if not 1 <= len(cfg.cameras) <= MAX_CAMERAS:
        print(f"[robot.cam] 카메라는 1~{MAX_CAMERAS}대 지원 (설정: {len(cfg.cameras)}대)")
        sys.exit(1)
    return cfg


def make_camera(entry: CameraEntry, synthetic: bool):
    if synthetic:
        return SyntheticCamera(entry.name, entry.width, entry.height, entry.fps)
    from robot.cam.rs_color_camera import RSColorCamera
    return RSColorCamera(entry.serial, entry.width, entry.height, entry.fps)


def run(cfg: RobotCamConfig, synthetic: bool) -> int:
    publisher = CamZmqPublisher(cfg.stream.bind_host, cfg.stream.port)
    print(f"[robot.cam] PUB bound: {publisher.endpoint} "
          f"(quality={cfg.stream.jpeg_quality})")

    stop_event = threading.Event()
    workers = []
    cameras = []
    try:
        for entry in cfg.cameras:
            cam = make_camera(entry, synthetic)
            cam.start()
            cameras.append(cam)
            w = CaptureWorker(cam, entry.name, entry.width, entry.height,
                              entry.fps, publisher, cfg.stream.jpeg_quality,
                              stop_event)
            w.start()
            workers.append(w)
            print(f"[robot.cam] streaming: {entry.name} "
                  f"{entry.width}x{entry.height}@{entry.fps} "
                  f"(serial={entry.serial or 'auto'})")

        signal.signal(signal.SIGINT, lambda *_: stop_event.set())
        signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

        while not stop_event.is_set():
            stop_event.wait(2.0)
            if stop_event.is_set():
                break
            line = "  ".join(
                f"{s['name']}: {s['fps']:.1f}fps {s['kbps']:.0f}KB/s"
                + (f" fail={s['fail']}" if s['fail'] else "")
                for s in (w.get_stats() for w in workers)
            )
            print(f"[robot.cam] {line}", flush=True)
    finally:
        stop_event.set()
        for w in workers:
            w.join(timeout=2.0)
        for cam in cameras:
            cam.stop()
        publisher.close()
        print("[robot.cam] stopped.")
    return 0


def _selftest() -> int:
    """합성 카메라 2대 publish + 인라인 SUB 로 topic당 ≥3프레임 unpack 검증."""
    import zmq

    from protocol.cam_protocol import unpack_frame

    port = 19873  # 셀프테스트 전용 포트 (기본 9873 점유 회피)
    cfg = RobotCamConfig()
    cfg.stream.port = port
    cfg.cameras = [CameraEntry(name="syn0"), CameraEntry(name="syn1")]

    publisher = CamZmqPublisher(cfg.stream.bind_host, port)
    stop_event = threading.Event()
    workers = []
    for entry in cfg.cameras:
        cam = SyntheticCamera(entry.name, entry.width, entry.height, entry.fps)
        w = CaptureWorker(cam, entry.name, entry.width, entry.height, entry.fps,
                          publisher, cfg.stream.jpeg_quality, stop_event)
        w.start()
        workers.append(w)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 8)   # 1이면 다중 topic 기아 — publisher.py docstring 참조
    sub.connect(f"tcp://127.0.0.1:{port}")
    for entry in cfg.cameras:
        sub.setsockopt_string(zmq.SUBSCRIBE, entry.name)

    counts = {c.name: 0 for c in cfg.cameras}
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    deadline = time.monotonic() + 5.0
    rc = 0
    try:
        while time.monotonic() < deadline and min(counts.values()) < 3:
            if not poller.poll(timeout=200):
                continue
            frame = unpack_frame(sub.recv_multipart())
            assert frame is not None, "unpack_frame 실패"
            assert frame.jpeg[:2] == b"\xff\xd8", "JPEG magic 불일치"
            assert frame.width == 640 and frame.height == 480
            counts[frame.name] += 1
        if min(counts.values()) >= 3:
            print(f"[selftest] PASS — frames per topic: {counts}")
        else:
            print(f"[selftest] FAIL — 5초 내 topic당 3프레임 미달: {counts}")
            rc = 1
    finally:
        stop_event.set()
        for w in workers:
            w.join(timeout=2.0)
        sub.close(linger=0)
        publisher.close()
    return rc


def main() -> int:
    args = parse_args()

    if args.list_cameras:
        from robot.cam.list_cameras import main as list_main
        return list_main()
    if args.selftest:
        return _selftest()

    cfg = build_config(args)
    return run(cfg, synthetic=args.synthetic > 0)


if __name__ == "__main__":
    sys.exit(main())
