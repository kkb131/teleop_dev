#!/usr/bin/env python3
"""조종 PC 웹 대시보드 데몬 (포트 9877).

    python3 -m launcher.sender.web [--config launcher/sender/config/sender.yaml]

- 브라우저: http://<조종 PC IP>:9877/ — 헤드셋 Chrome 에서도
  http://localhost:9877/ (adb reverse 9877 후) 로 접속 가능.
- 이 데몬이 조종측 프로세스(XR 러너/cam viewer)의 유일한 관리자
  (bringup CLI 와 flock 상호배제, robot 데몬과는 별개 lock — 동시 실행 가능).
- 데몬 종료 시 관리 중 자식 전체 정지.
"""

from __future__ import annotations

import argparse
import signal
import ssl
import sys
import threading
from http.server import ThreadingHTTPServer

from launcher.lock import LauncherLock, acquire_or_exit_message
from launcher.manager import LocalManager
from launcher.probe import HwProber
from launcher.sender.actions import RunnerControl, SessionRunner
from launcher.sender.adb import AdbHelper, AdbWatcher
from launcher.sender.api import SenderApiHandler
from launcher.sender.parts import load_sender_config
from launcher.sender.status import SenderStatusCollector

LOCK_PATH = "/tmp/teleop_sender_launcher.lock"


def main() -> int:
    parser = argparse.ArgumentParser(description="조종 PC 웹 대시보드")
    parser.add_argument("--config", default=None,
                        help="sender.yaml 경로 (default: launcher/sender/config/sender.yaml)")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()

    cfg = load_sender_config(args.config)
    web = cfg.launcher.web
    host = args.host or web.host
    port = args.port or web.port

    lock = LauncherLock(LOCK_PATH)
    msg = acquire_or_exit_message(lock, f"http://localhost:{port}/api/health")
    if msg:
        print(f"[sender.web] {msg}")
        return 1

    manager = LocalManager(cfg.launcher, groups=["operator"])
    adb_helper = AdbHelper(cfg.adb.binary)
    adb_watcher = AdbWatcher(adb_helper, cfg.adb.reverse_ports,
                             cfg.adb.probe_interval_s)
    adb_watcher.start()
    prober = HwProber({
        f"bridge_{cfg.bridge_port}": ("127.0.0.1", cfg.bridge_port),
        f"cam_{cfg.cam_http_port}": ("127.0.0.1", cfg.cam_http_port),
    })
    prober.start()
    runner = RunnerControl(cfg, manager)
    session = SessionRunner(cfg, manager, runner, adb_helper)
    collector = SenderStatusCollector(cfg, manager, adb_watcher, prober,
                                      current_mode=runner.current_mode,
                                      session_state=session.state)

    SenderApiHandler.cfg = cfg
    SenderApiHandler.manager = manager
    SenderApiHandler.collector = collector
    SenderApiHandler.runner = runner
    SenderApiHandler.session = session
    SenderApiHandler.adb = adb_helper
    SenderApiHandler.token = web.token

    httpd = ThreadingHTTPServer((host, port), SenderApiHandler)

    scheme = "http"
    if web.tls_cert and web.tls_key:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(certfile=web.tls_cert, keyfile=web.tls_key)
        httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
        scheme = "https"

    stop_once = threading.Event()

    def shutdown(signum, frame):
        if stop_once.is_set():
            return
        stop_once.set()
        print("\n[sender.web] 종료 — 관리 중인 컴포넌트 전체 정지...")
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"[sender.web] {scheme}://{host}:{port}/  "
          f"(config={cfg.config_path}, token={'set' if web.token else 'none'})")
    print(f"[sender.web] runners: {', '.join(r.name for r in cfg.runners)}")
    print("[sender.web] 헤드셋 브라우저 접속: adb reverse 후 http://localhost:9877/")

    try:
        httpd.serve_forever()
    finally:
        session.cancel()
        stopped = manager.stop_all()
        if stopped:
            print(f"[sender.web] stopped: {', '.join(stopped)}")
        adb_watcher.stop()
        prober.stop()
        httpd.server_close()
        lock.release()
        print("[sender.web] exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
