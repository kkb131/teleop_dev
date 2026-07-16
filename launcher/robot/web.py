#!/usr/bin/env python3
"""로봇 PC 웹 대시보드 데몬.

    python3 -m launcher.robot.web [--config launcher/robot/config/robot.yaml]
                                  [--host 0.0.0.0] [--port 9876]

- 조종 PC 브라우저에서 http://<robot_pc_ip>:9876/ 접속.
- 이 데몬이 로봇 구성요소(팔/손/캠) 프로세스의 유일한 관리자
  (bringup CLI 와 flock 상호배제). 데몬 종료 시 관리 중인 자식 전체 정지
  (고아 프로세스 방지) — 상시 운용은 tmux/systemd 권장.
- web.tls_cert + web.tls_key 지정 시 HTTPS.
"""

from __future__ import annotations

import argparse
import signal
import ssl
import sys
import threading
from http.server import ThreadingHTTPServer

from launcher.manager import LocalManager
from launcher.robot.actions import ActionRunner
from launcher.robot.api import RobotApiHandler
from launcher.robot.lock import LauncherLock, acquire_or_exit_message
from launcher.robot.parts import load_robot_config
from launcher.robot.status import HwProber, StatusCollector


def main() -> int:
    parser = argparse.ArgumentParser(description="로봇 PC 웹 대시보드")
    parser.add_argument("--config", default=None,
                        help="robot.yaml 경로 (default: launcher/robot/config/robot.yaml)")
    parser.add_argument("--host", default=None, help="bind host override")
    parser.add_argument("--port", type=int, default=None, help="bind port override")
    args = parser.parse_args()

    cfg = load_robot_config(args.config)
    web = cfg.launcher.web
    host = args.host or web.host
    port = args.port or web.port

    lock = LauncherLock()
    msg = acquire_or_exit_message(lock)
    if msg:
        print(f"[web] {msg}")
        return 1

    manager = LocalManager(cfg.launcher, groups=["robot"])
    prober = HwProber({p.name: p.probe for p in cfg.parts if p.probe})
    prober.start()
    actions = ActionRunner(cfg, manager)
    collector = StatusCollector(cfg, manager, prober)

    RobotApiHandler.cfg = cfg
    RobotApiHandler.manager = manager
    RobotApiHandler.collector = collector
    RobotApiHandler.actions = actions
    RobotApiHandler.token = web.token

    httpd = ThreadingHTTPServer((host, port), RobotApiHandler)

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
        print("\n[web] 종료 — 관리 중인 컴포넌트 전체 정지...")
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"[web] {scheme}://{host}:{port}/  "
          f"(config={cfg.config_path}, token={'set' if web.token else 'none'})")
    print(f"[web] parts: {', '.join(p.name for p in cfg.parts)}")
    print("[web] 조종 PC 브라우저에서 접속하세요. Ctrl+C 로 종료 (자식 전체 정지)")

    try:
        httpd.serve_forever()
    finally:
        actions.stop_oneshots()
        stopped = manager.stop_all()
        if stopped:
            print(f"[web] stopped: {', '.join(stopped)}")
        prober.stop()
        httpd.server_close()
        lock.release()
        print("[web] exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
