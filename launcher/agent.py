"""robot PC 용 headless agent 데몬 — HTTP API 로 컴포넌트 제어.

조종 PC 의 GUI (launcher/gui.py → RemoteManager) 가 이 API 로 robot PC 의
컴포넌트를 원격 시작/정지/모니터링한다. 표준 라이브러리만 사용.

    python3 -m launcher agent --config launcher/config/teleop_system.yaml

API (모두 JSON):
    GET  /api/health                          → {"ok": true, "groups": [...]}
    GET  /api/components                      → [status, ...]
    POST /api/components/<name>/start         → status
    POST /api/components/<name>/stop          → status
    POST /api/components/<name>/input         body {"text": "r"} → {"ok": bool}
    GET  /api/components/<name>/log?since=N&max=200
                                              → {"lines": [[seq, line],...]}
    POST /api/start_all                       body {"names": [...]}? → {"started": [...]}
    POST /api/stop_all                        body {"names": [...]}? → {"stopped": [...]}

인증: config agent.token 이 비어있지 않으면 X-Auth-Token 헤더 필수.
      (신뢰 LAN 밖이라면 반드시 token + 방화벽 사용)
"""

from __future__ import annotations

import json
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from launcher.config import LauncherConfig
from launcher.manager import LocalManager


class AgentHandler(BaseHTTPRequestHandler):
    manager: LocalManager = None      # serve() 에서 주입
    token: str = ""
    protocol_version = "HTTP/1.1"

    # ── helpers ─────────────────────────────────────────────────────────

    def _json(self, code: int, payload) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _auth_ok(self) -> bool:
        if not self.token:
            return True
        return self.headers.get("X-Auth-Token", "") == self.token

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}

    def log_message(self, fmt, *args):  # 기본 stderr 액세스 로그 억제
        pass

    # ── routes ──────────────────────────────────────────────────────────

    def do_GET(self):
        if not self._auth_ok():
            return self._json(401, {"error": "bad token"})
        url = urlparse(self.path)
        parts = [p for p in url.path.split("/") if p]

        if parts == ["api", "health"]:
            return self._json(200, {"ok": True,
                                    "components": self.manager.names()})
        if parts == ["api", "components"]:
            return self._json(200, self.manager.status_all())
        if len(parts) == 4 and parts[:2] == ["api", "components"] and parts[3] == "log":
            name = parts[2]
            if name not in self.manager.names():
                return self._json(404, {"error": f"unknown component {name}"})
            q = parse_qs(url.query)
            since = int(q.get("since", ["0"])[0])
            max_lines = min(int(q.get("max", ["200"])[0]), 1000)
            lines = self.manager.log_since(name, since, max_lines)
            return self._json(200, {"lines": lines})
        return self._json(404, {"error": "not found"})

    def do_POST(self):
        if not self._auth_ok():
            return self._json(401, {"error": "bad token"})
        url = urlparse(self.path)
        parts = [p for p in url.path.split("/") if p]

        if parts == ["api", "start_all"]:
            names = self._read_body().get("names")
            return self._json(200, {"started": self.manager.start_all(names)})
        if parts == ["api", "stop_all"]:
            names = self._read_body().get("names")
            return self._json(200, {"stopped": self.manager.stop_all(names)})
        if len(parts) == 4 and parts[:2] == ["api", "components"]:
            name, action = parts[2], parts[3]
            if name not in self.manager.names():
                return self._json(404, {"error": f"unknown component {name}"})
            if action == "start":
                return self._json(200, self.manager.start(name))
            if action == "stop":
                return self._json(200, self.manager.stop(name))
            if action == "input":
                text = self._read_body().get("text", "")
                ok = self.manager.send_input(name, text)
                return self._json(200, {"ok": ok})
        return self._json(404, {"error": "not found"})


def serve(config: LauncherConfig, groups=("robot",)) -> int:
    manager = LocalManager(config, groups=list(groups))
    AgentHandler.manager = manager
    AgentHandler.token = config.agent.token

    httpd = ThreadingHTTPServer((config.agent.host, config.agent.port), AgentHandler)

    stop_once = threading.Event()

    def shutdown(signum, frame):
        if stop_once.is_set():
            return
        stop_once.set()
        print("\n[agent] 종료 — 모든 컴포넌트 정지 중...")
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"[agent] listening on http://{config.agent.host}:{config.agent.port}"
          f"  (groups={list(groups)}, token={'set' if config.agent.token else 'none'})")
    print(f"[agent] components: {', '.join(manager.names()) or '(없음)'}")
    try:
        httpd.serve_forever()
    finally:
        stopped = manager.stop_all()
        if stopped:
            print(f"[agent] stopped: {', '.join(stopped)}")
        httpd.server_close()
        print("[agent] exit")
    return 0
