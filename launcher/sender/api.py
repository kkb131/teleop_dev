"""조종 PC 웹 대시보드 HTTP 핸들러.

라우트:
    GET  /                        index.html
    GET  /params                  params.html
    GET  /assets/<name>           app.js / style.css (화이트리스트)

    GET  /api/health              {ok, runners, components}
    GET  /api/status              SenderStatusCollector.snapshot()
    GET  /api/components/<n>/log?since=N&max=200
    POST /api/runner/start        {mode} → 200 | 409 {running_mode}
    POST /api/runner/stop
    POST /api/runner/key          {key: r|p|c|space|+|-|x} → 200 | 400 | 409
    POST /api/cam/start | /api/cam/stop
    POST /api/adb/reverse         → 포트별 결과 (+출력 캡처)
    POST /api/adb/restart         kill-server → start-server
    GET  /api/session             세션 상태
    POST /api/session/start       {mode?, skip_adb?} → 200 | 409
    POST /api/session/cancel
    GET  /api/params/xr-dual
    PUT  /api/params/xr-dual      {values: {...}}

인증: web.token 설정 시 X-Auth-Token 또는 ?token= (API 만).
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from launcher.manager import LocalManager
from launcher import params_engine
from launcher.sender.actions import RunnerControl, SessionRunner
from launcher.sender.adb import AdbHelper
from launcher.sender.params_schema import SCHEMAS
from launcher.sender.parts import SenderConfig
from launcher.sender.status import SenderStatusCollector

_ASSETS_DIR = Path(__file__).parent / "assets"
_PAGES = {"": "index.html", "params": "params.html"}
_ASSET_TYPES = {
    "app.js": "application/javascript; charset=utf-8",
    "style.css": "text/css; charset=utf-8",
}


class SenderApiHandler(BaseHTTPRequestHandler):
    # web.py serve() 에서 주입
    cfg: SenderConfig = None
    manager: LocalManager = None
    collector: SenderStatusCollector = None
    runner: RunnerControl = None
    session: SessionRunner = None
    adb: AdbHelper = None
    token: str = ""

    protocol_version = "HTTP/1.1"

    # ── helpers (robot api.py 패턴) ─────────────────────────────────────

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, payload) -> None:
        self._send(code, json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _page(self, filename: str) -> None:
        path = _ASSETS_DIR / filename
        if not path.exists():
            return self._json(404, {"error": f"{filename} 없음"})
        self._send(200, path.read_bytes(), "text/html; charset=utf-8")

    def _auth_ok(self, query: dict) -> bool:
        if not self.token:
            return True
        if self.headers.get("X-Auth-Token", "") == self.token:
            return True
        return query.get("token", [""])[0] == self.token

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}

    def log_message(self, fmt, *args):
        pass

    def _params_target(self, target: str):
        p = self.cfg.params.get(target)
        if not p or p["schema"] not in SCHEMAS:
            return None
        return self.cfg.resolve_path(p["file"]), SCHEMAS[p["schema"]]

    # ── GET ─────────────────────────────────────────────────────────────

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        parts = [p for p in url.path.split("/") if p]

        if not parts or (len(parts) == 1 and parts[0] in _PAGES):
            return self._page(_PAGES[parts[0] if parts else ""])
        if len(parts) == 2 and parts[0] == "assets":
            name = parts[1]
            if name not in _ASSET_TYPES:
                return self._json(404, {"error": "asset 없음"})
            return self._send(200, (_ASSETS_DIR / name).read_bytes(),
                              _ASSET_TYPES[name])

        if parts[0] != "api":
            return self._json(404, {"error": "not found"})
        if not self._auth_ok(query):
            return self._json(401, {"error": "bad token"})

        if parts == ["api", "health"]:
            return self._json(200, {
                "ok": True,
                "runners": [r.name for r in self.cfg.runners],
                "components": self.manager.names(),
            })
        if parts == ["api", "status"]:
            return self._json(200, self.collector.snapshot())
        if parts == ["api", "session"]:
            return self._json(200, self.session.state())
        if len(parts) == 4 and parts[1] == "components" and parts[3] == "log":
            name = parts[2]
            if name not in self.manager.names():
                return self._json(404, {"error": f"unknown component {name}"})
            since = int(query.get("since", ["0"])[0])
            max_lines = min(int(query.get("max", ["200"])[0]), 1000)
            return self._json(200,
                              {"lines": self.manager.log_since(name, since, max_lines)})
        if len(parts) == 3 and parts[1] == "params":
            target = self._params_target(parts[2])
            if target is None:
                return self._json(404, {"error": f"params 대상 아님: {parts[2]}"})
            file_path, schema = target
            try:
                return self._json(200, params_engine.get_params(file_path, schema)
                                  | {"target": parts[2]})
            except OSError as e:
                return self._json(500, {"error": str(e)})

        return self._json(404, {"error": "not found"})

    # ── POST ────────────────────────────────────────────────────────────

    def do_POST(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        parts = [p for p in url.path.split("/") if p]
        if not parts or parts[0] != "api":
            return self._json(404, {"error": "not found"})
        if not self._auth_ok(query):
            return self._json(401, {"error": "bad token"})
        body = self._read_body()

        if parts == ["api", "runner", "start"]:
            mode = body.get("mode", "")
            try:
                r = self.runner.start(mode)
            except KeyError as e:
                return self._json(404, {"error": str(e)})
            return self._json(200 if r.get("ok") else 409, r)
        if parts == ["api", "runner", "stop"]:
            return self._json(200, self.runner.stop())
        if parts == ["api", "runner", "key"]:
            r = self.runner.send_key(str(body.get("key", "")))
            if r.get("ok"):
                return self._json(200, r)
            code = 409 if "러너 없음" in str(r.get("error", "")) else 400
            return self._json(code, r)

        if parts == ["api", "cam", "start"] or parts == ["api", "cam", "stop"]:
            if not self.cfg.cam_component:
                return self._json(404, {"error": "cam viewer 미설정"})
            if parts[2] == "start":
                self.manager.start(self.cfg.cam_component)
            else:
                self.manager.stop(self.cfg.cam_component)
            return self._json(200, self.manager.status(self.cfg.cam_component))

        if parts == ["api", "adb", "reverse"]:
            if not self.adb.available():
                return self._json(400, {"ok": False, "error": "adb 미설치"})
            return self._json(200, self.adb.ensure_reverses(
                self.cfg.adb.reverse_ports))
        if parts == ["api", "adb", "restart"]:
            if not self.adb.available():
                return self._json(400, {"ok": False, "error": "adb 미설치"})
            return self._json(200, self.adb.restart_server())

        if parts == ["api", "session", "start"]:
            try:
                r = self.session.start(mode=body.get("mode"),
                                       skip_adb=bool(body.get("skip_adb", False)))
            except KeyError as e:
                return self._json(404, {"error": str(e)})
            return self._json(200 if r.get("ok") else 409, r)
        if parts == ["api", "session", "cancel"]:
            return self._json(200, self.session.cancel())

        return self._json(404, {"error": "not found"})

    def do_PUT(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        parts = [p for p in url.path.split("/") if p]
        if not self._auth_ok(query):
            return self._json(401, {"error": "bad token"})
        if len(parts) == 3 and parts[:2] == ["api", "params"]:
            target = self._params_target(parts[2])
            if target is None:
                return self._json(404, {"error": f"params 대상 아님: {parts[2]}"})
            file_path, schema = target
            values = self._read_body().get("values", {})
            if not isinstance(values, dict) or not values:
                return self._json(400, {"error": "body {values: {...}} 필요"})
            r = params_engine.set_params(file_path, schema, values)
            return self._json(200 if r.ok else 400, {
                "ok": r.ok, "saved": r.saved, "errors": r.errors,
                "comments_preserved": r.comments_preserved,
                "restart_required": True,
            })
        return self._json(404, {"error": "not found"})
