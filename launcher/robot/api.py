"""웹 대시보드 HTTP 핸들러 — 페이지 서빙 + JSON API.

stdlib ThreadingHTTPServer 기반 (robot PC 도커에 추가 의존성 없음).

라우트:
    GET  /                        index.html (메인 대시보드)
    GET  /arm?side=right|left     arm.html (팔 파라미터/상태)
    GET  /cam                     cam.html (캠 파라미터/상태)
    GET  /assets/<name>           app.js / style.css (화이트리스트)

    GET  /api/health              {ok, parts, components}
    GET  /api/status              StatusCollector.snapshot() (+oneshots)
    POST /api/parts/<n>/start     파츠 컴포넌트 topo 순 시작
    POST /api/parts/<n>/stop      역순 정지
    POST /api/parts/<n>/restart   stop → start
    POST /api/start_all           전체 시작 (topo)
    POST /api/stop_all            전체 정지 (역순)
    POST /api/estop               body {sides?: ["right","left"]}
    POST /api/arms/<side>/move_home   body {stop_receiver?: bool} → 200/409
    GET  /api/components/<n>/log?since=N&max=200
    GET  /api/params/<target>     target ∈ {arm-right, arm-left, cam}
    PUT  /api/params/<target>     body {values: {dotted.key: v}}

인증: web.token 설정 시 X-Auth-Token 헤더 또는 ?token= 쿼리 필수 (API 만).
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from launcher.manager import LocalManager
from launcher.robot.actions import ActionRunner
from launcher.robot.parts import RobotConfig
from launcher.robot.status import StatusCollector

_ASSETS_DIR = Path(__file__).parent / "assets"

_PAGES = {
    "": "index.html",
    "arm": "arm.html",
    "cam": "cam.html",
}
_ASSET_TYPES = {
    "app.js": "application/javascript; charset=utf-8",
    "style.css": "text/css; charset=utf-8",
}


class RobotApiHandler(BaseHTTPRequestHandler):
    # web.py serve() 에서 주입
    cfg: RobotConfig = None
    manager: LocalManager = None
    collector: StatusCollector = None
    actions: ActionRunner = None
    token: str = ""

    protocol_version = "HTTP/1.1"

    # ── helpers ─────────────────────────────────────────────────────────

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

    def log_message(self, fmt, *args):   # 기본 액세스 로그 억제
        pass

    def _params_target(self, target: str):
        """target 이름 → (params_file 절대경로, schema 이름). 없으면 None."""
        for p in self.cfg.parts:
            if p.name == target and p.params_file:
                return self.cfg.resolve_path(p.params_file), p.params_schema
        return None

    # ── GET ─────────────────────────────────────────────────────────────

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        parts = [p for p in url.path.split("/") if p]

        # 페이지 / 정적 자산 (no auth — API 만 보호)
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
                "parts": [p.name for p in self.cfg.parts],
                "components": self.manager.names(),
            })

        if parts == ["api", "status"]:
            return self._json(200, self.collector.snapshot(
                oneshot_statuses=self.actions.oneshot_statuses()))

        if len(parts) == 4 and parts[1] == "components" and parts[3] == "log":
            name = parts[2]
            since = int(query.get("since", ["0"])[0])
            max_lines = min(int(query.get("max", ["200"])[0]), 1000)
            try:
                if name in self.manager.names():
                    lines = self.manager.log_since(name, since, max_lines)
                else:
                    lines = self.actions.oneshot_log_since(name, since, max_lines)
            except KeyError:
                return self._json(404, {"error": f"unknown component {name}"})
            return self._json(200, {"lines": lines})

        if len(parts) == 3 and parts[1] == "params":
            target = self._params_target(parts[2])
            if target is None:
                return self._json(404, {"error": f"params 대상 아님: {parts[2]}"})
            from launcher.robot.params import get_params
            file_path, schema = target
            try:
                return self._json(200, get_params(file_path, schema)
                                  | {"target": parts[2]})
            except (OSError, KeyError) as e:
                return self._json(500, {"error": str(e)})

        return self._json(404, {"error": "not found"})

    # ── POST / PUT ──────────────────────────────────────────────────────

    def do_POST(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        parts = [p for p in url.path.split("/") if p]
        if not parts or parts[0] != "api":
            return self._json(404, {"error": "not found"})
        if not self._auth_ok(query):
            return self._json(401, {"error": "bad token"})

        if parts == ["api", "start_all"]:
            return self._json(200, {"started": self.manager.start_all()})
        if parts == ["api", "stop_all"]:
            stopped = self.manager.stop_all()
            self.actions.stop_oneshots()
            return self._json(200, {"stopped": stopped})

        if parts == ["api", "estop"]:
            sides = tuple(self._read_body().get("sides", ["right", "left"]))
            return self._json(200, self.actions.estop(sides))

        if len(parts) == 4 and parts[1] == "arms" and parts[3] == "move_home":
            side = parts[2]
            body = self._read_body()
            result = self.actions.move_home(
                side, stop_receiver=bool(body.get("stop_receiver", False)))
            if result.get("ok"):
                return self._json(200, result)
            code = 409 if result.get("receiver_running") or "진행 중" in \
                str(result.get("error", "")) else 400
            return self._json(code, result)

        if len(parts) == 4 and parts[1] == "parts":
            name, action = parts[2], parts[3]
            try:
                part = self.cfg.part(name)
            except KeyError:
                return self._json(404, {"error": f"unknown part {name}"})
            comps = part.components
            if action == "start":
                return self._json(200, {"started": self.manager.start_all(comps)})
            if action == "stop":
                return self._json(200, {"stopped": self.manager.stop_all(comps)})
            if action == "restart":
                stopped = self.manager.stop_all(comps)
                started = self.manager.start_all(comps)
                return self._json(200, {"stopped": stopped, "started": started})

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
            from launcher.robot.params import set_params
            file_path, schema = target
            values = self._read_body().get("values", {})
            if not isinstance(values, dict) or not values:
                return self._json(400, {"error": "body {values: {...}} 필요"})
            r = set_params(file_path, schema, values)
            payload = {"ok": r.ok, "saved": r.saved, "errors": r.errors,
                       "comments_preserved": r.comments_preserved,
                       "restart_required": True}
            return self._json(200 if r.ok else 400, payload)

        return self._json(404, {"error": "not found"})
