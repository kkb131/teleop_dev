"""RemoteManager — agent HTTP API 클라이언트 (LocalManager 와 동일 인터페이스).

GUI 가 robot PC 의 컴포넌트를 로컬과 똑같이 다루기 위한 duck-type 어댑터.
표준 라이브러리 urllib 만 사용.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import List, Optional, Tuple


class AgentUnreachable(Exception):
    pass


class RemoteManager:
    def __init__(self, base_url: str, token: str = "", timeout: float = 3.0):
        self.base_url = base_url.rstrip("/")
        self._token = token
        self._timeout = timeout

    # ── HTTP ────────────────────────────────────────────────────────────

    def _request(self, method: str, path: str, body: Optional[dict] = None):
        url = f"{self.base_url}{path}"
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(url, data=data, method=method)
        req.add_header("Content-Type", "application/json")
        if self._token:
            req.add_header("X-Auth-Token", self._token)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as e:
            raise AgentUnreachable(f"{method} {url}: {e}") from e

    # ── LocalManager 와 동일 인터페이스 ─────────────────────────────────

    def names(self) -> List[str]:
        return self._request("GET", "/api/health")["components"]

    def start(self, name: str) -> dict:
        return self._request("POST", f"/api/components/{name}/start")

    def stop(self, name: str) -> dict:
        return self._request("POST", f"/api/components/{name}/stop")

    def status_all(self) -> List[dict]:
        return self._request("GET", "/api/components")

    def log_since(self, name: str, since: int = 0,
                  max_lines: int = 200) -> List[Tuple[int, str]]:
        r = self._request("GET", f"/api/components/{name}/log?since={since}&max={max_lines}")
        return [tuple(x) for x in r["lines"]]

    def send_input(self, name: str, text: str) -> bool:
        return self._request("POST", f"/api/components/{name}/input",
                             {"text": text}).get("ok", False)

    def start_all(self, names: Optional[List[str]] = None) -> List[str]:
        return self._request("POST", "/api/start_all",
                             {"names": names} if names else {})["started"]

    def stop_all(self, names: Optional[List[str]] = None) -> List[str]:
        return self._request("POST", "/api/stop_all",
                             {"names": names} if names else {})["stopped"]
