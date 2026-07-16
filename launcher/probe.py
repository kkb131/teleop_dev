"""TCP 도달성 probe + ANSI 제거 유틸 (공용).

robot: UR(30004)/DG5F Modbus(502), sender: 로컬 8013/8014 등 —
대상은 호출측이 dict 로 지정한다.
"""

from __future__ import annotations

import re
import socket
import threading
import time
from typing import Dict, Tuple

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def strip_ansi(line: str) -> str:
    return ANSI_RE.sub("", line).replace("\r", "").strip()


class HwProber(threading.Thread):
    """대상별 (host, port) TCP 도달성 검사 — 주기적 백그라운드."""

    def __init__(self, targets: Dict[str, Tuple[str, int]],
                 interval_s: float = 3.0, timeout_s: float = 0.5):
        super().__init__(daemon=True, name="hw-prober")
        self._targets = targets                # name → (host, port)
        self._interval = interval_s
        self._timeout = timeout_s
        self._lock = threading.Lock()
        self._results: Dict[str, dict] = {}
        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            for name, (host, port) in self._targets.items():
                reachable = self._probe(host, port)
                with self._lock:
                    self._results[name] = {
                        "host": host, "port": port,
                        "reachable": reachable, "checked_at": time.time(),
                    }
            self._stop.wait(self._interval)

    def _probe(self, host: str, port: int) -> bool:
        try:
            with socket.create_connection((host, port), timeout=self._timeout):
                return True
        except OSError:
            return False

    def stop(self) -> None:
        self._stop.set()

    def snapshot(self) -> Dict[str, dict]:
        now = time.time()
        with self._lock:
            return {
                name: {**r, "checked_s_ago": round(now - r["checked_at"], 1)}
                for name, r in self._results.items()
            }
