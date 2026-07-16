"""adb 헬퍼 — 헤드셋 연결/reverse 상태 조회 및 설정.

모든 호출은 subprocess (timeout 5s) + 출력 캡처. adb 미설치 환경도
정상 상태값("installed": false)으로 처리 — 웹 UI 가 설치 안내 표시.

참고 (docs/xr_input_guide.md): apt adb 는 glibc 문제 — Google
platform-tools 를 /opt/platform-tools 에 설치, PATH 등록.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
from typing import Dict, List, Optional

_TIMEOUT_S = 5.0


class AdbHelper:
    def __init__(self, binary: str = "adb"):
        self._binary = binary

    # ── 저수준 실행 ─────────────────────────────────────────────────────

    def available(self) -> bool:
        return shutil.which(self._binary) is not None

    def _run(self, *args: str) -> dict:
        """{ok, code, output} — output 은 stdout+stderr 합본 (UI 표시용)."""
        try:
            r = subprocess.run([self._binary, *args], capture_output=True,
                               text=True, timeout=_TIMEOUT_S)
            output = (r.stdout + r.stderr).strip()
            return {"ok": r.returncode == 0, "code": r.returncode,
                    "output": output, "cmd": f"{self._binary} {' '.join(args)}"}
        except FileNotFoundError:
            return {"ok": False, "code": -1, "output": "adb 미설치 (PATH 확인)",
                    "cmd": f"{self._binary} {' '.join(args)}"}
        except subprocess.TimeoutExpired:
            return {"ok": False, "code": -1, "output": f"timeout ({_TIMEOUT_S}s)",
                    "cmd": f"{self._binary} {' '.join(args)}"}

    # ── 조회 ────────────────────────────────────────────────────────────

    def devices(self) -> List[dict]:
        """[{serial, state}] — state: device|unauthorized|offline|..."""
        r = self._run("devices")
        if not r["ok"]:
            return []
        return parse_devices(r["output"])

    def reverse_list(self) -> List[dict]:
        """[{serial, remote, local}] — 예: tcp:8013 tcp:8013."""
        r = self._run("reverse", "--list")
        if not r["ok"]:
            return []
        return parse_reverse_list(r["output"])

    # ── 액션 ────────────────────────────────────────────────────────────

    def reverse(self, port: int) -> dict:
        return self._run("reverse", f"tcp:{port}", f"tcp:{port}")

    def ensure_reverses(self, ports: List[int]) -> dict:
        """모든 포트 reverse 시도. {ok, results:{port: {ok, output}}}."""
        results = {}
        all_ok = True
        for p in ports:
            r = self.reverse(p)
            results[str(p)] = r
            all_ok = all_ok and r["ok"]
        return {"ok": all_ok, "results": results}

    def restart_server(self) -> dict:
        """kill-server → start-server (연결 복구용)."""
        r1 = self._run("kill-server")
        r2 = self._run("start-server")
        return {"ok": r2["ok"],
                "output": f"$ {r1['cmd']}\n{r1['output']}\n"
                          f"$ {r2['cmd']}\n{r2['output']}".strip()}


# ── 출력 파서 (unit test 대상 — 순수 함수) ──────────────────────────────

def parse_devices(output: str) -> List[dict]:
    """`adb devices` 출력 파싱.

    List of devices attached
    R3CX70XXXXX	device
    emulator-5554	unauthorized
    """
    devices = []
    seen_header = False
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("List of devices"):
            seen_header = True
            continue
        if not seen_header or line.startswith("*"):   # daemon start 메시지 등
            continue
        parts = line.split()
        if len(parts) >= 2:
            devices.append({"serial": parts[0], "state": parts[1]})
    return devices


def parse_reverse_list(output: str) -> List[dict]:
    """`adb reverse --list` 출력 파싱.

    (reverse) UsbFfs tcp:8013 tcp:8013     ← 구형 포맷
    R3CX70XXXXX tcp:8013 tcp:8013          ← 신형 포맷
    """
    out = []
    for line in output.splitlines():
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        remote, local = parts[-2], parts[-1]
        if remote.startswith("tcp:") and local.startswith("tcp:"):
            out.append({"serial": parts[0].strip("()"),
                        "remote": remote, "local": local})
    return out


class AdbWatcher(threading.Thread):
    """devices + reverse --list 를 주기 폴링해 스냅샷 유지 (3s)."""

    def __init__(self, helper: AdbHelper, reverse_ports: List[int],
                 interval_s: float = 3.0):
        super().__init__(daemon=True, name="adb-watcher")
        self._helper = helper
        self._ports = reverse_ports
        self._interval = interval_s
        self._lock = threading.Lock()
        self._snap: dict = {"installed": helper.available(), "devices": [],
                            "reverses": [], "reverse_ok": False}
        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            installed = self._helper.available()
            devices = self._helper.devices() if installed else []
            reverses = self._helper.reverse_list() if installed else []
            reversed_ports = {int(r["remote"].split(":")[1]) for r in reverses}
            snap = {
                "installed": installed,
                "devices": devices,
                "connected": any(d["state"] == "device" for d in devices),
                "reverses": reverses,
                "reverse_ok": all(p in reversed_ports for p in self._ports),
                "missing_reverses": [p for p in self._ports
                                     if p not in reversed_ports],
            }
            with self._lock:
                self._snap = snap
            self._stop.wait(self._interval)

    def stop(self) -> None:
        self._stop.set()

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snap)
