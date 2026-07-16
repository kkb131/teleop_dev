"""상태 수집 — 프로세스 상태 + 하드웨어 TCP 도달성 + 로그 파싱 메트릭.

수신부 코드를 수정하지 않고 상태를 얻는다:
- 프로세스: ManagedProcess.status()
- 하드웨어: HwProber 가 UR(30004)/DG5F Modbus(502) 에 TCP connect 검사
- 팔 라이브 상태: admittance 수신부의 8줄 상태블록(stdout, 100ms 주기)을
  로그 링버퍼에서 ANSI 제거 후 정규식으로 파싱
- 캠: robot.cam.main 의 2s 주기 stats 라인 파싱 ("name: 30.0fps 850KB/s fail=N")
"""

from __future__ import annotations

import re
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple

from launcher.manager import LocalManager
from launcher.robot.parts import PartSpec, RobotConfig

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

# admittance 수신부 상태블록 (robot/arm/admittance/main.py _write_status 포맷)
_ARM_PATTERNS = {
    "ee_pos": re.compile(
        r"EE Pos\s*:\s*x=\s*(-?\d+\.\d+)\s+y=\s*(-?\d+\.\d+)\s+z=\s*(-?\d+\.\d+)"),
    "ee_rpy": re.compile(
        r"EE RPY\s*:\s*R=\s*(-?\d+\.?\d*)\s+P=\s*(-?\d+\.?\d*)\s+Y=\s*(-?\d+\.?\d*)"),
    "safety": re.compile(r"Safety\s*:\s*(\S+)(?:\s+(.*?))?\s*$"),
    "estop": re.compile(r"E-Stop\s*:\s*(!! ACTIVE !!|off)"),
    "admit": re.compile(r"Admit\s*:\s*(.+?)\s*$"),
    "input_ms": re.compile(r"Input\s*:\s*(\d+)ms ago"),
}

# robot.cam.main 의 stats 라인: "[robot.cam] head: 29.8fps 850KB/s fail=2  wrist: ..."
_CAM_LINE_RE = re.compile(r"\[robot\.cam\]\s+(.+)")
_CAM_STAT_RE = re.compile(r"(\S+): ([\d.]+)fps (\d+)KB/s(?: fail=(\d+))?")


def strip_ansi(line: str) -> str:
    return ANSI_RE.sub("", line).replace("\r", "").strip()


def parse_arm_status(lines: List[str]) -> dict:
    """로그 라인들(최근 순서대로)에서 팔 상태블록 필드 추출 — 키별 마지막 매치."""
    out: dict = {}
    for raw in lines:
        line = strip_ansi(raw)
        m = _ARM_PATTERNS["ee_pos"].search(line)
        if m:
            out["ee_pos"] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
            continue
        m = _ARM_PATTERNS["ee_rpy"].search(line)
        if m:
            out["ee_rpy_deg"] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
            continue
        m = _ARM_PATTERNS["estop"].search(line)
        if m:
            out["estop_active"] = m.group(1) != "off"
            continue
        m = _ARM_PATTERNS["safety"].search(line)
        if m:
            out["safety"] = m.group(1)
            if m.group(2):
                out["safety_msg"] = m.group(2).strip()
            continue
        m = _ARM_PATTERNS["admit"].search(line)
        if m:
            out["admittance"] = m.group(1)
            continue
        m = _ARM_PATTERNS["input_ms"].search(line)
        if m:
            out["input_age_ms"] = int(m.group(1))
    return out


def parse_cam_stats(lines: List[str]) -> List[dict]:
    """로그 라인들에서 마지막 캠 stats 라인 파싱 → [{name,fps,kbps,fail}]."""
    latest: Optional[str] = None
    for raw in lines:
        line = strip_ansi(raw)
        m = _CAM_LINE_RE.search(line)
        if m and _CAM_STAT_RE.search(m.group(1)):
            latest = m.group(1)
    if latest is None:
        return []
    return [
        {"name": name, "fps": float(fps), "kbps": int(kbps),
         "fail": int(fail) if fail else 0}
        for name, fps, kbps, fail in _CAM_STAT_RE.findall(latest)
    ]


class HwProber(threading.Thread):
    """파츠별 probe (host, port) TCP 도달성 검사 — 3s 주기 백그라운드."""

    def __init__(self, targets: Dict[str, Tuple[str, int]],
                 interval_s: float = 3.0, timeout_s: float = 0.5):
        super().__init__(daemon=True, name="hw-prober")
        self._targets = targets                # part_name → (host, port)
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


class StatusCollector:
    """웹 GET /api/status 응답 조립."""

    # 로그 파싱 시 최근 N 줄만 스캔 (상태블록 100ms 주기 → 40줄 ≈ 최근 0.5s)
    SCAN_LINES = 40
    # 캠 stats 는 2s 주기 — 로그 seq 가 6s 이상 정지면 stale
    CAM_STALE_S = 6.0

    def __init__(self, cfg: RobotConfig, manager: LocalManager,
                 prober: HwProber):
        self._cfg = cfg
        self._manager = manager
        self._prober = prober
        self._cam_last_seq: int = 0
        self._cam_last_seq_time: float = 0.0

    def _part_state(self, comp_statuses: List[dict]) -> str:
        states = [s["state"] for s in comp_statuses]
        if all(s == "running" for s in states):
            return "running"
        if any(s == "exited" for s in states):
            return "error"
        if any(s == "running" for s in states):
            return "partial"
        return "stopped"

    def _metrics_for(self, part: PartSpec, comp_statuses: List[dict]) -> dict:
        """kind 별 라이브 메트릭 (로그 파싱)."""
        if part.kind == "arm":
            name = part.components[0]
            if any(s["name"] == name and s["state"] == "running"
                   for s in comp_statuses):
                lines = [l for _, l in
                         self._manager.log_since(name, 0, self.SCAN_LINES * 8)]
                return parse_arm_status(lines[-self.SCAN_LINES * 8:])
            return {}
        if part.kind == "cam":
            name = part.components[0]
            running = any(s["name"] == name and s["state"] == "running"
                          for s in comp_statuses)
            if not running:
                self._cam_last_seq = 0
                return {}
            lines_seq = self._manager.log_since(name, 0, 200)
            cams = parse_cam_stats([l for _, l in lines_seq])
            # stale 판정: 로그 seq 가 진행 중인지
            cur_seq = lines_seq[-1][0] if lines_seq else 0
            now = time.time()
            if cur_seq != self._cam_last_seq:
                self._cam_last_seq = cur_seq
                self._cam_last_seq_time = now
            stale = (bool(cams)
                     and now - self._cam_last_seq_time > self.CAM_STALE_S)
            return {"cameras": cams, "stale": stale}
        return {}

    def snapshot(self, oneshot_statuses: Optional[List[dict]] = None) -> dict:
        hw = self._prober.snapshot()
        all_status = {s["name"]: s for s in self._manager.status_all()}
        parts_out = {}
        for part in self._cfg.parts:
            comp_statuses = [all_status[c] for c in part.components
                             if c in all_status]
            parts_out[part.name] = {
                "label": part.label,
                "kind": part.kind,
                "side": part.side,
                "state": self._part_state(comp_statuses),
                "components": comp_statuses,
                "hw": hw.get(part.name),
                "metrics": self._metrics_for(part, comp_statuses),
                "has_params": bool(part.params_file),
            }
        out = {"ts": time.time(), "parts": parts_out}
        if oneshot_statuses:
            out["oneshots"] = oneshot_statuses
        return out
