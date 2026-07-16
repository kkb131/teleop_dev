"""조종 PC 상태 수집 — adb + 로컬 포트 probe + 러너 로그 파싱.

러너/BridgePoseStore 코드를 수정하지 않고 상태를 얻는다. 파싱 대상 로그
계약 (실측 확인):
- 팔:   `[Sender:right] #NNN pos=[...] spd=SYNC x0.5`  (dual, 2s 주기)
        `[Sender] #NNN pos=[...] spd=READY x0.2`       (single, \\r 인라인 —
        pty 펌프가 \\r 을 줄바꿈 처리하므로 라인으로 도착)
- 손:   `[xr_dual:hand:right] #N TRACK|LOST skip=N ...` (dual)
        `[run_xr_teleop:hand] #N TRACK|LOST skip=N ...` (single)
- 캘리: `[XRArmSender:right] calibrate (r)` (dual) / `[XRArmSender] calibrate (` (single)
- 거부: `WARN: 헤드셋 ws msg 없음` 등 4종
- 핸드셰이크: `[Sender] Initial pose received:` / `WARNING: Pose query failed`
  (무라벨 — 집계만 가능)
- 헤드셋 WS: `[BridgePoseStore] ws client connected` / `ws disconnected`
- 제스처: `[gesture] ...`
- cam viewer: `[sender.cam] head: 29.8fps 850KB/s lat=12ms`
"""

from __future__ import annotations

import re
import time
from typing import Callable, Dict, List, Optional

from launcher.manager import LocalManager
from launcher.probe import HwProber, strip_ansi
from launcher.sender.adb import AdbWatcher
from launcher.sender.parts import SenderConfig

import yaml as pyyaml
from pathlib import Path

# ── 정규식 (검증된 실포맷) ───────────────────────────────────────────────

_ARM_STATUS_RE = re.compile(
    r"\[Sender(?::(?P<side>\w+))?\]\s+#\s*(?P<n>\d+)\s+"
    r"pos=\[(?P<pos>[^\]]+)\]\s+spd=(?P<spd>.+?)\s*$")
_HAND_RE = re.compile(
    r"\[(?:xr_dual:hand:(?P<side>\w+)|run_xr_teleop:hand)\]\s+#\s*(?P<n>\d+)\s+"
    r"(?P<state>TRACK|LOST)\s*skip=(?P<skip>\d+)")
_CALIB_RE = re.compile(r"\[XRArmSender(?::(?P<side>\w+))?\]\s+calibrate \(")
_WARN_RE = re.compile(
    r"\[XRArmSender(?::\w+)?\]\s+WARN:\s*(?P<msg>"
    r"헤드셋 ws msg 없음|BridgePoseStore msg 가 sender 시작 시각보다 오래됨"
    r"|마지막 msg age=[\d.]+s > [\d.]+s|user wrist pose 가 아직 invalid"
    r"|BridgePoseStore stale)")
_HANDSHAKE_OK_RE = re.compile(r"\[Sender\]\s+Initial pose received")
_HANDSHAKE_FAIL_RE = re.compile(r"\[Sender\]\s+WARNING: Pose query failed")
_WS_CONN_RE = re.compile(r"\[BridgePoseStore\]\s+ws client connected")
_WS_DISC_RE = re.compile(r"\[BridgePoseStore\]\s+ws disconnected")
_GESTURE_RE = re.compile(r"\[gesture\]\s+(?P<msg>.+)$")
_RUNNING_RE = re.compile(r"\[Sender\]\s+Running\.")
_SENDER_CAM_RE = re.compile(r"\[sender\.cam\]\s+(.+)")
_SENDER_CAM_STAT_RE = re.compile(r"(\S+): ([\d.]+)fps (\d+)KB/s lat=(\d+)ms")


def _spd_state(spd: str) -> str:
    """'SYNC x0.5' / 'READY PAUSE x0.2' → 대표 상태 문자열."""
    tokens = spd.split()
    states = [t for t in tokens if not t.startswith("x")]
    return " ".join(states) if states else "?"


def parse_runner_log(lines: List[str]) -> dict:
    """러너 로그 라인들(오래된 → 최신)에서 전체 상태 추출."""
    arms: Dict[str, dict] = {}
    hands: Dict[str, dict] = {}
    calib_counts: Dict[str, int] = {}
    handshake = {"received": 0, "failed": 0}
    headset_ws: Optional[str] = None
    last_warn: Optional[str] = None
    last_gesture: Optional[str] = None
    running_seen = False

    for raw in lines:
        line = strip_ansi(raw)
        if not line:
            continue
        m = _ARM_STATUS_RE.search(line)
        if m:
            side = m.group("side") or "single"
            spd = m.group("spd").strip()
            arms[side] = {"spd": spd, "state": _spd_state(spd),
                          "count": int(m.group("n"))}
            continue
        m = _HAND_RE.search(line)
        if m:
            side = m.group("side") or "single"
            hands[side] = {"tracking": m.group("state") == "TRACK",
                           "skip": int(m.group("skip"))}
            continue
        m = _CALIB_RE.search(line)
        if m:
            side = m.group("side") or "single"
            calib_counts[side] = calib_counts.get(side, 0) + 1
            continue
        m = _WARN_RE.search(line)
        if m:
            last_warn = m.group("msg")
            continue
        if _HANDSHAKE_OK_RE.search(line):
            handshake["received"] += 1
            continue
        if _HANDSHAKE_FAIL_RE.search(line):
            handshake["failed"] += 1
            continue
        if _WS_CONN_RE.search(line):
            headset_ws = "connected"
            continue
        if _WS_DISC_RE.search(line):
            headset_ws = "disconnected"
            continue
        m = _GESTURE_RE.search(line)
        if m:
            last_gesture = m.group("msg")
            continue
        if _RUNNING_RE.search(line):
            running_seen = True

    for side, count in calib_counts.items():
        arms.setdefault(side, {})["calib_count"] = count
    return {
        "arms": arms,
        "hands": hands,
        "handshake": handshake,
        "headset_ws": headset_ws or "unknown",
        "last_warn": last_warn,
        "last_gesture": last_gesture,
        "running_seen": running_seen,
    }


def count_calibrates(lines: List[str]) -> Dict[str, int]:
    """(자동 캘리 FSM 용) 태그별 calibrate 발생 횟수."""
    counts: Dict[str, int] = {}
    for raw in lines:
        m = _CALIB_RE.search(strip_ansi(raw))
        if m:
            side = m.group("side") or "single"
            counts[side] = counts.get(side, 0) + 1
    return counts


def find_hard_fail(lines: List[str]) -> Optional[str]:
    """(자동 캘리 FSM 용) 재시도 무의미한 실패 — 사유 문자열 또는 None."""
    for raw in lines:
        if _HANDSHAKE_FAIL_RE.search(strip_ansi(raw)):
            return "로봇 PC 무응답 (Pose query failed) — 로봇측 수신부/네트워크 확인"
    return None


def find_soft_fail(lines: List[str]) -> Optional[str]:
    """(자동 캘리 FSM 용) 재시도 가능한 거부 WARN — 마지막 사유."""
    last = None
    for raw in lines:
        m = _WARN_RE.search(strip_ansi(raw))
        if m:
            last = m.group("msg")
    return last


def parse_sender_cam_stats(lines: List[str]) -> List[dict]:
    """cam viewer 로그의 마지막 stats 라인 → [{name,fps,kbps,lat_ms}]."""
    latest: Optional[str] = None
    for raw in lines:
        m = _SENDER_CAM_RE.search(strip_ansi(raw))
        if m and _SENDER_CAM_STAT_RE.search(m.group(1)):
            latest = m.group(1)
    if latest is None:
        return []
    return [
        {"name": name, "fps": float(fps), "kbps": int(kbps), "lat_ms": int(lat)}
        for name, fps, kbps, lat in _SENDER_CAM_STAT_RE.findall(latest)
    ]


def check_ip_mismatch(cfg: SenderConfig) -> Optional[str]:
    """sender.yaml robot_pc_ip vs xr_dual.yaml network.robot_pc_ip 비교."""
    target = cfg.params.get("xr-dual")
    if not target or not cfg.robot_pc_ip:
        return None
    try:
        data = pyyaml.safe_load(
            Path(cfg.resolve_path(target["file"])).read_text()) or {}
        dual_ip = (data.get("network") or {}).get("robot_pc_ip", "")
    except (OSError, pyyaml.YAMLError):
        return None
    if dual_ip and dual_ip != cfg.robot_pc_ip:
        return (f"robot PC IP 불일치: sender.yaml={cfg.robot_pc_ip} (단일 모드) "
                f"≠ xr_dual.yaml={dual_ip} (듀얼 모드)")
    return None


class SenderStatusCollector:
    """웹 GET /api/status 응답 조립."""

    SCAN_LINES = 300     # 러너 로그 스캔 범위 (2s 주기 상태라인 기준 수 분)

    def __init__(self, cfg: SenderConfig, manager: LocalManager,
                 adb_watcher: AdbWatcher, prober: HwProber,
                 current_mode: Callable[[], Optional[str]],
                 session_state: Callable[[], dict]):
        self._cfg = cfg
        self._manager = manager
        self._adb = adb_watcher
        self._prober = prober
        self._current_mode = current_mode
        self._session_state = session_state

    def snapshot(self) -> dict:
        cfg = self._cfg
        warnings: List[str] = []

        mismatch = check_ip_mismatch(cfg)
        if mismatch:
            warnings.append(mismatch)

        adb = self._adb.snapshot()
        if adb.get("installed") and not adb.get("connected"):
            warnings.append("헤드셋 미연결 (adb devices 에 device 상태 없음)")

        # 러너 상태
        mode = self._current_mode()
        runner_out: dict = {"mode": mode, "component": None}
        if mode:
            comp_name = cfg.runner(mode).component
            comp = self._manager.status(comp_name)
            runner_out["component"] = comp
            if comp["state"] == "running":
                lines = [l for _, l in self._manager.log_since(
                    comp_name, 0, self.SCAN_LINES)]
                runner_out.update(parse_runner_log(lines))

        # cam viewer
        cam_out: dict = {"component": None, "cameras": []}
        if cfg.cam_component:
            comp = self._manager.status(cfg.cam_component)
            cam_out["component"] = comp
            if comp["state"] == "running":
                lines = [l for _, l in self._manager.log_since(
                    cfg.cam_component, 0, 100)]
                cam_out["cameras"] = parse_sender_cam_stats(lines)

        return {
            "ts": time.time(),
            "adb": adb,
            "ports": self._prober.snapshot(),
            "runner": runner_out,
            "runners": [{"name": r.name, "label": r.label, "kind": r.kind,
                         "side": r.side} for r in cfg.runners],
            "cam_viewer": cam_out,
            "session": self._session_state(),
            "warnings": warnings,
        }
