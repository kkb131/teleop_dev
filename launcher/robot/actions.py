"""웹 액션 — 비상정지(E-Stop) burst, 초기 위치 이동 oneshot 스폰.

E-Stop 원리:
    팔 수신부(UnifiedNetworkInput)는 UDP 를 드레인한 뒤 **최신 패킷만**
    처리하므로, sender 가 50Hz 로 스트리밍 중이면 단발 estop 패킷은 race
    에서 질 수 있다 → 200Hz × 0.5s burst 로 전송해 확률적으로 보장.
    estop 패킷은 buttons.estop 만 보고 즉시 latch 되며 pose 는 사용되지
    않는다 (robot/core/input_handler.py — estop 시 target 없이 return).
    latch 후에는 reset 버튼 외엔 해제되지 않는다.

    백스톱: burst 후 1.0s 안에 수신부 로그에 "E-Stop : !! ACTIVE !!" 가
    보이지 않으면 해당 수신부를 SIGINT — servoJ 스트림 중단 → UR 컨트롤러
    자체 안전정지.

패킷은 protocol.arm_protocol.TeleopPosePacket 와이어 포맷과 호환되는
수제 JSON (launcher 의 self-contained 원칙 유지 — numpy/protocol import 없음).
"""

from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from launcher.config import ComponentSpec
from launcher.manager import LocalManager
from launcher.process import ManagedProcess
from launcher.robot.parts import RobotConfig
from launcher.robot.status import parse_arm_status


def build_estop_packet() -> bytes:
    """protocol/arm_protocol.py from_bytes 호환 estop 패킷.

    필수: v==1, type=="teleop_pose", pos(3), quat(4 wxyz).
    buttons 는 부분집합 허용 — estop 만 채움. pose 는 estop 경로에서
    사용되지 않는다 (수신부가 estop 확인 즉시 return).
    """
    return json.dumps({
        "v": 1,
        "type": "teleop_pose",
        "pos": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "buttons": {"estop": True},
        "gripper": 0.0,
        "timestamp": time.time(),
    }, separators=(",", ":")).encode("utf-8")


def send_estop_burst(port: int, hz: int = 200, duration_s: float = 0.5,
                     host: str = "127.0.0.1") -> int:
    """localhost UDP 로 estop 패킷 burst. 전송 발수 반환."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / max(hz, 1)
    deadline = time.perf_counter() + duration_s
    sent = 0
    try:
        while time.perf_counter() < deadline:
            sock.sendto(build_estop_packet(), (host, port))
            sent += 1
            time.sleep(dt)
    finally:
        sock.close()
    return sent


class ActionRunner:
    """estop / move_home 실행기 — 웹 API 와 공유되는 상태 보유."""

    ESTOP_LOG_CONFIRM_S = 1.0     # burst 후 로그에서 latch 확인 대기

    def __init__(self, cfg: RobotConfig, manager: LocalManager):
        self._cfg = cfg
        self._manager = manager
        self._oneshots: Dict[str, ManagedProcess] = {}   # move-home-<side> 등
        self._lock = threading.Lock()
        self.last_estop: Optional[dict] = None

    # ── E-Stop ──────────────────────────────────────────────────────────

    def _arm_port(self, side: str) -> int:
        """트리거 시점에 팔 yaml 의 input.unified_port 를 fresh 하게 읽기."""
        mh = self._cfg.move_home.get(side)
        fallback = self._cfg.estop.fallback_arm_ports.get(side, 9871)
        if not mh or not mh.config:
            return fallback
        try:
            data = yaml.safe_load(
                Path(self._cfg.resolve_path(mh.config)).read_text()) or {}
            return int((data.get("input") or {}).get("unified_port", fallback))
        except (OSError, ValueError, yaml.YAMLError):
            return fallback

    def _arm_receiver_name(self, side: str) -> Optional[str]:
        for p in self._cfg.parts:
            if p.kind == "arm" and p.side == side and p.components:
                return p.components[0]
        return None

    def _hand_receiver_names(self) -> List[str]:
        out = []
        for p in self._cfg.parts:
            if p.kind == "hand":
                # receiver 만 정지 (드라이버는 유지 — 손이 현재 자세 유지)
                out.extend(c for c in p.components if "receiver" in c)
        return out

    def estop(self, sides=("right", "left")) -> dict:
        """비상정지: 팔별 burst → 손 수신부 정지 → 로그 미확인 팔 SIGINT."""
        es = self._cfg.estop
        result = {"burst_sent": {}, "hands_stopping": [], "backstop_stopped": [],
                  "warnings": [], "ts": time.time()}

        running_arms = []
        for side in sides:
            name = self._arm_receiver_name(side)
            if name is None:
                continue
            try:
                running = self._manager.status(name)["state"] == "running"
            except KeyError:
                running = False
            port = self._arm_port(side)
            sent = send_estop_burst(port, es.burst_hz, es.burst_duration_s)
            result["burst_sent"][side] = {"port": port, "packets": sent}
            if running:
                running_arms.append((side, name))
            else:
                result["warnings"].append(
                    f"{side} 팔 수신부가 실행 중이 아님 — burst 는 무의미 (수신자 없음)")

        # 손 수신부 정지 (백그라운드 — stop 은 blocking)
        hand_names = [n for n in self._hand_receiver_names()
                      if self._safe_state(n) == "running"]
        result["hands_stopping"] = hand_names
        for n in hand_names:
            threading.Thread(target=self._manager.stop, args=(n,),
                             daemon=True, name=f"estop-stop-{n}").start()

        # 백스톱: 로그에서 latch 확인, 미확인 시 SIGINT
        deadline = time.time() + self.ESTOP_LOG_CONFIRM_S
        pending = dict(running_arms)
        while pending and time.time() < deadline:
            for side, name in list(pending.items()):
                lines = [l for _, l in self._manager.log_since(name, 0, 100)]
                st = parse_arm_status(lines[-40:])
                if st.get("estop_active"):
                    del pending[side]
            if pending:
                time.sleep(0.1)
        for side, name in pending.items():
            result["backstop_stopped"].append(name)
            result["warnings"].append(
                f"{side} 팔 로그에서 E-Stop latch 미확인 — 수신부 SIGINT (servoJ 중단)")
            threading.Thread(target=self._manager.stop, args=(name,),
                             daemon=True, name=f"estop-backstop-{name}").start()

        self.last_estop = result
        return result

    def _safe_state(self, name: str) -> str:
        try:
            return self._manager.status(name)["state"]
        except KeyError:
            return "unknown"

    # ── 초기 위치 이동 ──────────────────────────────────────────────────

    def move_home(self, side: str, stop_receiver: bool = False) -> dict:
        """oneshot move-home-<side> 스폰. 수신부 실행 중이면 거부(409성)."""
        if side not in self._cfg.move_home:
            return {"ok": False, "error": f"move_home 설정에 {side!r} 없음"}
        mh = self._cfg.move_home[side]
        receiver = self._arm_receiver_name(side)

        with self._lock:
            # 이미 이동 중?
            prev = self._oneshots.get(f"move-home-{side}")
            if prev is not None and prev.is_running():
                return {"ok": False, "error": f"{side} 초기 위치 이동이 이미 진행 중"}

            # RTDE 배타성 가드
            if receiver and self._safe_state(receiver) == "running":
                if not stop_receiver:
                    return {"ok": False, "receiver_running": True,
                            "error": f"{side} 팔 수신부({receiver})가 실행 중 — "
                                     "정지 후 이동하거나 stop_receiver=true 로 요청"}
                self._manager.stop(receiver)   # blocking (grace 내)

            spec = ComponentSpec(
                name=f"move-home-{side}",
                group="robot",
                command=(f"python3 -m launcher.robot.move_home --side {side} "
                         f"--config {mh.config} --robot-ip {mh.robot_ip} "
                         f"--speed {mh.speed} --accel {mh.accel}"),
                cwd=self._cfg.teleop_dir,
                stop_grace_s=3.0,
                oneshot=True,
            )
            proc = ManagedProcess(spec, self._cfg.launcher.setups)
            proc.start()
            self._oneshots[spec.name] = proc

        return {"ok": True, "component": spec.name}

    # ── oneshot 상태/로그 노출 (대시보드 병합용) ────────────────────────

    def oneshot_statuses(self) -> List[dict]:
        with self._lock:
            return [p.status() for p in self._oneshots.values()]

    def oneshot_log_since(self, name: str, since: int = 0,
                          max_lines: int = 200):
        with self._lock:
            proc = self._oneshots.get(name)
        if proc is None:
            raise KeyError(name)
        return proc.log_since(since, max_lines)

    def stop_oneshots(self) -> None:
        with self._lock:
            for p in self._oneshots.values():
                if p.is_running():
                    p.stop()
