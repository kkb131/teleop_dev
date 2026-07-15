#!/usr/bin/env python3
"""XR (Galaxy XR / Quest 3) arm sender — WebXR wrist → robot base_link → UDP 9871.

조종 PC 에서 BridgePoseStore 의 right_arm_pose 를 읽고 relative motion 매핑
으로 robot 의 target TCP pose 를 계산해 UDP 9871 패킷 송신.

robot PC 측은 기존 `UnifiedNetworkInput` 그대로 사용 (변경 없음):
  - 'r' 키 (recalibrate) → robot/admittance 측 reset 트리거 → reset_pose →
    query_pose 응답 → sender 가 새 origin 캡처

엔진 (xr_teleop run_teleop_ur10e_ws.py 의 r/c/p 키 + relative motion) 을
teleop_dev sender 구조에 맞춰 이식:
  - sender_base 의 _apply_delta (누적) 모델 대신, XRRelativeFrameAligner 가
    절대 target pose 를 매 loop 마다 계산.
  - virtual_pos / virtual_quat 를 매 loop 에서 직접 set.
  - 키 매핑:
      r — recalibrate (사용자 손목 + 로봇 origin 둘 다 다시 capture)
      p — pause toggle (수동 — 손 새 위치로 옮길 때. resume 시 자동 recalibrate)
      c — immediate recalibrate (pause 없이 즉시 — jump 가능)
      x / Esc / q — quit
      space — estop (robot 측 즉시 정지)
      +/- — robot 측 speed scale

Pipeline:
    BridgePoseStore.right_arm_pose (4×4 WebXR local-floor)
        → XRRelativeFrameAligner.apply → (target_pos, target_quat) base_link
        → TeleopPosePacket → UDP 9871
        → robot/arm/admittance|impedance/main.py + UnifiedNetworkInput
"""

from __future__ import annotations

import argparse
import queue
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from sender.arm.sender_base import (
    TeleopSenderBase,
    InputResult,
)
from sender.arm.xr_frame_align import XRRelativeFrameAligner
from sender.xr_common import BridgePoseStore
from sender.xr_common.watchdog import StoreWatchdog, WorkspaceLimits

from protocol.arm_protocol import ButtonState


def _is_valid_pose(pose: np.ndarray) -> bool:
    if pose is None or pose.shape != (4, 4):
        return False
    if not np.isfinite(pose).all():
        return False
    if np.allclose(pose, 0):
        return False
    if abs(pose[3, 3] - 1.0) > 1e-6:
        return False
    return True


class XRArmSender(TeleopSenderBase):
    """WebXR wrist → robot base_link target pose sender (relative motion).

    sender_base 의 _apply_delta (incremental) 패턴과 다르게, **매 loop 에서
    절대 pose 를 set** 한다 — XRRelativeFrameAligner 가 origin + delta 로 직접
    target 계산. _read_input 은 InputResult 의 buttons / quit 만 채우고
    delta_pos / delta_rot 은 zeros (sender_base 가 _apply_delta(0,0) 호출 →
    virtual_pose 변경 없음).

    Override `run()` 도 가능하지만 sender_base 의 query_pose 핸드셰이크 / quit
    / reset / log 흐름 그대로 활용 위해 _read_input 으로 통합.
    """

    def __init__(
        self,
        target_ip: str,
        port: int = 9871,
        hz: int = 50,
        scale: float = 1.0,
        convention: str = "mediapipe",
        bridge_port: Optional[int] = None,
        hand_side: str = "right",
        no_keyboard: bool = False,
        watchdog_timeout_s: float = 0.2,
        workspace: Optional[WorkspaceLimits] = None,
        enforce_workspace: bool = True,
        r_remap: Optional[np.ndarray] = None,
        key_queue: Optional["queue.Queue"] = None,
        label: str = "",
    ):
        """
        r_remap : (3,3) WebXR→base_link 축 정렬 회전. None 이면 기존 기본값.
                  팔 마운트 방향이 다른 dual-arm 에서 팔별로 지정.
        key_queue : 지정 시 termios 대신 이 queue 에서 키를 읽음 — 여러 arm
                  sender 를 한 process 의 background thread 로 돌릴 때
                  (termios 는 process 당 1 소유자만 가능) main thread 가
                  키를 broadcast 하는 용도.
        label : 로그 prefix (예: "left" → [XRArmSender:left]). dual-arm 로그 구분.
        """
        super().__init__(target_ip, port, hz)
        self.scale = scale
        self.convention = convention
        self.hand_side = hand_side
        self.no_keyboard = no_keyboard
        self.enforce_workspace = enforce_workspace
        self.workspace = workspace if workspace is not None else WorkspaceLimits()
        self.key_queue = key_queue
        self._tag = f"[XRArmSender:{label}]" if label else "[XRArmSender]"
        if label:
            # dual 모드: \r 한 줄 status 가 thread 끼리 겹치므로 plain 주기 출력
            self._status_inline = False
            self._status_label = label

        # BridgePoseStore singleton — Phase B sender 와 같은 instance 공유 가능
        self._store = BridgePoseStore(use_hand_tracking=True, port=bridge_port)
        # 프로세스 재시작 + 헤드셋 ws 자동 reconnect 시나리오에서 이전 session 의
        # 마지막 pose 가 shared array 에 남아있을 수 있는 corner-case 차단.
        # 새 'r' 호출 시 freshness check (calibrate_now) 와 함께 stale origin
        # 캡처 방지.
        self._store.clear_state()
        print(f"{self._tag} BridgePoseStore ready: http://localhost:{self._store.port}/")

        self._aligner = XRRelativeFrameAligner(scale=scale, r_remap=r_remap)
        self._watchdog = StoreWatchdog(self._store, timeout_s=watchdog_timeout_s)
        self._clamp_warn_count = 0
        self._invalid_pose_count = 0

        # state machine
        self._started = False     # 'r' 키 누르기 전에는 robot origin 만 유지
        self._paused = False
        self._old_settings = None
        # no-keyboard 모드 자동 calibrate 재시도 rate-limit (1s)
        self._last_auto_calib_attempt = 0.0
        # 새 process 시작 시각 — 'r' calibrate 호출 시 BridgePoseStore msg 가
        # 이 시각 이후의 fresh msg 인지 확인하여 stale data 로 origin 잡히는 사고 방지.
        self._sender_start_time = time.perf_counter()

        # speed scale (sender_base 의 _get_speed_label 호환)
        self._speed_idx = 1
        self._speed_presets = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]

    @property
    def speed_scale(self) -> float:
        return self._speed_presets[self._speed_idx]

    def _get_speed_label(self) -> str:
        state = []
        if not self._started:
            state.append("READY")
        if self._paused:
            state.append("PAUSE")
        if self._aligner.calibrated and self._started and not self._paused:
            state.append("SYNC")
        state.append(f"x{self._speed_presets[self._speed_idx]:.1f}")
        return " ".join(state)

    # ── device setup ──────────────────────────────────────────────────
    def _setup_device(self):
        tag = self._tag
        print(f"\n{tag} ─────────────────────────────────────────────")
        print(f"{tag} keys:")
        print("  r       — sync 시작 또는 recalibrate (사용자 + robot origin 동시 capture)")
        print("  p       — pause / resume (resume 시 자동 recalibrate)")
        print("  c       — immediate recalibrate (pause 없이, jump 가능)")
        print("  Space   — E-Stop")
        print("  +/-     — speed (robot 측 적용)")
        print("  x / Esc — quit")
        print(f"{tag} hand_side = {self.hand_side}  convention = {self.convention}"
              f"  scale = {self.scale}")
        print(f"{tag} Quest 3 / Galaxy XR Chrome → http://localhost:"
              f"{self._store.port}/ → Enter VR/AR → 손 들이밀기 → 'r' 키")
        print(f"{tag} ⚠️  Chrome 창 / 헤드셋의 정면이 robot base_link 의")
        print(f"{tag}    정면 (+x) 과 align 된 자세에서 'r' 을 누르세요.")
        print(f"{tag}    헤드셋이 N° 회전한 자세로 'r' 을 누르면 손 motion")
        print(f"{tag}    이 N° 어긋난 frame 으로 매핑돼 robot 거동이 의도와")
        print(f"{tag}    회전된 채로 나옴. (xr_frame_align.py docstring 참조)")
        print(f"{tag} ─────────────────────────────────────────────")
        if self.key_queue is not None:
            # dual 모드: main thread 의 key dispatcher 가 termios 소유.
            # 키는 key_queue 로 수신 — _started 는 'r' 수신 시 전이.
            print(f"{tag} key_queue 모드 — main thread 의 키 입력이 broadcast 됨")
            return
        if self.no_keyboard:
            print(f"{tag} keyboard 비활성 (no-keyboard=True). "
                  "헤드셋 fresh msg 확보되면 자동 calibrate (1s 간격 재시도).")
            self._started = True
            return
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _cleanup_device(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _read_key(self) -> Optional[str]:
        if self.key_queue is not None:
            try:
                return self.key_queue.get_nowait()
            except queue.Empty:
                return None
        if self.no_keyboard:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[" and select.select([sys.stdin], [], [], 0.01)[0]:
                        sys.stdin.read(1)
                return "ESC"
            return ch
        return None

    def _user_pose(self) -> np.ndarray:
        """hand_side 에 따라 BridgePoseStore 의 손목 pose (4×4 WebXR local-floor) 반환."""
        if self.hand_side == "right":
            return self._store.right_arm_pose
        return self._store.left_arm_pose

    # ── input loop — override 패턴 ────────────────────────────────────
    def _read_input(self) -> InputResult:
        """sender_base 의 _read_input 호출. 매 loop 마다:

        1) key handling
        2) bridge 의 right_arm_pose 로 절대 target 계산
        3) virtual_pos / virtual_quat 직접 set
        4) 키 buttons 만 InputResult 에 채움 (delta=0)
        """
        result = InputResult()
        buttons = ButtonState()

        key = self._read_key()
        if key is not None:
            if key in ("x", "q", "ESC"):
                buttons.quit = True
                result.buttons = buttons
                result.quit = True
                return result
            if key == " ":
                buttons.estop = True
                result.buttons = buttons
                return result
            if key == "r":
                self._calibrate_now(label="r")
                self._started = True
            elif key == "c":
                self._calibrate_now(label="c (immediate)")
                self._started = True
            elif key == "p":
                self._paused = not self._paused
                if self._paused:
                    print(f"\n{self._tag} ⏸  PAUSED — robot 명령 유지. 손 새 위치로 옮긴 후 'p' 다시 누르면 resume + 자동 recalibrate.")
                else:
                    print(f"\n{self._tag} ▶  RESUMED — recalibrate.")
                    self._calibrate_now(label="p resume")
                    self._started = True
            elif key in ("+", "="):
                self._speed_idx = min(self._speed_idx + 1, len(self._speed_presets) - 1)
                buttons.speed_up = True
                print(f"\n{self._tag} speed = x{self.speed_scale:.1f}")
            elif key == "-":
                self._speed_idx = max(self._speed_idx - 1, 0)
                buttons.speed_down = True
                print(f"\n{self._tag} speed = x{self.speed_scale:.1f}")

        # 2) target pose 계산
        if not self._started or self._paused:
            # 시작 전 / paused: virtual_pose 유지 (sender_base 가 송신)
            result.buttons = buttons
            return result

        # 2.5) watchdog — BridgePoseStore msg 신선도 확인.
        # stale 이면 virtual_pose 유지 (마지막 valid target 송신 — robot 측이 자체
        # admittance / safety 로 stationary 또는 천천히 정지).
        if not self._watchdog.fresh():
            if self._watchdog.stale_count in (1, 30, 300):  # 1회 / 0.6s / 6s
                print(
                    f"\n{self._tag} WARN: BridgePoseStore stale (count={self._watchdog.stale_count})"
                    " — virtual_pose 유지, robot 명령 freeze"
                )
            result.buttons = buttons
            return result

        # 2.6) 캘리브레이션 완료 전에는 virtual_pose 를 절대 건드리지 않음.
        # _calibrate_now 가 실패해도 _started=True 가 될 수 있고 (r 키 stale 등),
        # 그 상태에서 aligner.apply() 는 (0,0,0) 을 반환해 robot 이 base 원점으로
        # 기어가는 사고 방지. no-keyboard 모드는 여기서 1s 간격 자동 재시도.
        if not self._aligner.calibrated:
            if self.no_keyboard:
                now = time.perf_counter()
                if now - self._last_auto_calib_attempt >= 1.0:
                    self._last_auto_calib_attempt = now
                    self._calibrate_now(label="auto (no-keyboard)")
            result.buttons = buttons
            return result

        user_pose = self._user_pose()

        # 2.65) invalid pose 방어 — watchdog 은 msg 신선도만 보므로, 손 tracking
        # 유실 등으로 pose 가 zeros/비유한이면 virtual_pose 유지 (freeze).
        if not _is_valid_pose(user_pose):
            self._invalid_pose_count += 1
            if self._invalid_pose_count in (1, 30, 300):
                print(
                    f"\n{self._tag} WARN: user wrist pose invalid "
                    f"(count={self._invalid_pose_count}) — virtual_pose 유지"
                )
            result.buttons = buttons
            return result
        self._invalid_pose_count = 0

        target_pos, target_quat = self._aligner.apply(user_pose)

        # 2.7) workspace clamp (sender 측 안전 layer). robot PC 측 safety_monitor
        # 가 별도로 envelope 검사하지만 sender 측에서도 clamp 해 큰 jump 회피.
        if self.enforce_workspace:
            clamped, was_clamped = self.workspace.clamp(target_pos)
            if was_clamped:
                self._clamp_warn_count += 1
                if self._clamp_warn_count in (1, 30, 300):
                    print(
                        f"\n{self._tag} WARN: target pos {target_pos} → clamped {clamped}"
                        f" (count={self._clamp_warn_count})"
                    )
                target_pos = clamped

        # 3) virtual_pos / virtual_quat 직접 set (sender_base 가 _send_packet 에서 사용)
        self._virtual_pos = target_pos
        self._virtual_quat = target_quat

        result.buttons = buttons
        return result

    def _calibrate_now(self, label: str = "") -> None:
        """user origin + robot origin (query_pose 응답) 동시 캡처.

        Stale data 차단: BridgePoseStore 의 last_msg_time 이 sender_start_time
        이후의 fresh msg 인지 확인. 헤드셋 ws 가 아직 연결 안 됐거나, 이전
        session 의 마지막 pose 가 shared array 에 남아있다면 calibrate 거부.
        """
        # 1) freshness check — 새 process 시작 이후 들어온 fresh msg 만 valid
        stats = self._store.get_stats()
        last_msg_time = stats.get("last_msg_time", 0.0)
        if last_msg_time <= 0.0:
            print(f"\n{self._tag} WARN: 헤드셋 ws msg 없음 — 헤드셋 사이트 접속 + Enter VR/AR + 손 들이밀고 '{label}' 재시도")
            return
        if last_msg_time < self._sender_start_time:
            print(
                f"\n{self._tag} WARN: BridgePoseStore msg 가 sender 시작 시각보다 오래됨 "
                f"(stale). 헤드셋 사이트 새로고침 후 '{label}' 재시도"
            )
            return
        age = time.perf_counter() - last_msg_time
        if age > 0.5:
            print(
                f"\n{self._tag} WARN: 마지막 msg age={age:.2f}s > 0.5s — 헤드셋 연결 stale."
                f" 손 들이밀어 fresh data 확보 후 '{label}' 재시도"
            )
            return

        # 2) robot 의 현재 TCP pose 재조회 (sender_base.query_initial_pose 그대로)
        # blocking IO 가 필요해 raw socket 일시 blocking 으로 전환
        was_blocking = self._sock.getblocking()
        self._sock.setblocking(True)
        robot_pos, robot_quat = self.query_initial_pose()
        self._sock.setblocking(was_blocking)

        # 3) user origin
        user_pose = self._user_pose()

        if not _is_valid_pose(user_pose):
            print(f"\n{self._tag} WARN: user wrist pose 가 아직 invalid — 손 시야 안 들이밀고 '{label}' 재시도")
            return

        # 4) aligner 캘리브레이션
        print(f"\n{self._tag} calibrate ({label})")
        self._aligner.calibrate(user_pose, robot_pos, robot_quat)
        # virtual = origin 으로 set (jump 회피)
        self._virtual_pos = robot_pos.copy()
        self._virtual_quat = robot_quat.copy()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="XR arm sender (Galaxy XR / Quest 3 → UR10e via UDP 9871)"
    )
    parser.add_argument("--target-ip", required=True, help="Robot PC IP")
    parser.add_argument("--port", type=int, default=9871, help="UDP target port")
    parser.add_argument("--hz", type=int, default=50, help="Send rate Hz")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Position scale (1.0 = 1:1). rotation 은 항상 1:1")
    parser.add_argument("--convention", default="mediapipe",
                        choices=["mediapipe", "manus"],
                        help="WebXR → MANO frame chirality (현재 미사용, hand sender 와 동일 옵션 유지)")
    parser.add_argument("--bridge-port", type=int, default=None,
                        help="BridgePoseStore ws port (default 8013 or env XR_BRIDGE_PORT)")
    parser.add_argument("--hand", default="right", choices=["right", "left"])
    parser.add_argument("--no-keyboard", action="store_true",
                        help="키 입력 없이 즉시 시작 (sshd / headless). 사용자가 손 시야 안 둔 채로 자동 calibrate 시도")
    parser.add_argument("--watchdog-timeout-s", type=float, default=0.2,
                        help="BridgePoseStore stale 판정 timeout (default 0.2s). "
                             "msg age 가 이 값 초과 시 virtual_pose 유지 + warn.")
    parser.add_argument("--no-workspace-clamp", action="store_true",
                        help="workspace envelope clamp 비활성. 위험 — 디버그 / 검증 후에만 사용")
    parser.add_argument("--workspace", default="default",
                        help="workspace envelope (현재 default 만 지원: "
                             "x=[-0.7,0.7], y=[-0.7,0.7], z=[0.05,0.8])")
    args = parser.parse_args()

    workspace = WorkspaceLimits() if args.workspace == "default" else None
    if workspace is None and not args.no_workspace_clamp:
        print(f"[XRArmSender] unknown workspace {args.workspace!r}; using default")
        workspace = WorkspaceLimits()

    sender = XRArmSender(
        target_ip=args.target_ip,
        port=args.port,
        hz=args.hz,
        scale=args.scale,
        convention=args.convention,
        bridge_port=args.bridge_port,
        hand_side=args.hand,
        no_keyboard=args.no_keyboard,
        watchdog_timeout_s=args.watchdog_timeout_s,
        workspace=workspace,
        enforce_workspace=not args.no_workspace_clamp,
    )
    sender.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
