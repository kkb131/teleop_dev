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
    ):
        super().__init__(target_ip, port, hz)
        self.scale = scale
        self.convention = convention
        self.hand_side = hand_side
        self.no_keyboard = no_keyboard

        # BridgePoseStore singleton — Phase B sender 와 같은 instance 공유 가능
        self._store = BridgePoseStore(use_hand_tracking=True, port=bridge_port)
        print(f"[XRArmSender] BridgePoseStore ready: http://localhost:{self._store.port}/")

        self._aligner = XRRelativeFrameAligner(scale=scale)

        # state machine
        self._started = False     # 'r' 키 누르기 전에는 robot origin 만 유지
        self._paused = False
        self._old_settings = None

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
        print("\n[XRArmSender] ─────────────────────────────────────────────")
        print("[XRArmSender] keys:")
        print("  r       — sync 시작 또는 recalibrate (사용자 + robot origin 동시 capture)")
        print("  p       — pause / resume (resume 시 자동 recalibrate)")
        print("  c       — immediate recalibrate (pause 없이, jump 가능)")
        print("  Space   — E-Stop")
        print("  +/-     — speed (robot 측 적용)")
        print("  x / Esc — quit")
        print("[XRArmSender] convention =", self.convention, "  scale =", self.scale)
        print("[XRArmSender] Quest 3 / Galaxy XR Chrome → http://localhost:"
              f"{self._store.port}/ → Enter VR/AR → 손 들이밀기 → 'r' 키")
        print("[XRArmSender] ─────────────────────────────────────────────")
        if self.no_keyboard:
            print("[XRArmSender] keyboard 비활성 (no-keyboard=True). 시작 시 즉시 calibrate.")
            self._started = True
            return
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _cleanup_device(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _read_key(self) -> Optional[str]:
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
                    print(f"\n[XRArmSender] ⏸  PAUSED — robot 명령 유지. 손 새 위치로 옮긴 후 'p' 다시 누르면 resume + 자동 recalibrate.")
                else:
                    print(f"\n[XRArmSender] ▶  RESUMED — recalibrate.")
                    self._calibrate_now(label="p resume")
                    self._started = True
            elif key in ("+", "="):
                self._speed_idx = min(self._speed_idx + 1, len(self._speed_presets) - 1)
                buttons.speed_up = True
                print(f"\n[XRArmSender] speed = x{self.speed_scale:.1f}")
            elif key == "-":
                self._speed_idx = max(self._speed_idx - 1, 0)
                buttons.speed_down = True
                print(f"\n[XRArmSender] speed = x{self.speed_scale:.1f}")

        # 2) target pose 계산
        if not self._started or self._paused:
            # 시작 전 / paused: virtual_pose 유지 (sender_base 가 송신)
            result.buttons = buttons
            return result

        if self.hand_side == "right":
            user_pose = self._store.right_arm_pose
        else:
            user_pose = self._store.left_arm_pose

        target_pos, target_quat = self._aligner.apply(user_pose)

        # 3) virtual_pos / virtual_quat 직접 set (sender_base 가 _send_packet 에서 사용)
        self._virtual_pos = target_pos
        self._virtual_quat = target_quat

        result.buttons = buttons
        return result

    def _calibrate_now(self, label: str = "") -> None:
        """user origin + robot origin (query_pose 응답) 동시 캡처."""
        # 1) robot 의 현재 TCP pose 재조회 (sender_base.query_initial_pose 그대로)
        # blocking IO 가 필요해 raw socket 일시 blocking 으로 전환
        was_blocking = self._sock.getblocking()
        self._sock.setblocking(True)
        robot_pos, robot_quat = self.query_initial_pose()
        self._sock.setblocking(was_blocking)

        # 2) user origin
        if self.hand_side == "right":
            user_pose = self._store.right_arm_pose
        else:
            user_pose = self._store.left_arm_pose

        if not _is_valid_pose(user_pose):
            print(f"\n[XRArmSender] WARN: user wrist pose 가 아직 invalid — 손 시야 안 들이밀고 '{label}' 재시도")
            return

        # 3) aligner 캘리브레이션
        print(f"\n[XRArmSender] calibrate ({label})")
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
    args = parser.parse_args()

    sender = XRArmSender(
        target_ip=args.target_ip,
        port=args.port,
        hz=args.hz,
        scale=args.scale,
        convention=args.convention,
        bridge_port=args.bridge_port,
        hand_side=args.hand,
        no_keyboard=args.no_keyboard,
    )
    sender.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
