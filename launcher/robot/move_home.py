#!/usr/bin/env python3
"""초기 위치 이동 oneshot — 팔 yaml 의 initial_pose 로 moveJ.

웹 대시보드의 '초기 위치 이동' 버튼이 이 스크립트를 oneshot 프로세스로
실행한다 (해당 팔 수신부가 정지된 상태에서만 — RTDE 제어 배타성).
터미널에서 직접 실행도 가능:

    python3 -m launcher.robot.move_home --side right \\
        --config robot/arm/admittance/config/right.yaml --robot-ip 192.168.0.2

안전 규칙:
    - 팔 yaml 의 initial_pose.enabled 가 false 면 RTDE 연결 전에 거부
      (exit 2). 왼팔은 home 실측 후 left.yaml 에 기입해야 사용 가능 —
      docs/xr_dual_arm_left_tuning_ko.md 참조.
    - joint_values 는 반드시 6개 (rad).

exit code: 0 성공 / 1 인자·연결 오류 / 2 안전 거부 (enabled=false 등)
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _load_initial_pose(config_path: str) -> list:
    """팔 yaml 에서 initial_pose 읽기 + 안전 검증. 실패 시 SystemExit."""
    path = Path(config_path)
    if not path.exists():
        print(f"[move_home] ERROR: config 없음: {path}")
        raise SystemExit(1)
    data = yaml.safe_load(path.read_text()) or {}
    ip = data.get("initial_pose") or {}

    if not ip.get("enabled", False):
        print(f"[move_home] 거부: {path.name} 의 initial_pose.enabled 가 false.")
        print("[move_home] 이 팔의 home 자세가 아직 실측·검증되지 않았다는 뜻입니다.")
        print("[move_home] freedrive 로 안전한 home 을 잡고 joint 값을 yaml 에 기입한 뒤")
        print("[move_home] enabled: true 로 전환하세요 (docs/xr_dual_arm_left_tuning_ko.md §6).")
        raise SystemExit(2)

    joints = ip.get("joint_values")
    if not isinstance(joints, list) or len(joints) != 6:
        print(f"[move_home] 거부: initial_pose.joint_values 가 6개가 아님: {joints}")
        raise SystemExit(2)
    return [float(v) for v in joints]


def _print_joints(label: str, joints) -> None:
    print(f"[move_home] {label}:")
    for name, val in zip(JOINT_NAMES, joints):
        print(f"  {name:25s}: {val:+.4f} rad ({math.degrees(val):+.1f}°)")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="팔 yaml initial_pose 로 moveJ (수신부 정지 상태에서만)")
    parser.add_argument("--side", choices=["right", "left"], required=True,
                        help="로그 표기용 팔 구분")
    parser.add_argument("--config", required=True,
                        help="팔 yaml (initial_pose.joint_values 를 읽음)")
    parser.add_argument("--robot-ip", required=True, help="UR10e IP")
    parser.add_argument("--speed", type=float, default=0.3, help="rad/s (default 0.3)")
    parser.add_argument("--accel", type=float, default=0.3, help="rad/s² (default 0.3)")
    args = parser.parse_args()

    # 1) 안전 검증 (RTDE 연결 전)
    target = _load_initial_pose(args.config)
    print(f"[move_home] side={args.side}  robot={args.robot_ip}  "
          f"speed={args.speed} accel={args.accel}")

    # 2) RTDE 연결 (import 를 여기서 — 검증 실패 시 ur_rtde 불필요)
    import rtde_control
    import rtde_receive

    print(f"[move_home] {args.robot_ip} 연결 중...")
    recv = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    ctrl = rtde_control.RTDEControlInterface(args.robot_ip)
    try:
        _print_joints("현재 joints", recv.getActualQ())
        _print_joints("목표 joints", target)
        print("[move_home] 이동 중...")
        ctrl.moveJ(target, args.speed, args.accel)
        _print_joints("완료 joints", recv.getActualQ())
    finally:
        try:
            ctrl.stopScript()
        except Exception:
            pass
        recv.disconnect()
    print("[move_home] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
