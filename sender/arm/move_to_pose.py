#!/usr/bin/env python3
"""Move UR10e to a specified joint pose via ur_rtde moveJ.

Usage:
    python3 -m sender.arm.move_to_pose --robot-ip 192.168.0.2
    python3 -m sender.arm.move_to_pose --robot-ip 192.168.0.2 --speed 0.5 --accel 0.3
    python3 -m sender.arm.move_to_pose --robot-ip 192.168.0.2 --joints 2.40 -1.18 2.06 -0.88 2.24 0.15
"""

import argparse
import math

import rtde_control
import rtde_receive

# Default target pose (radians)
DEFAULT_JOINTS = [2.40, -1.18, 2.06, -0.88, 2.24, 0.15]

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def main():
    parser = argparse.ArgumentParser(description="Move UR10e to joint pose via moveJ")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--speed", type=float, default=0.3, help="Joint speed (rad/s, default: 0.3)")
    parser.add_argument("--accel", type=float, default=0.3, help="Joint acceleration (rad/s^2, default: 0.3)")
    parser.add_argument("--joints", type=float, nargs=6, default=None,
                        help="Target joint angles in radians (6 values)")
    args = parser.parse_args()

    target = args.joints if args.joints else DEFAULT_JOINTS

    print(f"[MoveToJoint] Connecting to {args.robot_ip}...")
    recv = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    ctrl = rtde_control.RTDEControlInterface(args.robot_ip)

    current = recv.getActualQ()
    print(f"[MoveToJoint] Current joints (rad):")
    for name, val in zip(JOINT_NAMES, current):
        print(f"  {name:25s}: {val:+.4f} ({math.degrees(val):+.1f}°)")

    print(f"\n[MoveToJoint] Target joints (rad):")
    for name, val in zip(JOINT_NAMES, target):
        print(f"  {name:25s}: {val:+.4f} ({math.degrees(val):+.1f}°)")

    print(f"\n[MoveToJoint] Moving... (speed={args.speed} rad/s, accel={args.accel} rad/s²)")
    ctrl.moveJ(target, args.speed, args.accel)

    final = recv.getActualQ()
    print(f"[MoveToJoint] Done. Final joints (rad):")
    for name, val in zip(JOINT_NAMES, final):
        print(f"  {name:25s}: {val:+.4f} ({math.degrees(val):+.1f}°)")

    ctrl.stopScript()
    recv.disconnect()


if __name__ == "__main__":
    main()
