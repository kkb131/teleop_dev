#!/usr/bin/env python3
"""DG5F preset pose test: spread (open hand) → fist (closed hand).

Requires dg5f_driver running:
    ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

Usage:
    python3 -m robot.hand.tests.test_pose --hand right
    python3 -m robot.hand.tests.test_pose --hand right --hold 3.0
    python3 -m robot.hand.tests.test_pose --hand right --pose spread
    python3 -m robot.hand.tests.test_pose --hand right --pose fist

Record mode (teach poses by hand):
    1. Manually move DG5F to desired spread position
    2. python3 -m robot.hand.tests.test_pose --hand right --record spread
    3. Manually move DG5F to desired fist position
    4. python3 -m robot.hand.tests.test_pose --hand right --record fist
    Saved to: robot/hand/config/poses.json
"""

import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState

RIGHT_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",  # Thumb
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",  # Index
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",  # Middle
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",  # Ring
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",  # Pinky
]
LEFT_JOINTS = [j.replace("rj_", "lj_") for j in RIGHT_JOINTS]

d = math.radians


# ─────────────────────────────────────────────────────────
# Pose definitions (right hand, radians)
# Joint order per finger: [spread, MCP_flex, PIP_flex, DIP_flex]
# ─────────────────────────────────────────────────────────

# Spread: open hand, fingers wide apart
SPREAD_RIGHT = [
    # Thumb: spread out, extended
    d(30),  d(0),   d(0),   d(0),
    # Index: spread out, extended
    d(20),  d(0),   d(0),   d(0),
    # Middle: neutral spread, extended
    d(0),   d(0),   d(0),   d(0),
    # Ring: spread out (opposite direction), extended
    d(-20), d(0),   d(0),   d(0),
    # Pinky: spread out, extended
    d(30),  d(0),   d(0),   d(0),
]

# Fist: tight fist, thumb over fingers
FIST_RIGHT = [
    # Thumb: tucked over fingers
    d(0),   d(-60), d(60),  d(60),
    # Index: fully curled
    d(0),   d(80),  d(80),  d(80),
    # Middle: fully curled
    d(0),   d(80),  d(80),  d(80),
    # Ring: fully curled
    d(0),   d(80),  d(80),  d(80),
    # Pinky: fully curled
    d(0),   d(30),  d(80),  d(80),
]

# Zero: all joints at 0
ZERO = [0.0] * 20

# Left hand: mirror spread joints (negate spread angles)
SPREAD_LEFT = list(SPREAD_RIGHT)
for i in range(5):
    SPREAD_LEFT[i * 4] = -SPREAD_RIGHT[i * 4]  # negate spread

FIST_LEFT = list(FIST_RIGHT)
FIST_LEFT[1] = -FIST_RIGHT[1]  # thumb MCP flex direction


POSES = {
    "zero":   {"right": ZERO,         "left": ZERO},
    "spread": {"right": SPREAD_RIGHT, "left": SPREAD_LEFT},
    "fist":   {"right": FIST_RIGHT,   "left": FIST_LEFT},
}

# ─────────────────────────────────────────────────────────
# Poses JSON file (overrides hardcoded defaults if present)
# ─────────────────────────────────────────────────────────

POSES_JSON = os.path.join(os.path.dirname(__file__), "..", "config", "poses.json")


def _load_poses_json():
    """Load saved poses from JSON, overriding hardcoded defaults."""
    if not os.path.exists(POSES_JSON):
        return
    try:
        with open(POSES_JSON, "r") as f:
            saved = json.load(f)
        for pose_name in ["spread", "fist"]:
            for hand_side in ["right", "left"]:
                key = f"{pose_name}_{hand_side}"
                if key in saved:
                    POSES[pose_name][hand_side] = saved[key]
                    print(f"  [poses.json] Loaded {pose_name}/{hand_side} from saved file")
    except Exception as e:
        print(f"  [WARN] Failed to load poses.json: {e}")


def _save_pose_json(pose_name: str, hand_side: str, values: list):
    """Save a single pose to JSON (preserving other poses)."""
    saved = {}
    if os.path.exists(POSES_JSON):
        try:
            with open(POSES_JSON, "r") as f:
                saved = json.load(f)
        except Exception:
            pass

    key = f"{pose_name}_{hand_side}"
    saved[key] = [round(v, 6) for v in values]

    os.makedirs(os.path.dirname(POSES_JSON), exist_ok=True)
    with open(POSES_JSON, "w") as f:
        json.dump(saved, f, indent=2)
    print(f"  Saved to: {POSES_JSON}")


# Load saved poses on import
_load_poses_json()


class PoseNode(Node):
    def __init__(self, hand: str):
        super().__init__("dg5f_pose_test")
        self._hand = hand
        prefix = f"dg5f_{hand}"
        side_prefix = "rj" if hand == "right" else "lj"
        self._joints = RIGHT_JOINTS if hand == "right" else LEFT_JOINTS

        pid_topic = f"/{prefix}/{side_prefix}_dg_pospid/reference"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(MultiDOFCommand, pid_topic, qos)

        js_topic = f"/{prefix}/joint_states"
        self._js_latest = None
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        self.get_logger().info(f"Publishing to: {pid_topic}")

    def _js_cb(self, msg: JointState):
        self._js_latest = msg

    def send_pose(self, positions: list):
        msg = MultiDOFCommand()
        msg.dof_names = list(self._joints)
        msg.values = [float(p) for p in positions]
        self._pub.publish(msg)

    def spin_for(self, seconds: float):
        t0 = time.monotonic()
        while (time.monotonic() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_feedback(self, timeout=5.0):
        self.get_logger().info("Waiting for joint_states...")
        t0 = time.monotonic()
        while self._js_latest is None and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._js_latest is not None


def main():
    parser = argparse.ArgumentParser(description="DG5F spread → fist pose test")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--hold", type=float, default=2.0,
                        help="Hold time per pose (seconds)")
    parser.add_argument("--pose", default=None,
                        choices=["spread", "fist", "zero"],
                        help="Send a single pose instead of the full sequence")
    parser.add_argument("--loop", action="store_true",
                        help="Loop spread→fist continuously until Ctrl+C")
    parser.add_argument("--record", default=None,
                        choices=["spread", "fist"],
                        help="Record current joint_states as a pose preset")
    args = parser.parse_args()

    rclpy.init()
    node = PoseNode(hand=args.hand)

    if not node.wait_for_feedback():
        print("[ERROR] No joint_states received. Is the driver running?")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        if args.record:
            # Record mode: read joint_states and save as pose preset
            print(f"\n  === RECORD MODE: {args.record} ({args.hand} hand) ===")
            print(f"  Move the DG5F hand to the desired '{args.record}' position.")
            print(f"  Press Enter when ready to record...")
            input()

            # Sample joint_states multiple times for stability
            print("  Sampling joint_states (10 samples)...")
            samples = []
            js_names = None
            for _ in range(10):
                node.spin_for(0.05)
                if node._js_latest and len(node._js_latest.position) >= 20:
                    if js_names is None:
                        js_names = list(node._js_latest.name)
                    samples.append(list(node._js_latest.position))
            if not samples or js_names is None:
                print("  [ERROR] No joint_states received!")
            else:
                raw_mean = np.mean(samples, axis=0)

                # Reorder from joint_states name order → RIGHT_JOINTS order
                target_joints = node._joints
                name_to_idx = {name: i for i, name in enumerate(js_names)}
                mean_pos = []
                for jname in target_joints:
                    if jname in name_to_idx:
                        mean_pos.append(float(raw_mean[name_to_idx[jname]]))
                    else:
                        print(f"  [WARN] Joint '{jname}' not found in joint_states!")
                        mean_pos.append(0.0)

                print(f"\n  joint_states order: {js_names[:5]}...")
                print(f"  target order:      {target_joints[:5]}...")

                # Print recorded values
                finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
                print(f"\n  Recorded '{args.record}' pose ({args.hand}):")
                for fi, fname in enumerate(finger_names):
                    vals = mean_pos[fi*4:(fi+1)*4]
                    degs = [math.degrees(v) for v in vals]
                    print(f"    {fname:8s}: [{', '.join(f'{v:+7.3f}' for v in vals)}] rad")
                    print(f"    {'':8s}  [{', '.join(f'{v:+6.1f}°' for v in degs)}]")

                # Save
                _save_pose_json(args.record, args.hand, mean_pos)
                POSES[args.record][args.hand] = mean_pos
                print(f"\n  Updated POSES['{args.record}']['{args.hand}']")
                print(f"  Test with: python3 -m robot.hand.tests.test_pose --hand {args.hand} --pose {args.record}")

        elif args.pose:
            # Single pose
            pose = POSES[args.pose][args.hand]
            print(f"\n  Sending pose: {args.pose}")
            node.send_pose(pose)
            node.spin_for(args.hold)
        elif args.loop:
            # Loop: spread → fist → spread → fist ...
            cycle = 0
            print(f"\n  Looping spread → fist (hold={args.hold}s). Ctrl+C to stop.")
            while True:
                print(f"\n  [{cycle}] SPREAD (open hand)")
                node.send_pose(POSES["spread"][args.hand])
                node.spin_for(args.hold)

                print(f"  [{cycle}] FIST (closed hand)")
                node.send_pose(POSES["fist"][args.hand])
                node.spin_for(args.hold)
                cycle += 1
        else:
            # Default sequence: zero → spread → fist → zero
            print(f"\n  Sequence: zero → spread → fist → zero")
            print(f"  Hold time: {args.hold}s per pose\n")

            print("  [1/4] ZERO (home)")
            node.send_pose(POSES["zero"][args.hand])
            node.spin_for(args.hold)

            print("  [2/4] SPREAD (open hand, fingers wide)")
            node.send_pose(POSES["spread"][args.hand])
            node.spin_for(args.hold)

            print("  [3/4] FIST (tight fist)")
            node.send_pose(POSES["fist"][args.hand])
            node.spin_for(args.hold)

            print("  [4/4] ZERO (home)")
            node.send_pose(POSES["zero"][args.hand])
            node.spin_for(args.hold)

            print("\n  Done.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted. Returning to zero...")
        node.send_pose(ZERO)
        node.spin_for(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
