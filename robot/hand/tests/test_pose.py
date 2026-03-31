#!/usr/bin/env python3
"""DG5F preset pose test: spread (open hand) → fist (closed hand).

Requires dg5f_driver running:
    ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

Usage:
    python3 -m robot.hand.tests.test_pose --hand right
    python3 -m robot.hand.tests.test_pose --hand right --hold 3.0
    python3 -m robot.hand.tests.test_pose --hand right --pose spread
    python3 -m robot.hand.tests.test_pose --hand right --pose fist
"""

import argparse
import math
import time

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
    args = parser.parse_args()

    rclpy.init()
    node = PoseNode(hand=args.hand)

    if not node.wait_for_feedback():
        print("[ERROR] No joint_states received. Is the driver running?")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        if args.pose:
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
