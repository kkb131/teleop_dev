#!/usr/bin/env python3
"""Compare different JointTrajectory publishing strategies on DG5F.

Tests 3 modes to find which gives best stiffness + responsiveness:
  A) time_from_start=0 + 60Hz single publish (instant position target)
  B) time_from_start=500ms + 60Hz single publish (current receiver.py)
  C) time_from_start=500ms + triple publish + header.stamp (test_tuning.py style)

Requires dg5f_driver running.

Usage:
    python3 -m teleop_dev.robot.hand.tests.test_command_mode --hand right
"""

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

RIGHT_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]
LEFT_JOINTS = [j.replace("rj_", "lj_") for j in RIGHT_JOINTS]


class CommandTestNode(Node):
    def __init__(self, hand: str):
        super().__init__("dg5f_command_test")
        self._hand = hand
        prefix = f"dg5f_{hand}"
        self._joints = RIGHT_JOINTS if hand == "right" else LEFT_JOINTS

        traj_topic = f"/{prefix}/{prefix}_controller/joint_trajectory"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(JointTrajectory, traj_topic, qos)

        js_topic = f"/{prefix}/joint_states"
        self._js_latest = None
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

    def _js_cb(self, msg):
        self._js_latest = msg

    def spin_for(self, seconds):
        t0 = time.monotonic()
        while (time.monotonic() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_feedback(self, timeout=5.0):
        print("  Waiting for joint_states...", end=" ", flush=True)
        t0 = time.monotonic()
        while self._js_latest is None and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._js_latest is None:
            print("[FAIL]")
            return False
        print("[OK]")
        return True

    def publish_mode_a(self, positions):
        """Mode A: time_from_start=0, single publish, with stamp."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self._joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=0.0).to_msg()
        msg.points = [point]
        self._pub.publish(msg)

    def publish_mode_b(self, positions, duration_s=0.5):
        """Mode B: time_from_start=duration, single publish, no stamp (current)."""
        msg = JointTrajectory()
        msg.joint_names = self._joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()
        msg.points = [point]
        self._pub.publish(msg)

    def publish_mode_c(self, positions, duration_s=0.5):
        """Mode C: time_from_start=duration, triple publish, with stamp (tuning style)."""
        msg = JointTrajectory()
        msg.joint_names = self._joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()
        msg.points = [point]
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)
            time.sleep(0.05)


def run_sine_test(node, mode_name, publish_fn, hz=30, duration=8.0):
    """Send a sine wave to Index PIP (joint 6) and measure stiffness.

    First 4 seconds: sine wave motion.
    Last 4 seconds: hold at 45 degrees — try pushing the finger!
    """
    print(f"\n  --- Mode {mode_name} (Hz={hz}) ---")
    print(f"  Phase 1 (0-4s): Sine wave on Index PIP")
    print(f"  Phase 2 (4-8s): HOLD at 45deg — try pushing the finger!")

    dt = 1.0 / hz
    target = [0.0] * 20
    start = time.time()

    try:
        while time.time() - start < duration:
            t = time.time() - start
            rclpy.spin_once(node, timeout_sec=0)

            if t < 4.0:
                # Sine wave phase
                angle = math.radians(45) * (math.sin(t * 2.0) * 0.5 + 0.5)
                target[6] = angle
            else:
                # Hold phase
                target[6] = math.radians(45)

            publish_fn(target)

            remaining = dt - (time.time() - start - int((time.time() - start) / dt) * dt)
            if remaining > 0:
                time.sleep(min(remaining, dt))
    except KeyboardInterrupt:
        pass

    print(f"  Done. Was it stiff during hold? Did it vibrate during sine?")


def main():
    parser = argparse.ArgumentParser(description="DG5F command mode comparison")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    args = parser.parse_args()

    rclpy.init()
    node = CommandTestNode(hand=args.hand)

    if not node.wait_for_feedback():
        node.destroy_node()
        rclpy.shutdown()
        return

    print("\n" + "=" * 60)
    print("  DG5F Command Mode Comparison")
    print("  Each mode: 4s sine wave → 4s hold (push test)")
    print("=" * 60)

    # Go to zero first
    print("\n  Going to zero...")
    node.publish_mode_c([0.0] * 20, duration_s=2.0)
    node.spin_for(3.0)

    # Mode A: instant position
    input("\n  Press Enter to start Mode A (instant, no duration)...")
    run_sine_test(node, "A: instant (time=0)", node.publish_mode_a, hz=30)

    # Reset
    node.publish_mode_c([0.0] * 20, duration_s=2.0)
    node.spin_for(2.0)

    # Mode B: current receiver.py style
    input("\n  Press Enter to start Mode B (500ms duration, single pub)...")
    run_sine_test(node, "B: 500ms single",
                  lambda pos: node.publish_mode_b(pos, 0.5), hz=30)

    # Reset
    node.publish_mode_c([0.0] * 20, duration_s=2.0)
    node.spin_for(2.0)

    # Mode C: test_tuning.py style
    input("\n  Press Enter to start Mode C (500ms duration, triple pub + stamp)...")
    run_sine_test(node, "C: 500ms triple+stamp",
                  lambda pos: node.publish_mode_c(pos, 0.5), hz=10)

    # Final reset
    print("\n  Returning to zero...")
    node.publish_mode_c([0.0] * 20, duration_s=2.0)
    node.spin_for(2.0)

    print("\n" + "=" * 60)
    print("  Which mode was best?")
    print("  A = instant position, most responsive, stiffness depends on PID")
    print("  B = current receiver.py approach")
    print("  C = test_tuning.py approach (triple publish + stamp)")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
