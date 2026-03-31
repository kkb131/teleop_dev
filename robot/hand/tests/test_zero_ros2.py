#!/usr/bin/env python3
"""Send all DG5F joints to 0 degrees via PidController reference topic.

Requires dg5f_driver running:
    ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

Usage:
    python3 -m robot.hand.tests.test_zero_ros2
    python3 -m robot.hand.tests.test_zero_ros2 --hand left
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

LEFT_JOINT_NAMES = [j.replace("rj_", "lj_") for j in RIGHT_JOINT_NAMES]


class ZeroPositionNode(Node):
    def __init__(self, hand: str):
        super().__init__("dg5f_zero_position")
        self._hand = hand

        prefix = f"dg5f_{hand}"
        side_prefix = "rj" if hand == "right" else "lj"
        self._joint_names = RIGHT_JOINT_NAMES if hand == "right" else LEFT_JOINT_NAMES

        # Publisher for PidController
        pid_topic = f"/{prefix}/{side_prefix}_dg_pospid/reference"
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(MultiDOFCommand, pid_topic, qos)

        # Subscriber for feedback
        js_topic = f"/{prefix}/joint_states"
        self._latest_js = None
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        self.get_logger().info(f"Publishing to: {pid_topic}")
        self.get_logger().info(f"Listening on: {js_topic}")

    def _js_cb(self, msg: JointState):
        self._latest_js = msg

    def wait_for_joint_states(self, timeout=5.0):
        self.get_logger().info("Waiting for joint_states...")
        t0 = time.monotonic()
        while self._latest_js is None and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest_js is None:
            self.get_logger().warn("No joint_states received (timeout)")
            return False
        self.get_logger().info(
            f"Got joint_states: {len(self._latest_js.position)} joints"
        )
        return True

    def send_zero(self):
        msg = MultiDOFCommand()
        msg.dof_names = list(self._joint_names)
        msg.values = [0.0] * 20
        self._pub.publish(msg)
        self.get_logger().info("Sent 0-degree command")

    def print_current_positions(self):
        if self._latest_js is None:
            return
        positions = self._latest_js.position
        print(f"\nCurrent positions ({len(positions)} joints):")
        names = self._latest_js.name
        for name, pos in zip(names, positions):
            print(f"  {name}: {pos:+.4f} rad ({pos * 180 / 3.14159:+.1f} deg)")


def main():
    parser = argparse.ArgumentParser(description="DG5F zero position via PidController")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    args = parser.parse_args()

    rclpy.init()
    node = ZeroPositionNode(hand=args.hand)

    try:
        if node.wait_for_joint_states():
            node.print_current_positions()

        node.send_zero()

        print("\nWaiting 3s for motion to settle...")
        t0 = time.monotonic()
        while (time.monotonic() - t0) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.print_current_positions()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Done")


if __name__ == "__main__":
    main()
