#!/usr/bin/env python3
"""Minimal direct publish test for DG5F PidController reference topic.

Usage:
    python3 -m teleop_dev.robot.hand.tests.test_direct_pub
    python3 -m teleop_dev.robot.hand.tests.test_direct_pub --hand right --target 0.5
    python3 -m teleop_dev.robot.hand.tests.test_direct_pub --hand left --target -0.3
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand

RIGHT_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

LEFT_JOINTS = [j.replace("rj_", "lj_") for j in RIGHT_JOINTS]


def main():
    parser = argparse.ArgumentParser(description="Direct PidController publish test")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--target", type=float, default=0.0, help="Target position (rad)")
    parser.add_argument("--count", type=int, default=5, help="Number of publishes")
    args = parser.parse_args()

    prefix = f"dg5f_{args.hand}"
    side_prefix = "rj" if args.hand == "right" else "lj"
    topic = f"/{prefix}/{side_prefix}_dg_pospid/reference"
    joints = RIGHT_JOINTS if args.hand == "right" else LEFT_JOINTS

    rclpy.init()
    node = Node("direct_pub_test")

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(MultiDOFCommand, topic, qos)

    print(f"Topic:    {topic}")
    print(f"Joints:   {len(joints)}")
    print(f"Target:   {args.target} rad")
    print(f"Count:    {args.count}x")

    # Wait for DDS discovery
    print("\nWaiting 2s for DDS discovery...")
    time.sleep(2.0)

    sub_count = pub.get_subscription_count()
    print(f"Subscribers on topic: {sub_count}")
    if sub_count == 0:
        print("[WARN] No subscribers! Controller may not be listening.")
        print(f"  Check: ros2 topic info {topic}")
        print(f"  Check: ros2 control list_controllers -c /{prefix}/controller_manager")

    # Build and publish
    msg = MultiDOFCommand()
    msg.dof_names = list(joints)
    msg.values = [args.target] * 20

    print(f"\nPublishing {args.count} times...")
    for i in range(args.count):
        pub.publish(msg)
        print(f"  [{i+1}/{args.count}] published")
        time.sleep(0.2)

    print("\nDone. If robot didn't move, check:")
    print(f"  1. ros2 topic info {topic} -v")
    print(f"  2. ros2 control list_controllers -c /{prefix}/controller_manager")
    print(f"  3. Driver terminal for error messages")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
