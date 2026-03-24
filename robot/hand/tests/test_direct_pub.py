#!/usr/bin/env python3
"""Minimal direct publish test for DG5F JointTrajectory topic.

Usage:
    python3 -m tesollo.tests.test_direct_pub
    python3 -m tesollo.tests.test_direct_pub --topic /dg5f_right/dg5f_right_controller/joint_trajectory
    python3 -m tesollo.tests.test_direct_pub --hand left --target 0.5
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RIGHT_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

LEFT_JOINTS = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]


def main():
    parser = argparse.ArgumentParser(description="Direct JointTrajectory publish test")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--topic", default=None, help="Override topic name")
    parser.add_argument("--target", type=float, default=0.0, help="Target position (rad)")
    parser.add_argument("--duration", type=float, default=2.0, help="Trajectory duration (sec)")
    parser.add_argument("--count", type=int, default=10, help="Number of publishes")
    args = parser.parse_args()

    prefix = f"dg5f_{args.hand}"
    topic = args.topic or f"/{prefix}/{prefix}_controller/joint_trajectory"
    joints = RIGHT_JOINTS if args.hand == "right" else LEFT_JOINTS

    rclpy.init()
    node = Node("direct_pub_test")

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(JointTrajectory, topic, qos)

    print(f"Topic:    {topic}")
    print(f"Joints:   {len(joints)}")
    print(f"Target:   {args.target} rad")
    print(f"Duration: {args.duration}s")
    print(f"Count:    {args.count}x")

    # Wait for DDS discovery
    print("\nWaiting 2s for DDS discovery...")
    time.sleep(2.0)

    # Check subscriber count
    sub_count = pub.get_subscription_count()
    print(f"Subscribers on topic: {sub_count}")
    if sub_count == 0:
        print("[WARN] No subscribers! The controller may not be listening.")
        print(f"  Check: ros2 topic info {topic}")
        print(f"  Check: ros2 control list_controllers -c /{prefix}/controller_manager")

    # Build message
    msg = JointTrajectory()
    msg.joint_names = joints

    point = JointTrajectoryPoint()
    point.positions = [args.target] * 20
    point.time_from_start = Duration(seconds=args.duration).to_msg()
    msg.points = [point]

    # Publish repeatedly
    print(f"\nPublishing {args.count} times...")
    for i in range(args.count):
        # Try with stamp
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        print(f"  [{i+1}/{args.count}] published (stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec})")
        time.sleep(0.2)

    # Also try with zero stamp (some controllers prefer this)
    print("\nPublishing 3x with zero stamp...")
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    for i in range(3):
        pub.publish(msg)
        print(f"  [{i+1}/3] published (stamp=0.0)")
        time.sleep(0.2)

    print("\nDone. If robot didn't move, check:")
    print(f"  1. ros2 topic info {topic} -v")
    print(f"  2. ros2 control list_controllers -c /{prefix}/controller_manager")
    print(f"  3. Driver terminal for error messages")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
