#!/usr/bin/env python3
"""DG5F JTC controller tuning test suite.

Tests zero-position accuracy, oscillation, step response, and joint mapping.
Outputs per-joint diagnostics and tuning recommendations.

Requires dg5f_driver running:
    ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72

Usage:
    python3 -m teleop_dev.robot.hand.tests.test_tuning --hand right
    python3 -m teleop_dev.robot.hand.tests.test_tuning --hand right --test zero
    python3 -m teleop_dev.robot.hand.tests.test_tuning --hand right --test oscillation
    python3 -m teleop_dev.robot.hand.tests.test_tuning --hand right --test step
    python3 -m teleop_dev.robot.hand.tests.test_tuning --hand right --test mapping
"""

import argparse
import math
import time
from collections import defaultdict

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

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
DEG = 180.0 / math.pi


class TuningNode(Node):
    def __init__(self, hand: str):
        super().__init__("dg5f_tuning_test")
        self._hand = hand
        prefix = f"dg5f_{hand}"
        self._joints = RIGHT_JOINTS if hand == "right" else LEFT_JOINTS

        # Publisher
        traj_topic = f"/{prefix}/{prefix}_controller/joint_trajectory"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(JointTrajectory, traj_topic, qos)

        # Subscriber
        js_topic = f"/{prefix}/joint_states"
        self._js_history = []
        self._js_latest = None
        self._recording = False
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

    def _js_cb(self, msg: JointState):
        self._js_latest = msg
        if self._recording:
            self._js_history.append((time.monotonic(), list(msg.position)))

    def send_positions(self, positions, duration_s=0.0):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self._joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()
        msg.points = [point]
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)
            time.sleep(0.05)

    def spin_for(self, seconds: float):
        t0 = time.monotonic()
        while (time.monotonic() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_feedback(self, timeout=5.0):
        self.get_logger().info("Waiting for joint_states...")
        t0 = time.monotonic()
        while self._js_latest is None and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._js_latest is None:
            self.get_logger().error("No joint_states received!")
            return False
        return True

    def get_positions(self) -> np.ndarray:
        if self._js_latest and len(self._js_latest.position) >= 20:
            return np.array(self._js_latest.position[:20])
        return np.zeros(20)

    def start_recording(self):
        self._js_history = []
        self._recording = True

    def stop_recording(self):
        self._recording = False
        return self._js_history


# ─────────────────────────────────────────────────────────
# Test 1: Zero Position Accuracy
# ─────────────────────────────────────────────────────────

def test_zero_accuracy(node: TuningNode):
    print("\n" + "=" * 60)
    print("  Test 1: Zero Position Accuracy")
    print("=" * 60)

    node.send_positions([0.0] * 20, duration_s=2.0)
    print("  Sent 0-degree command, waiting 3s to settle...")
    node.spin_for(3.0)

    # Sample 20 readings
    samples = []
    for _ in range(20):
        node.spin_for(0.05)
        samples.append(node.get_positions().copy())
    samples = np.array(samples)
    mean_pos = samples.mean(axis=0)

    warn_count = 0
    threshold_rad = 0.02  # ~1 degree

    for i, name in enumerate(node._joints):
        err = mean_pos[i]
        tag = "[WARN: >1deg]" if abs(err) > threshold_rad else "[OK]"
        if abs(err) > threshold_rad:
            warn_count += 1
        print(f"  {name}: {err:+.4f} rad ({err * DEG:+.1f} deg)  {tag}")

    print(f"\n  Summary: {warn_count}/20 joints exceed 1-degree threshold")
    if warn_count > 0:
        print("  >> Recommendation: Add I gain (start: i=0.05, i_clamp_max=0.5)")
    return mean_pos


# ─────────────────────────────────────────────────────────
# Test 2: Oscillation Detection
# ─────────────────────────────────────────────────────────

def test_oscillation(node: TuningNode):
    print("\n" + "=" * 60)
    print("  Test 2: Oscillation Detection")
    print("=" * 60)

    osc_joints = []

    for finger_idx in range(5):
        finger_name = FINGER_NAMES[finger_idx]
        test_joints = [finger_idx * 4 + 2, finger_idx * 4 + 3]  # PIP, DIP

        # Move to 80 degrees
        target = [0.0] * 20
        for j in test_joints:
            target[j] = math.radians(80)
        node.send_positions(target, duration_s=1.0)
        node.spin_for(1.5)

        # Step back to 0 and record
        node.start_recording()
        node.send_positions([0.0] * 20, duration_s=0.0)
        node.spin_for(2.0)
        history = node.stop_recording()

        if len(history) < 10:
            print(f"  {finger_name}: insufficient data ({len(history)} samples)")
            continue

        # Analyze last 1 second
        positions = np.array([h[1] for h in history])
        times = np.array([h[0] for h in history])
        cutoff = times[-1] - 1.0
        mask = times >= cutoff
        tail = positions[mask]

        for j in test_joints:
            joint_name = node._joints[j]
            std = np.std(tail[:, j])
            tag = "[OSCILLATING]" if std > 0.01 else "[STABLE]"
            if std > 0.01:
                osc_joints.append(joint_name)
            print(f"  {finger_name} ({joint_name}): std={std:.4f} rad  {tag}")

    if osc_joints:
        print(f"\n  >> Oscillating joints: {', '.join(osc_joints)}")
        print("  >> Recommendation: Reduce P or add D gain (start: d=0.1)")
    else:
        print("\n  All joints stable.")


# ─────────────────────────────────────────────────────────
# Test 3: Step Response
# ─────────────────────────────────────────────────────────

def test_step_response(node: TuningNode):
    print("\n" + "=" * 60)
    print("  Test 3: Step Response (per finger)")
    print("=" * 60)

    # Go to zero first
    node.send_positions([0.0] * 20, duration_s=1.0)
    node.spin_for(2.0)

    target_rad = math.radians(80)

    for finger_idx in range(5):
        finger_name = FINGER_NAMES[finger_idx]
        j = finger_idx * 4 + 2  # PIP joint

        # Step to 80 degrees
        target = [0.0] * 20
        target[j] = target_rad

        node.start_recording()
        node.send_positions(target, duration_s=0.0)
        node.spin_for(2.0)
        history = node.stop_recording()

        if len(history) < 10:
            print(f"  {finger_name}: insufficient data")
            continue

        positions = np.array([h[1][:20] for h in history])
        times = np.array([h[0] for h in history])
        times -= times[0]  # relative time

        joint_pos = positions[:, j]
        threshold_90 = target_rad * 0.9

        # 90% settling time
        settle_idx = np.where(joint_pos >= threshold_90)[0]
        settle_time = times[settle_idx[0]] if len(settle_idx) > 0 else float('inf')

        # Overshoot
        peak = np.max(joint_pos)
        overshoot_pct = max(0, (peak - target_rad) / target_rad * 100)

        tags = []
        if settle_time > 1.0:
            tags.append("SLOW")
        if overshoot_pct > 10:
            tags.append("OVERSHOOT")
        tag_str = f"  [WARN: {', '.join(tags)}]" if tags else "  [OK]"

        print(f"  {finger_name} ({node._joints[j]}): "
              f"90% settle={settle_time:.2f}s, overshoot={overshoot_pct:.1f}%{tag_str}")

        # Return to zero
        node.send_positions([0.0] * 20, duration_s=0.5)
        node.spin_for(1.0)

    print("\n  >> If SLOW: Increase P or ff_velocity_scale")
    print("  >> If OVERSHOOT: Decrease P")


# ─────────────────────────────────────────────────────────
# Test 4: Joint Mapping Verification
# ─────────────────────────────────────────────────────────

def test_mapping(node: TuningNode):
    print("\n" + "=" * 60)
    print("  Test 4: Joint Mapping Verification")
    print("=" * 60)

    # Record baseline at zero
    node.send_positions([0.0] * 20, duration_s=1.0)
    node.spin_for(2.0)
    baseline = node.get_positions()

    target_rad = math.radians(30)
    crosstalk_threshold = math.radians(5)
    issues = []

    for i, name in enumerate(node._joints):
        # Command single joint
        target = [0.0] * 20
        target[i] = target_rad
        node.send_positions(target, duration_s=0.5)
        node.spin_for(1.0)

        current = node.get_positions()
        actual_move = current[i] - baseline[i]
        accuracy = abs(actual_move - target_rad)

        # Check crosstalk
        crosstalk = []
        for j in range(20):
            if j != i:
                delta = abs(current[j] - baseline[j])
                if delta > crosstalk_threshold:
                    crosstalk.append((node._joints[j], delta))

        acc_tag = "[OK]" if accuracy < math.radians(5) else "[WARN: inaccurate]"
        print(f"  {name}: cmd={target_rad:.3f} actual={actual_move:.3f} err={accuracy:.3f}  {acc_tag}")

        if accuracy >= math.radians(5):
            issues.append(f"{name}: accuracy {accuracy * DEG:.1f} deg")

        for ct_name, ct_val in crosstalk:
            print(f"    crosstalk on {ct_name} = {ct_val:.3f} rad ({ct_val * DEG:.1f} deg)  [WARN]")
            issues.append(f"{name} → {ct_name}: {ct_val * DEG:.1f} deg crosstalk")

        # Return to zero
        node.send_positions([0.0] * 20, duration_s=0.3)
        node.spin_for(0.5)

    if issues:
        print(f"\n  Issues found: {len(issues)}")
        for issue in issues:
            print(f"    - {issue}")
    else:
        print("\n  All joints mapped correctly, no crosstalk detected.")


# ─────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="DG5F JTC tuning test suite")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--test", default="all",
                        choices=["all", "zero", "oscillation", "step", "mapping"],
                        help="Run specific test or all")
    args = parser.parse_args()

    rclpy.init()
    node = TuningNode(hand=args.hand)

    if not node.wait_for_feedback():
        node.destroy_node()
        rclpy.shutdown()
        return

    print("\n" + "=" * 60)
    print(f"  DG5F JTC Tuning Test Suite ({args.hand} hand)")
    print(f"  Current config: P=1.5, I=0, D=0, ff=0.0001")
    print("=" * 60)

    try:
        tests = {
            "zero": test_zero_accuracy,
            "oscillation": test_oscillation,
            "step": test_step_response,
            "mapping": test_mapping,
        }

        if args.test == "all":
            for name, test_fn in tests.items():
                test_fn(node)
        else:
            tests[args.test](node)

        # Final: return to zero
        print("\n  Returning to home position...")
        node.send_positions([0.0] * 20, duration_s=1.0)
        node.spin_for(2.0)

        print("\n" + "=" * 60)
        print("  Tuning Parameter Reference")
        print("=" * 60)
        print("  Config: dg5f_driver/config/dg5f_right_controller.yaml")
        print("  Parameters per joint (gains.{joint_name}):")
        print("    p: proportional (higher=faster, risk oscillation)")
        print("    i: integral (eliminates steady-state error)")
        print("    d: derivative (dampens oscillation)")
        print("    i_clamp_max/min: limits integral windup")
        print("    ff_velocity_scale: velocity feedforward")
        print("  After editing, restart the driver to apply.")

    except KeyboardInterrupt:
        print("\n  Interrupted. Sending zero...")
        node.send_positions([0.0] * 20, duration_s=1.0)
        node.spin_for(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  Done.")


if __name__ == "__main__":
    main()
