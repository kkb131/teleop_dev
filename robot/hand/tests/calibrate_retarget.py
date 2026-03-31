#!/usr/bin/env python3
"""2-pose calibration for Manus→DG5F retarget pipeline.

Runs while receiver.py + dg5f_driver (pid_all) are active.
Reads Manus data via UDP and DG5F feedback via /joint_states.

Procedure:
    1. User spreads hand fully → record Manus + DG5F positions
    2. User makes fist → record Manus + DG5F positions
    3. Compare DG5F actual vs desired → compute new calibration_factors
    4. Save to config/default.yaml

Requires (all running simultaneously):
    T1: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
    T2: python3 -m teleop_dev.robot.hand.receiver --hand right
    T3: (operator PC) python3 -m teleop_dev.operator.hand.manus_sender

Usage:
    python3 -m teleop_dev.robot.hand.tests.calibrate_retarget --hand right
"""

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from teleop_dev.robot.hand.retarget import (
    ManusToD5FRetarget, RIGHT_LIMITS, LEFT_LIMITS,
    DEFAULT_CALIBRATION_FACTORS,
)

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES = ["Spread", "MCP", "PIP", "DIP"]
NUM_JOINTS = 20


# ── DG5F target poses ────────────────────────────────────

def get_spread_pose(hand_side):
    """Open hand with fingers spread — spread joints at comfortable angle, flexion=0."""
    limits = RIGHT_LIMITS if hand_side == "right" else LEFT_LIMITS
    pose = np.zeros(20)
    for f in range(5):
        # Spread: use ~50% of range toward "spread" direction
        lim = limits[f * 4]
        # For right hand: negative spread = fingers apart for Index/Middle/Ring
        if hand_side == "right":
            pose[f * 4] = lim.min_rad * 0.5 if f in [1, 2, 3] else 0.0
        else:
            pose[f * 4] = lim.max_rad * 0.5 if f in [1, 2, 3] else 0.0
    return pose


def get_fist_pose(hand_side):
    """Tight fist — max flexion, spread=0."""
    limits = RIGHT_LIMITS if hand_side == "right" else LEFT_LIMITS
    pose = np.zeros(20)
    for f in range(5):
        pose[f * 4 + 0] = 0.0  # spread: neutral
        for j in range(1, 4):
            idx = f * 4 + j
            pose[idx] = limits[idx].max_rad * 0.8  # 80% of max for safety
    return pose


# ── ROS2 node for DG5F feedback ──────────────────────────

class CalibNode(Node):
    def __init__(self, hand: str):
        super().__init__("calibrate_retarget")
        prefix = f"dg5f_{hand}"
        self._js_latest = None
        self._sub = self.create_subscription(
            JointState, f"/{prefix}/joint_states", self._js_cb, 10)

    def _js_cb(self, msg):
        self._js_latest = msg

    def get_dg5f_positions(self) -> np.ndarray:
        if self._js_latest and len(self._js_latest.position) >= 20:
            return np.array(self._js_latest.position[:20])
        return None

    def spin_for(self, seconds):
        t0 = time.monotonic()
        while (time.monotonic() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)


# ── UDP Manus data reader ────────────────────────────────

class ManusUDPReader:
    """Read Manus data from the same UDP stream that receiver.py listens on.

    We sniff by binding to the same port with SO_REUSEADDR.
    Alternatively, this reads from the receiver's forwarded data.
    """
    def __init__(self, port=9872):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(1.0)

    def read(self) -> np.ndarray:
        try:
            raw, _ = self._sock.recvfrom(4096)
            pkt = json.loads(raw.decode())
            if pkt.get("type") == "manus":
                return np.array(pkt["joint_angles"], dtype=np.float32)
        except (socket.timeout, json.JSONDecodeError, KeyError):
            pass
        return None

    def close(self):
        self._sock.close()


def collect_samples(node, manus_reader, duration=3.0, hz=60):
    """Collect Manus + DG5F samples simultaneously."""
    dt = 1.0 / hz
    manus_samples = []
    dg5f_samples = []
    start = time.time()

    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0)

        m = manus_reader.read()
        d = node.get_dg5f_positions()

        if m is not None and d is not None:
            manus_samples.append(m.copy())
            dg5f_samples.append(d.copy())

        time.sleep(dt)

    if not manus_samples:
        return None, None
    return np.mean(manus_samples, axis=0), np.mean(dg5f_samples, axis=0)


def main():
    parser = argparse.ArgumentParser(description="2-pose retarget calibration")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--port", type=int, default=9872)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    config_path = Path(__file__).parent.parent / "config" / "default.yaml"

    print("=" * 65)
    print("  Manus → DG5F 2-Pose Calibration")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s per pose")
    print(f"  Config: {config_path}")
    print("=" * 65)
    print("\n  Prerequisites:")
    print("    T1: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py")
    print("    T2: python3 -m teleop_dev.robot.hand.receiver --hand right")
    print("    T3: (operator) python3 -m teleop_dev.operator.hand.manus_sender")

    rclpy.init()
    node = CalibNode(hand=args.hand)
    manus_reader = ManusUDPReader(port=args.port)

    retarget = ManusToD5FRetarget(hand_side=args.hand)
    limits = RIGHT_LIMITS if args.hand == "right" else LEFT_LIMITS

    # Wait for feedback
    print("\n  Waiting for DG5F joint_states...", end=" ", flush=True)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_dg5f_positions() is not None:
            break
    if node.get_dg5f_positions() is None:
        print("[FAIL]")
        sys.exit(1)
    print("[OK]")

    # Wait for Manus data
    print("  Waiting for Manus UDP data...", end=" ", flush=True)
    got_manus = False
    for _ in range(30):
        if manus_reader.read() is not None:
            got_manus = True
            break
    if not got_manus:
        print("[FAIL] — is manus_sender running?")
        sys.exit(1)
    print("[OK]")

    # ── Pose 1: Spread (open hand, fingers apart) ────────
    spread_target = get_spread_pose(args.hand)
    print(f"\n" + "-" * 65)
    print(f"  POSE 1: SPREAD (open hand, fingers wide apart)")
    print(f"  DG5F will go to spread position.")
    print(f"  Match this pose with your Manus glove hand.")
    input("  Press Enter when ready... ")

    print(f"  Recording {args.duration}s...")
    manus_spread, dg5f_spread = collect_samples(node, manus_reader, args.duration)
    if manus_spread is None:
        print("  [ERROR] No data!")
        sys.exit(1)
    retarget_spread = retarget.retarget(manus_spread)
    print(f"  Spread pose recorded. ({len(manus_spread)} joints)")

    # ── Pose 2: Fist ─────────────────────────────────────
    fist_target = get_fist_pose(args.hand)
    print(f"\n" + "-" * 65)
    print(f"  POSE 2: FIST (tight fist, thumb over fingers)")
    print(f"  DG5F will go to fist position.")
    print(f"  Match this pose with your Manus glove hand.")
    input("  Press Enter when ready... ")

    print(f"  Recording {args.duration}s...")
    manus_fist, dg5f_fist = collect_samples(node, manus_reader, args.duration)
    if manus_fist is None:
        print("  [ERROR] No data!")
        sys.exit(1)
    retarget_fist = retarget.retarget(manus_fist)
    print(f"  Fist pose recorded.")

    manus_reader.close()
    node.destroy_node()
    rclpy.shutdown()

    # ── Compute calibration ──────────────────────────────
    # Current retarget output range (with current calibration)
    current_range = retarget_fist - retarget_spread

    # Desired DG5F range
    desired_range = fist_target - spread_target

    # Actual DG5F range (what actually happened)
    actual_range = dg5f_fist - dg5f_spread

    # New factor: scale current factor by (desired / actual)
    old_cal = np.array(DEFAULT_CALIBRATION_FACTORS, dtype=np.float64)
    new_cal = old_cal.copy()

    for i in range(20):
        if abs(current_range[i]) > 0.01 and abs(actual_range[i]) > 0.01:
            # Ratio: how much should we scale to match desired?
            correction = desired_range[i] / current_range[i]
            new_cal[i] = old_cal[i] * correction
        # Clamp to reasonable range
        new_cal[i] = np.clip(new_cal[i], 0.1, 5.0)

    # ── Display ──────────────────────────────────────────
    print(f"\n" + "=" * 65)
    print(f"  Calibration Results")
    print(f"=" * 65)
    print(f"\n  {'Finger':7s} {'Joint':6s} {'Old':>6s} {'New':>6s} "
          f"{'ActualRange':>12s} {'DesiredRange':>13s} {'Error':>7s}")
    print(f"  {'-' * 60}")

    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            ar = math.degrees(actual_range[idx])
            dr = math.degrees(desired_range[idx])
            err = math.degrees(desired_range[idx] - actual_range[idx])
            changed = "*" if abs(new_cal[idx] - old_cal[idx]) > 0.1 else " "
            print(f"  {FINGER_NAMES[f]:7s} {JOINT_NAMES[j]:6s} "
                  f"{old_cal[idx]:6.2f} {new_cal[idx]:6.2f}{changed}"
                  f"{ar:+12.1f}° {dr:+13.1f}° {err:+7.1f}°")

    # ── Save to default.yaml ─────────────────────────────
    print(f"\n  Saving to {config_path}...")

    import yaml
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    cfg["retarget"]["calibration_factors"] = [round(float(v), 3) for v in new_cal]

    with open(config_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)

    print(f"  Saved! New calibration_factors:")
    for f_idx in range(5):
        vals = ", ".join(f"{new_cal[f_idx*4+j]:.3f}" for j in range(4))
        print(f"    {FINGER_NAMES[f_idx]:7s}: [{vals}]")

    print(f"\n  Restart receiver.py to apply the new calibration.")
    print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
