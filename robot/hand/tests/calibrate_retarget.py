#!/usr/bin/env python3
"""2-pose calibration for Manus→DG5F retarget (observer mode).

Observes DG5F /joint_states ONLY — does NOT bind UDP or send commands.
Compares actual DG5F positions vs poses.json reference to compute
calibration_factors.

Requires (all running simultaneously):
    T1: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
    T2: python3 -m robot.hand.receiver --hand right
    T3: (operator PC) manus_sender

Usage:
    python3 -m robot.hand.tests.calibrate_retarget --hand right
"""

import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot.hand.retarget import DEFAULT_CALIBRATION_FACTORS
from robot.hand.tesollo_config import TesolloConfig

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_LABELS = ["Spread", "MCP", "PIP", "DIP"]

POSES_JSON = os.path.join(os.path.dirname(__file__), "..", "config", "poses.json")


def load_reference_poses(hand_side):
    """Load spread/fist reference poses from poses.json."""
    if not os.path.exists(POSES_JSON):
        print(f"  [ERROR] poses.json not found: {POSES_JSON}")
        print(f"  Record first: python3 -m robot.hand.tests.test_pose --hand {hand_side} --record spread")
        return None, None

    with open(POSES_JSON) as f:
        saved = json.load(f)

    spread_key = f"spread_{hand_side}"
    fist_key = f"fist_{hand_side}"

    for key in [spread_key, fist_key]:
        if key not in saved:
            print(f"  [ERROR] '{key}' not in poses.json")
            print(f"  Record: python3 -m robot.hand.tests.test_pose --hand {hand_side} --record {key.split('_')[0]}")
            return None, None

    return np.array(saved[spread_key]), np.array(saved[fist_key])


class CalibObserver(Node):
    """Observes DG5F joint_states only. No publishing."""

    def __init__(self, hand: str):
        super().__init__("calibrate_observer")
        prefix = f"dg5f_{hand}"
        self._js_latest = None
        self._js_names = None
        self._joints = ([f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
                        if hand == "right" else
                        [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)])
        self._sub = self.create_subscription(
            JointState, f"/{prefix}/joint_states", self._js_cb, 10)

    def _js_cb(self, msg):
        self._js_latest = msg
        if self._js_names is None:
            self._js_names = list(msg.name)

    def get_positions(self) -> np.ndarray:
        if self._js_latest is None or self._js_names is None:
            return None
        name_to_idx = {n: i for i, n in enumerate(self._js_names)}
        pos = []
        for jname in self._joints:
            if jname in name_to_idx:
                pos.append(float(self._js_latest.position[name_to_idx[jname]]))
            else:
                pos.append(0.0)
        return np.array(pos)


def collect_dg5f(node, duration=3.0, hz=60):
    """Collect DG5F joint_states samples only."""
    dt = 1.0 / hz
    samples = []
    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.01)
        d = node.get_positions()
        if d is not None:
            samples.append(d.copy())
        time.sleep(dt)
    if not samples:
        return None
    return np.mean(samples, axis=0)


def main():
    parser = argparse.ArgumentParser(description="2-pose retarget calibration (observer)")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--port", type=int, default=9872,
                        help="UDP port for reload trigger")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    config_path = Path(__file__).parent.parent / "config" / "default.yaml"

    print("=" * 65)
    print("  Manus → DG5F 2-Pose Calibration (Observer Mode)")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s per pose")
    print("=" * 65)
    print("\n  This script OBSERVES /joint_states only.")
    print("  receiver.py keeps controlling DG5F — no interference.")
    print("\n  Prerequisites (must be running):")
    print("    T1: dg5f_driver (pid_all_controller)")
    print("    T2: robot.hand.receiver --hand right")
    print("    T3: (operator) manus_sender")

    # Load reference poses
    ref_spread, ref_fist = load_reference_poses(args.hand)
    if ref_spread is None or ref_fist is None:
        sys.exit(1)

    print(f"\n  Reference from poses.json:")
    print(f"    Spread: [{', '.join(f'{v:.2f}' for v in ref_spread[:4])}...]")
    print(f"    Fist:   [{', '.join(f'{v:.2f}' for v in ref_fist[:4])}...]")

    # Load current calibration
    cfg = TesolloConfig.load(str(config_path))
    old_cal = np.array(cfg.retarget.calibration_factors, dtype=np.float64)

    rclpy.init()
    node = CalibObserver(hand=args.hand)

    # Wait for joint_states
    print("\n  Waiting for DG5F joint_states...", end=" ", flush=True)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_positions() is not None:
            break
    if node.get_positions() is None:
        print("[FAIL]")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("[OK]")

    # ── Pose 1: Spread ───────────────────────────────────
    print(f"\n" + "-" * 65)
    print(f"  POSE 1: SPREAD")
    print(f"  Open your Manus glove hand wide (fingers spread apart).")
    print(f"  DG5F should follow via receiver.py.")
    input("  Press Enter when holding SPREAD... ")

    print(f"  Recording DG5F positions ({args.duration}s)...")
    dg5f_spread = collect_dg5f(node, args.duration)
    if dg5f_spread is None:
        print("  [ERROR] No joint_states data!")
        sys.exit(1)
    print(f"  Spread recorded.")

    # ── Pose 2: Fist ─────────────────────────────────────
    print(f"\n" + "-" * 65)
    print(f"  POSE 2: FIST")
    print(f"  Make a tight fist with your Manus glove hand.")
    print(f"  DG5F should follow via receiver.py.")
    input("  Press Enter when holding FIST... ")

    print(f"  Recording DG5F positions ({args.duration}s)...")
    dg5f_fist = collect_dg5f(node, args.duration)
    if dg5f_fist is None:
        print("  [ERROR] No joint_states data!")
        sys.exit(1)
    print(f"  Fist recorded.")

    node.destroy_node()
    rclpy.shutdown()

    # ── Compute calibration ──────────────────────────────
    actual_range = dg5f_fist - dg5f_spread      # what DG5F actually did
    desired_range = ref_fist - ref_spread         # what poses.json says

    new_cal = old_cal.copy()
    for i in range(20):
        if abs(actual_range[i]) > 0.01:
            new_cal[i] = old_cal[i] * (desired_range[i] / actual_range[i])
        new_cal[i] = np.clip(new_cal[i], 0.1, 5.0)

    # ── Display ──────────────────────────────────────────
    print(f"\n" + "=" * 65)
    print(f"  Calibration Results")
    print(f"=" * 65)
    print(f"\n  {'Finger':7s} {'Joint':6s} {'Old':>6s} {'New':>6s} "
          f"{'Actual':>8s} {'Desired':>8s} {'Error':>8s}")
    print(f"  {'-' * 55}")

    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            ar = math.degrees(actual_range[idx])
            dr = math.degrees(desired_range[idx])
            err = dr - ar
            changed = "*" if abs(new_cal[idx] - old_cal[idx]) > 0.1 else " "
            print(f"  {FINGER_NAMES[f]:7s} {JOINT_LABELS[j]:6s} "
                  f"{old_cal[idx]:6.2f} {new_cal[idx]:6.2f}{changed}"
                  f"{ar:+8.1f}° {dr:+8.1f}° {err:+8.1f}°")

    # ── Save to default.yaml ─────────────────────────────
    print(f"\n  Saving to {config_path}...")

    import yaml
    with open(config_path) as f:
        raw_yaml = yaml.safe_load(f)

    raw_yaml["retarget"]["calibration_factors"] = [round(float(v), 3) for v in new_cal]

    with open(config_path, "w") as f:
        yaml.dump(raw_yaml, f, default_flow_style=False, sort_keys=False)

    print(f"  Saved!")
    for f_idx in range(5):
        vals = ", ".join(f"{new_cal[f_idx*4+j]:.3f}" for j in range(4))
        print(f"    {FINGER_NAMES[f_idx]:7s}: [{vals}]")

    # Reload trigger
    print(f"\n  Sending reload trigger to receiver...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(json.dumps({"type": "reload_config"}).encode(),
                 ("127.0.0.1", args.port))
        s.close()
        print(f"  Receiver will apply new calibration automatically.")
    except Exception as e:
        print(f"  [WARN] Reload trigger failed: {e}")
        print(f"  Restart receiver.py manually to apply.")

    print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
