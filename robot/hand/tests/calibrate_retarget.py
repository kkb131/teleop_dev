#!/usr/bin/env python3
"""2-pose calibration for Manus→DG5F retarget (observer mode).

Runs alongside receiver.py — does NOT send commands to DG5F.
Observes Manus UDP + DG5F /joint_states, computes calibration factors
so that spread→poses.json spread, fist→poses.json fist.

Requires (all running simultaneously):
    T1: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
    T2: python3 -m robot.hand.receiver --hand right
    T3: (operator PC) python3 -m sender.hand.manus_sender

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

from robot.hand.retarget import (
    ManusToD5FRetarget, RIGHT_LIMITS, LEFT_LIMITS,
    DEFAULT_CALIBRATION_FACTORS,
)
from robot.hand.tesollo_config import TesolloConfig

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_LABELS = ["Spread", "MCP", "PIP", "DIP"]

# Load poses from test_pose.py's poses.json
POSES_JSON = os.path.join(os.path.dirname(__file__), "..", "config", "poses.json")


def load_reference_poses(hand_side):
    """Load spread/fist reference poses from poses.json."""
    if not os.path.exists(POSES_JSON):
        print(f"  [ERROR] poses.json not found: {POSES_JSON}")
        print(f"  Record poses first: python3 -m robot.hand.tests.test_pose --hand {hand_side} --record spread")
        return None, None

    with open(POSES_JSON) as f:
        saved = json.load(f)

    spread_key = f"spread_{hand_side}"
    fist_key = f"fist_{hand_side}"

    if spread_key not in saved:
        print(f"  [ERROR] '{spread_key}' not in poses.json")
        print(f"  Record: python3 -m robot.hand.tests.test_pose --hand {hand_side} --record spread")
        return None, None
    if fist_key not in saved:
        print(f"  [ERROR] '{fist_key}' not in poses.json")
        print(f"  Record: python3 -m robot.hand.tests.test_pose --hand {hand_side} --record fist")
        return None, None

    return np.array(saved[spread_key]), np.array(saved[fist_key])


class CalibObserver(Node):
    """Observes DG5F joint_states without publishing commands."""

    def __init__(self, hand: str):
        super().__init__("calibrate_observer")
        prefix = f"dg5f_{hand}"
        self._hand = hand
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

    def get_dg5f_positions(self) -> np.ndarray:
        """Return 20 joint positions in canonical order."""
        if self._js_latest is None or self._js_names is None:
            return None
        # Reorder from joint_states name order → canonical order
        name_to_idx = {n: i for i, n in enumerate(self._js_names)}
        pos = []
        for jname in self._joints:
            if jname in name_to_idx:
                pos.append(float(self._js_latest.position[name_to_idx[jname]]))
            else:
                pos.append(0.0)
        return np.array(pos)

    def spin_for(self, seconds):
        t0 = time.monotonic()
        while (time.monotonic() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)


class ManusUDPSniff:
    """Sniff Manus UDP packets (SO_REUSEPORT alongside receiver.py)."""

    def __init__(self, port=9872):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
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


def collect_pose(node, manus_sniff, duration=3.0, hz=60):
    """Collect Manus + DG5F joint_states simultaneously. No commands sent."""
    dt = 1.0 / hz
    manus_samples = []
    dg5f_samples = []
    start = time.time()

    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0)
        m = manus_sniff.read()
        d = node.get_dg5f_positions()
        if m is not None and d is not None:
            manus_samples.append(m.copy())
            dg5f_samples.append(d.copy())
        time.sleep(dt)

    if not manus_samples:
        return None, None
    return np.mean(manus_samples, axis=0), np.mean(dg5f_samples, axis=0)


def main():
    parser = argparse.ArgumentParser(description="2-pose retarget calibration (observer)")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--port", type=int, default=9872)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    config_path = Path(__file__).parent.parent / "config" / "default.yaml"

    print("=" * 65)
    print("  Manus → DG5F 2-Pose Calibration (Observer Mode)")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s per pose")
    print("=" * 65)
    print("\n  This script OBSERVES only — receiver.py controls DG5F.")
    print("  Prerequisites (must be running):")
    print("    T1: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py")
    print("    T2: python3 -m robot.hand.receiver --hand right")
    print("    T3: (operator) manus_sender")

    # Load reference poses from poses.json
    ref_spread, ref_fist = load_reference_poses(args.hand)
    if ref_spread is None or ref_fist is None:
        sys.exit(1)

    print(f"\n  Reference poses loaded from poses.json:")
    print(f"    Spread: {ref_spread[:4]}... (first 4 joints)")
    print(f"    Fist:   {ref_fist[:4]}... (first 4 joints)")

    # Load current calibration
    cfg = TesolloConfig.load(str(config_path))
    old_cal = np.array(cfg.retarget.calibration_factors, dtype=np.float64)
    retarget = ManusToD5FRetarget(
        hand_side=args.hand, calibration_factors=cfg.retarget.calibration_factors)

    rclpy.init()
    node = CalibObserver(hand=args.hand)
    manus_sniff = ManusUDPSniff(port=args.port)

    # Wait for data
    print("\n  Waiting for DG5F joint_states...", end=" ", flush=True)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_dg5f_positions() is not None:
            break
    if node.get_dg5f_positions() is None:
        print("[FAIL]")
        sys.exit(1)
    print("[OK]")

    print("  Waiting for Manus UDP...", end=" ", flush=True)
    got = False
    for _ in range(30):
        if manus_sniff.read() is not None:
            got = True
            break
    if not got:
        print("[FAIL] — is manus_sender running?")
        sys.exit(1)
    print("[OK]")

    # ── Pose 1: Spread ───────────────────────────────────
    print(f"\n" + "-" * 65)
    print(f"  POSE 1: SPREAD")
    print(f"  Open your hand wide (fingers spread apart).")
    print(f"  The DG5F should also be in spread position (via receiver).")
    input("  Press Enter when holding the SPREAD pose... ")

    print(f"  Recording {args.duration}s (observing only)...")
    manus_spread, dg5f_spread = collect_pose(node, manus_sniff, args.duration)
    if manus_spread is None:
        print("  [ERROR] No data!")
        sys.exit(1)
    retarget_spread = retarget.retarget(manus_spread)
    print(f"  Spread recorded: {len(manus_spread)} joints")

    # ── Pose 2: Fist ─────────────────────────────────────
    print(f"\n" + "-" * 65)
    print(f"  POSE 2: FIST")
    print(f"  Make a tight fist (thumb over fingers).")
    print(f"  The DG5F should also be in fist position (via receiver).")
    input("  Press Enter when holding the FIST pose... ")

    print(f"  Recording {args.duration}s (observing only)...")
    manus_fist, dg5f_fist = collect_pose(node, manus_sniff, args.duration)
    if manus_fist is None:
        print("  [ERROR] No data!")
        sys.exit(1)
    retarget_fist = retarget.retarget(manus_fist)
    print(f"  Fist recorded.")

    manus_sniff.close()
    node.destroy_node()
    rclpy.shutdown()

    # ── Compute new calibration ──────────────────────────
    # What retarget currently produces (with current cal)
    current_range = retarget_fist - retarget_spread

    # What DG5F actually did
    actual_range = dg5f_fist - dg5f_spread

    # What poses.json says it SHOULD do
    desired_range = ref_fist - ref_spread

    # Correction: scale old_cal so that current_range → desired_range
    new_cal = old_cal.copy()
    for i in range(20):
        if abs(current_range[i]) > 0.01:
            correction = desired_range[i] / current_range[i]
            new_cal[i] = old_cal[i] * correction
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

    # Send reload trigger to receiver
    print(f"\n  Sending reload trigger to receiver (port {args.port})...")
    trigger_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    trigger_sock.sendto(
        json.dumps({"type": "reload_config"}).encode(),
        ("127.0.0.1", args.port))
    trigger_sock.close()
    print(f"  Receiver will apply new calibration automatically.")
    print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
