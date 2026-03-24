#!/usr/bin/env python3
"""Finger ROM (Range of Motion) calibration for Manus gloves.

Records min/max joint angles for each finger to create a per-user
calibration file. This calibration normalizes joint angles to [0, 1]
range for consistent mapping to robot hands (e.g., Tesollo DG 5F M).

Procedure:
    1. Fully open hand → record minimum angles
    2. Make a fist → record maximum angles
    3. Save calibration JSON

Usage:
    python3 -m manus.calibrate --sdk-path manus/sdk/SDKClient_Linux/SDKClient_Linux.out
    python3 -m manus.calibrate --output calibration_right.json --hand right
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from teleop_dev.operator.hand.manus_reader import (
    ManusReader, HandData,
    FINGER_NAMES, JOINT_NAMES_PER_FINGER,
    NUM_JOINTS, NUM_FINGERS, JOINTS_PER_FINGER,
)


def _collect_samples(reader: ManusReader, num_samples: int = 100,
                     hz: int = 60) -> np.ndarray:
    """Collect multiple samples and return mean joint angles.

    Returns ndarray[20] of mean angles, or None if no data.
    """
    dt = 1.0 / hz
    samples = []

    for i in range(num_samples):
        data = reader.get_hand_data()
        if data is not None:
            samples.append(data.joint_angles.copy())

        # Progress indicator
        pct = (i + 1) / num_samples * 100
        bar_len = 30
        filled = int(bar_len * (i + 1) / num_samples)
        bar = "#" * filled + "-" * (bar_len - filled)
        print(f"\r    [{bar}] {pct:5.1f}%  ({len(samples)} samples)", end="", flush=True)

        time.sleep(dt)

    print()

    if not samples:
        return None

    return np.mean(samples, axis=0)


def main():
    parser = argparse.ArgumentParser(
        description="Manus glove finger ROM calibration"
    )
    parser.add_argument("--sdk-path", default="manus/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right"],
                        help="Which hand to calibrate (default: right)")
    parser.add_argument("--output", default=None,
                        help="Output JSON file (default: calibration_<hand>.json)")
    parser.add_argument("--samples", type=int, default=100,
                        help="Samples per pose (default: 100)")
    args = parser.parse_args()

    output_path = args.output or f"manus/calibration_{args.hand}.json"

    print("=" * 60)
    print("  Manus Glove — Finger ROM Calibration")
    print(f"  Hand: {args.hand}  |  Samples: {args.samples}")
    print("=" * 60)

    # Connect
    print("\n  Connecting to Manus SDK...")
    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        sys.exit(1)

    time.sleep(0.5)

    # ── Step 1: Open hand (minimum angles) ────────────────
    print("\n" + "-" * 60)
    print("  STEP 1: OPEN HAND")
    print("  Spread all fingers as wide as possible.")
    print("  Hold steady and press Enter when ready...")
    input("  > ")

    print("  Recording open-hand pose...")
    min_angles = _collect_samples(reader, args.samples)

    if min_angles is None:
        print("  [ERROR] No data received. Check glove connection.")
        reader.disconnect()
        sys.exit(1)

    print("  Open-hand angles recorded.")

    # ── Step 2: Closed hand (maximum angles) ──────────────
    print("\n" + "-" * 60)
    print("  STEP 2: CLOSE HAND")
    print("  Make a tight fist (curl all fingers).")
    print("  Hold steady and press Enter when ready...")
    input("  > ")

    print("  Recording closed-hand pose...")
    max_angles = _collect_samples(reader, args.samples)

    if max_angles is None:
        print("  [ERROR] No data received. Check glove connection.")
        reader.disconnect()
        sys.exit(1)

    print("  Closed-hand angles recorded.")

    # Disconnect
    reader.disconnect()

    # ── Build calibration data ────────────────────────────
    # Ensure min < max for each joint
    for i in range(NUM_JOINTS):
        if min_angles[i] > max_angles[i]:
            min_angles[i], max_angles[i] = max_angles[i], min_angles[i]

    ranges = max_angles - min_angles

    calibration = {
        "hand": args.hand,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "samples_per_pose": args.samples,
        "joints": {},
    }

    print("\n" + "=" * 60)
    print("  Calibration Results")
    print("=" * 60)
    print(f"\n  {'Finger':8s} {'Joint':12s} {'Min':>8s} {'Max':>8s} {'Range':>8s}")
    print(f"  {'-' * 48}")

    for f_idx, fname in enumerate(FINGER_NAMES):
        jnames = JOINT_NAMES_PER_FINGER[fname]
        for j in range(JOINTS_PER_FINGER):
            idx = f_idx * JOINTS_PER_FINGER + j
            joint_key = f"{fname}_{jnames[j]}"
            calibration["joints"][joint_key] = {
                "index": idx,
                "min": float(min_angles[idx]),
                "max": float(max_angles[idx]),
                "range": float(ranges[idx]),
            }
            print(f"  {fname:8s} {jnames[j]:12s} "
                  f"{min_angles[idx]:+8.4f} {max_angles[idx]:+8.4f} {ranges[idx]:8.4f}")

    # ── Save ──────────────────────────────────────────────
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    with open(output, "w") as f:
        json.dump(calibration, f, indent=2)

    print(f"\n  Calibration saved to: {output.resolve()}")
    print(f"  Use with: --calibration-file {output}")
    print(f"\n  To apply in config/default.yaml:")
    print(f"    joint_mapping:")
    print(f"      calibration_file: \"{output}\"")
    print(f"{'=' * 60}")


def normalize_joints(joint_angles: np.ndarray,
                     calibration_file: str) -> np.ndarray:
    """Normalize raw joint angles to [0, 1] using calibration data.

    Parameters
    ----------
    joint_angles : ndarray[20]
        Raw joint angles from ManusReader.
    calibration_file : str
        Path to calibration JSON file.

    Returns
    -------
    ndarray[20]
        Normalized joint angles in [0, 1] range.
    """
    with open(calibration_file) as f:
        cal = json.load(f)

    normalized = np.zeros_like(joint_angles)

    for joint_key, joint_data in cal["joints"].items():
        idx = joint_data["index"]
        jmin = joint_data["min"]
        jmax = joint_data["max"]
        jrange = jmax - jmin

        if jrange > 1e-6:
            normalized[idx] = np.clip(
                (joint_angles[idx] - jmin) / jrange, 0.0, 1.0
            )
        else:
            normalized[idx] = 0.0

    return normalized


if __name__ == "__main__":
    main()
