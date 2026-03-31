#!/usr/bin/env python3
"""2-pose calibration: compute retarget calibration_factors from Open + Fist.

Procedure:
    1. DG5F goes to open (all zeros) → user opens hand → record Manus
    2. DG5F goes to fist (max flexion) → user makes fist → record Manus
    3. Compute per-joint scale factor and save to JSON

Requires:
    - DG5F driver running: ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
    - Manus glove connected (ManusReader)

Usage:
    cd /workspaces/tamp_ws/src/teleop_dev
    python3 -m robot.hand.tests.calibrate_retarget --hand right
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
from operator.hand.manus_reader import ManusReader, NUM_JOINTS, DEFAULT_SDK_BIN
from robot.hand.retarget import ManusToD5FRetarget, RIGHT_LIMITS, LEFT_LIMITS, DEFAULT_CALIBRATION_FACTORS

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES = ["Spread", "MCP", "PIP", "DIP"]

# DG5F fist positions (max flexion, spread=0)
# Uses joint limit max for flexion joints, 0 for spread
def get_fist_pose(hand_side):
    limits = RIGHT_LIMITS if hand_side == "right" else LEFT_LIMITS
    fist = np.zeros(20)
    for f in range(5):
        # Spread: keep at 0
        fist[f * 4 + 0] = 0.0
        # Flexion joints: use max limit
        for j in range(1, 4):
            idx = f * 4 + j
            fist[idx] = limits[idx].max_rad * 0.8  # 80% of max for safety
    return fist


def collect_manus(reader, duration=3.0, hz=60):
    """Collect Manus samples and return mean."""
    dt = 1.0 / hz
    samples = []
    start = time.time()
    while time.time() - start < duration:
        data = reader.get_hand_data()
        if data is not None:
            samples.append(data.joint_angles.copy())
        time.sleep(dt)
    if not samples:
        return None
    return np.mean(samples, axis=0)


def main():
    parser = argparse.ArgumentParser(description="2-pose retarget calibration")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_BIN)
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Seconds per pose recording (default: 3)")
    parser.add_argument("--output", default=None,
                        help="Output JSON path (default: config/calibration_<hand>.json)")
    args = parser.parse_args()

    output_path = args.output or f"robot/hand/config/calibration_{args.hand}.json"

    print("=" * 65)
    print("  Manus → DG5F 2-Pose Calibration")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s per pose")
    print("=" * 65)

    # Connect to Manus
    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
    except Exception as e:
        print(f"  [ERROR] {e}")
        sys.exit(1)
    time.sleep(0.5)

    # Create retarget with calibration=1.0 (unit) to measure raw mapping
    unit_cal = [1.0] * 20
    retarget_unit = ManusToD5FRetarget(hand_side=args.hand, calibration_factors=unit_cal)
    limits = RIGHT_LIMITS if args.hand == "right" else LEFT_LIMITS

    # ── Pose 1: Open hand ────────────────────────────────
    print(f"\n" + "-" * 65)
    print(f"  POSE 1: OPEN HAND")
    print(f"  DG5F target: all zeros (open)")
    print(f"  Your hand: spread fingers wide, then close them together")
    print(f"  Press Enter when your hand is FLAT with fingers TOGETHER...")
    input("  > ")

    print(f"  Recording Manus ({args.duration}s)...")
    manus_open = collect_manus(reader, args.duration)
    if manus_open is None:
        print("  [ERROR] No Manus data!")
        reader.disconnect()
        sys.exit(1)
    print(f"  Open hand recorded.")

    # ── Pose 2: Fist ─────────────────────────────────────
    fist_pose = get_fist_pose(args.hand)
    print(f"\n" + "-" * 65)
    print(f"  POSE 2: FIST")
    print(f"  DG5F target: fist (80% max flexion)")
    print(f"  Your hand: make a TIGHT FIST")
    print(f"  Press Enter when your hand is a FIST...")
    input("  > ")

    print(f"  Recording Manus ({args.duration}s)...")
    manus_fist = collect_manus(reader, args.duration)
    if manus_fist is None:
        print("  [ERROR] No Manus data!")
        reader.disconnect()
        sys.exit(1)
    print(f"  Fist recorded.")

    reader.disconnect()

    # ── Compute calibration ──────────────────────────────
    # retarget with unit calibration
    dg5f_open_unit = retarget_unit.retarget(manus_open)
    dg5f_fist_unit = retarget_unit.retarget(manus_fist)
    manus_range = dg5f_fist_unit - dg5f_open_unit  # retarget output range with cal=1

    # DG5F desired range (open=0, fist=target)
    dg5f_open_target = np.zeros(20)
    dg5f_fist_target = fist_pose
    dg5f_range = dg5f_fist_target - dg5f_open_target  # desired DG5F range

    new_cal = np.ones(20)
    for i in range(20):
        if abs(manus_range[i]) > 0.01:  # avoid division by zero
            new_cal[i] = dg5f_range[i] / manus_range[i]
        else:
            new_cal[i] = DEFAULT_CALIBRATION_FACTORS[i]  # keep default

        # Clamp to reasonable range
        new_cal[i] = np.clip(new_cal[i], 0.1, 5.0)

    # ── Display results ──────────────────────────────────
    print(f"\n" + "=" * 65)
    print(f"  Calibration Results")
    print(f"=" * 65)
    print(f"\n  {'Finger':7s} {'Joint':6s} {'Old':>6s} {'New':>6s} "
          f"{'ManusRange':>11s} {'DG5FRange':>10s}")
    print(f"  {'-' * 52}")

    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            old = DEFAULT_CALIBRATION_FACTORS[idx]
            new = new_cal[idx]
            mr = math.degrees(manus_range[idx])
            dr = math.degrees(dg5f_range[idx])
            changed = "*" if abs(new - old) > 0.1 else " "
            print(f"  {FINGER_NAMES[f]:7s} {JOINT_NAMES[j]:6s} "
                  f"{old:6.2f} {new:6.2f}{changed}"
                  f"{mr:+11.1f}° {dr:+10.1f}°")

    # ── Save ─────────────────────────────────────────────
    cal_data = {
        "hand": args.hand,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "calibration_factors": new_cal.tolist(),
        "manus_open": manus_open.tolist(),
        "manus_fist": manus_fist.tolist(),
        "dg5f_fist_target": fist_pose.tolist(),
    }

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        json.dump(cal_data, f, indent=2)

    print(f"\n  Saved to: {out.resolve()}")

    # Print for copy-paste into config
    print(f"\n  Copy-paste for retarget config:")
    print(f"  calibration_factors:")
    for f in range(5):
        vals = ", ".join(f"{new_cal[f*4+j]:.2f}" for j in range(4))
        print(f"    # {FINGER_NAMES[f]:7s}: {vals}")

    print(f"\n  Or in tesollo_config.py format:")
    lines = []
    for f in range(5):
        vals = ", ".join(f"{new_cal[f*4+j]:.2f}" for j in range(4))
        lines.append(vals)
    print(f"    [{', '.join(lines)}]")

    print(f"\n{'=' * 65}")


if __name__ == "__main__":
    main()
