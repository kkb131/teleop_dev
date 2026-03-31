#!/usr/bin/env python3
"""Hand motion diagnostic: record open/fist poses and analyze joint angles.

Records 3s of "open hand" and 3s of "fist", then prints:
- Per-joint mean angles for each pose
- Flexion sign (positive or negative when curling)
- Angle range (how much each joint moves)
- Recommendations for visualizer tuning

Usage:
    cd /workspaces/tamp_ws/src/teleop_dev
    python3 -m operator.hand.tests.test_hand_motion
    python3 -m operator.hand.tests.test_hand_motion --hand left --duration 5
"""

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from operator.hand.manus_reader import (
    ManusReader, HandData,
    FINGER_NAMES, JOINT_NAMES_PER_FINGER,
    NUM_JOINTS, JOINTS_PER_FINGER,
    DEFAULT_SDK_BIN,
)


def collect_pose(reader, duration: float, hz: int = 60) -> np.ndarray:
    """Collect samples for `duration` seconds, return all samples as (N, 20)."""
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
    return np.array(samples)


def print_pose_table(label: str, samples: np.ndarray):
    """Print mean/min/max/std for each joint."""
    mean = samples.mean(axis=0)
    std = samples.std(axis=0)
    mn = samples.min(axis=0)
    mx = samples.max(axis=0)

    print(f"\n  {label} ({len(samples)} samples)")
    print(f"  {'Finger':8s} {'Joint':12s} {'Mean':>8s} {'Min':>8s} {'Max':>8s} {'Std':>8s}")
    print(f"  {'-' * 56}")

    for f_idx, fname in enumerate(FINGER_NAMES):
        jnames = JOINT_NAMES_PER_FINGER[fname]
        for j in range(JOINTS_PER_FINGER):
            idx = f_idx * JOINTS_PER_FINGER + j
            marker = ""
            if j > 0:  # flexion joints
                marker = " <-- flexion"
            print(f"  {fname:8s} {jnames[j]:12s} "
                  f"{mean[idx]:+8.4f} {mn[idx]:+8.4f} {mx[idx]:+8.4f} {std[idx]:8.4f}{marker}")


def main():
    parser = argparse.ArgumentParser(description="Hand motion diagnostic")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_BIN)
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Seconds per pose (default: 3)")
    args = parser.parse_args()

    print("=" * 65)
    print("  Hand Motion Diagnostic")
    print(f"  Hand: {args.hand}  |  Duration per pose: {args.duration}s")
    print("=" * 65)

    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
    except Exception as e:
        print(f"  [ERROR] {e}")
        sys.exit(1)

    time.sleep(0.5)

    # ── Pose 1: Open hand ────────────────────────────────
    print(f"\n  POSE 1: OPEN HAND")
    print(f"  Spread all fingers wide. Hold steady.")
    print(f"  Recording starts in 2 seconds...")
    time.sleep(2.0)
    print(f"  Recording {args.duration}s...")
    open_samples = collect_pose(reader, args.duration)

    if open_samples is None:
        print("  [ERROR] No data received")
        reader.disconnect()
        sys.exit(1)

    # ── Pose 2: Fist ─────────────────────────────────────
    print(f"\n  POSE 2: MAKE A FIST")
    print(f"  Curl all fingers tightly. Hold steady.")
    print(f"  Recording starts in 2 seconds...")
    time.sleep(2.0)
    print(f"  Recording {args.duration}s...")
    fist_samples = collect_pose(reader, args.duration)

    if fist_samples is None:
        print("  [ERROR] No data received")
        reader.disconnect()
        sys.exit(1)

    reader.disconnect()

    # ── Analysis ─────────────────────────────────────────
    print_pose_table("OPEN HAND", open_samples)
    print_pose_table("FIST", fist_samples)

    open_mean = open_samples.mean(axis=0)
    fist_mean = fist_samples.mean(axis=0)
    delta = fist_mean - open_mean

    print(f"\n  DELTA (fist - open):")
    print(f"  {'Finger':8s} {'Joint':12s} {'Open':>8s} {'Fist':>8s} {'Delta':>8s} {'Sign':>6s}")
    print(f"  {'-' * 56}")

    flexion_signs = []
    flexion_ranges = []

    for f_idx, fname in enumerate(FINGER_NAMES):
        jnames = JOINT_NAMES_PER_FINGER[fname]
        for j in range(JOINTS_PER_FINGER):
            idx = f_idx * JOINTS_PER_FINGER + j
            sign = "+" if delta[idx] > 0 else "-"
            is_flexion = j > 0

            marker = ""
            if is_flexion:
                marker = f"  {'CURL+' if delta[idx] > 0 else 'CURL-'}"
                flexion_signs.append(delta[idx] > 0)
                flexion_ranges.append(abs(delta[idx]))

            print(f"  {fname:8s} {jnames[j]:12s} "
                  f"{open_mean[idx]:+8.4f} {fist_mean[idx]:+8.4f} {delta[idx]:+8.4f} "
                  f"  {sign}{marker}")

    # ── Summary ──────────────────────────────────────────
    print(f"\n{'=' * 65}")
    print(f"  ANALYSIS SUMMARY")
    print(f"{'=' * 65}")

    # Flexion sign consistency
    pos_count = sum(flexion_signs)
    neg_count = len(flexion_signs) - pos_count
    print(f"\n  Flexion sign when curling:")
    print(f"    Positive (angle increases): {pos_count}/{len(flexion_signs)}")
    print(f"    Negative (angle decreases): {neg_count}/{len(flexion_signs)}")

    if pos_count > neg_count:
        print(f"    → Flexion is POSITIVE when curling")
        print(f"    → Visualizer should ADD flexion to angle (current: angle += flex * 0.55)")
    else:
        print(f"    → Flexion is NEGATIVE when curling")
        print(f"    → Visualizer should SUBTRACT flexion: angle -= flex * 0.55")

    # Range analysis
    avg_range = np.mean(flexion_ranges)
    max_range = np.max(flexion_ranges)
    print(f"\n  Flexion range (radians):")
    print(f"    Average: {avg_range:.4f} rad ({np.degrees(avg_range):.1f} deg)")
    print(f"    Max:     {max_range:.4f} rad ({np.degrees(max_range):.1f} deg)")

    # Damping recommendation
    # Visualizer uses angle += flex * 0.55
    # Full curl should be ~pi/2 (90 deg) visual rotation
    if avg_range > 0.01:
        ideal_damping = (np.pi / 2) / (avg_range * 3)  # 3 joints contribute
        print(f"\n  Damping coefficient recommendation:")
        print(f"    Current: 0.55")
        print(f"    Suggested: {ideal_damping:.2f} (for ~90° visual curl)")
        if ideal_damping > 1.5:
            print(f"    ⚠ Very high — raw angles may be too small (SDK degree→radian issue?)")
        elif ideal_damping < 0.2:
            print(f"    ⚠ Very low — raw angles may be too large")

    # Degree vs radian check
    print(f"\n  Degree vs Radian check:")
    print(f"    If values are ~0.5-2.0 rad → already in radians (expected)")
    print(f"    If values are ~30-90 → still in degrees (conversion missing)")
    print(f"    Observed avg flexion range: {avg_range:.4f}")
    if avg_range > 10:
        print(f"    ⚠ VALUES LOOK LIKE DEGREES — check deg2rad conversion!")

    print(f"\n{'=' * 65}")


if __name__ == "__main__":
    main()
