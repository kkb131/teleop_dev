#!/usr/bin/env python3
"""Step 3: Continuous data streaming test.

Reads glove data continuously for a configurable duration and
reports statistics (data rate, joint angle ranges, dropped frames).

Requirements:
    - Step 2 passed (glove connected)
    - Gloves worn on hand(s)

Usage: python3 -m sender.hand.tests.test_step3_stream [--duration 5] [--hz 60]
"""

import argparse
import sys
import time

import numpy as np

from sender.hand.manus_reader import (
    ManusReader, HandData,
    FINGER_NAMES, JOINT_NAMES_PER_FINGER,
    NUM_JOINTS, NUM_FINGERS, JOINTS_PER_FINGER,
)


def main():
    parser = argparse.ArgumentParser(description="Step 3: Streaming test")
    parser.add_argument("--sdk-path", default="sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right"],
                        help="Which hand to test (default: right)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Test duration in seconds (default: 5)")
    parser.add_argument("--hz", type=int, default=60,
                        help="Read rate in Hz (default: 60)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Step 3: Continuous Streaming Test")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s  |  Rate: {args.hz} Hz")
    print("=" * 60)
    passed = 0
    failed = 0

    # Connect
    print("\n[TEST] Connect...", end=" ")
    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # Give SDK time to start data stream
    time.sleep(0.5)

    # Collect data
    print(f"\n[TEST] Streaming {args.duration}s at {args.hz} Hz...")
    print("       Move your fingers to exercise full range of motion!")
    print()

    dt = 1.0 / args.hz
    total_frames = int(args.duration * args.hz)
    success_count = 0
    null_count = 0
    all_angles = []
    timestamps = []

    start_time = time.perf_counter()

    for i in range(total_frames):
        t_loop = time.perf_counter()

        data = reader.get_hand_data()
        if data is not None:
            success_count += 1
            all_angles.append(data.joint_angles.copy())
            timestamps.append(data.timestamp)

            # Live display every 10 frames
            if i % 10 == 0:
                _print_live(data, i, total_frames)
        else:
            null_count += 1

        elapsed = time.perf_counter() - t_loop
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)

    actual_duration = time.perf_counter() - start_time

    # Clear live display
    print(f"\033[{NUM_FINGERS + 3}B")

    # Disconnect
    reader.disconnect()

    # ── Analysis ──────────────────────────────────────────

    print("\n" + "=" * 60)
    print("  Streaming Results")
    print("=" * 60)

    # Test: Data rate
    print(f"\n[TEST] Data rate...", end=" ")
    actual_hz = success_count / actual_duration if actual_duration > 0 else 0
    if success_count > 0:
        print(f"[PASS] {actual_hz:.1f} Hz ({success_count}/{total_frames} frames)")
        passed += 1
    else:
        print(f"[FAIL] No data received")
        failed += 1

    # Test: Data availability
    print(f"[TEST] Data availability...", end=" ")
    availability = success_count / total_frames * 100 if total_frames > 0 else 0
    if availability > 80:
        print(f"[PASS] {availability:.1f}% ({null_count} null frames)")
        passed += 1
    elif availability > 50:
        print(f"[WARN] {availability:.1f}% — some data loss")
        passed += 1
    else:
        print(f"[FAIL] {availability:.1f}% — too many null frames")
        failed += 1

    if len(all_angles) == 0:
        print("\n[FAIL] No data to analyze")
        failed += 1
        _summary(passed, failed)
        return

    angles_array = np.array(all_angles)  # shape: (N, 20)

    # Test: Joint angle ranges
    print(f"\n[TEST] Joint angle ranges...", end=" ")
    mins = angles_array.min(axis=0)
    maxs = angles_array.max(axis=0)
    ranges = maxs - mins

    # Check if at least some joints show variation
    active_joints = (ranges > 0.01).sum()
    if active_joints > 5:
        print(f"[PASS] {active_joints}/{NUM_JOINTS} joints show variation")
        passed += 1
    elif active_joints > 0:
        print(f"[WARN] Only {active_joints}/{NUM_JOINTS} joints active")
        print("       Move fingers more during test")
        passed += 1
    else:
        print(f"[WARN] No joint variation detected")
        print("       Glove may not be properly worn")
        passed += 1

    # Print per-finger statistics
    print(f"\n  Per-finger joint ranges (min / max / range):")
    print(f"  {'Finger':8s} {'Joint':12s} {'Min':>8s} {'Max':>8s} {'Range':>8s}")
    print(f"  {'-' * 48}")

    for f_idx, fname in enumerate(FINGER_NAMES):
        jnames = JOINT_NAMES_PER_FINGER[fname]
        for j in range(JOINTS_PER_FINGER):
            idx = f_idx * JOINTS_PER_FINGER + j
            print(f"  {fname:8s} {jnames[j]:12s} "
                  f"{mins[idx]:+8.3f} {maxs[idx]:+8.3f} {ranges[idx]:8.3f}")

    # Test: Timestamp consistency
    print(f"\n[TEST] Timestamp consistency...", end=" ")
    if len(timestamps) > 1:
        ts = np.array(timestamps)
        dts = np.diff(ts)
        mean_dt = dts.mean()
        std_dt = dts.std()
        max_gap = dts.max()
        print(f"[PASS]")
        print(f"       Mean interval: {mean_dt * 1000:.1f} ms")
        print(f"       Std deviation: {std_dt * 1000:.1f} ms")
        print(f"       Max gap:       {max_gap * 1000:.1f} ms")
        passed += 1
    else:
        print("[SKIP] Not enough data points")

    # Summary statistics
    print(f"\n  Summary:")
    print(f"    Total frames:   {total_frames}")
    print(f"    Received:       {success_count}")
    print(f"    Null:           {null_count}")
    print(f"    Duration:       {actual_duration:.2f}s")
    print(f"    Effective rate: {actual_hz:.1f} Hz")

    _summary(passed, failed)


def _print_live(data: HandData, frame: int, total: int):
    """Print live joint angles (overwrites previous output)."""
    if frame > 0:
        # Move cursor up to overwrite
        print(f"\033[{NUM_FINGERS + 2}A", end="")

    pct = frame / total * 100 if total > 0 else 0
    print(f"  [{frame:5d}/{total}] ({pct:5.1f}%)")

    for f_idx, fname in enumerate(FINGER_NAMES):
        joints = data.joint_angles[f_idx * JOINTS_PER_FINGER:
                                   (f_idx + 1) * JOINTS_PER_FINGER]
        bar = " ".join(f"{v:+6.3f}" for v in joints)
        print(f"  {fname:7s}: {bar}")

    print()


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 3 complete — proceed to Step 4")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 60}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
