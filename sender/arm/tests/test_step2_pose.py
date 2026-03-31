#!/usr/bin/env python3
"""Step 2: Tracker pose reading validation.

Reads pose data for a duration and validates:
- Valid pose percentage
- Position range (sanity check)
- Quaternion normalization
- Actual read frequency

Requirements: SteamVR running, tracker paired and tracked
Usage: python3 -m sender.arm.tests.test_step2_pose [--duration 3] [--serial LHR-xxx]
"""

import argparse
import sys
import time

import numpy as np

from sender.arm.vive_tracker import ViveTracker


def main():
    parser = argparse.ArgumentParser(description="Step 2: Pose reading test")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Test duration in seconds (default: 3)")
    parser.add_argument("--serial", default=None, help="Tracker serial")
    parser.add_argument("--hz", type=int, default=50, help="Read rate (default: 50)")
    args = parser.parse_args()

    print("=" * 50)
    print("  Step 2: Tracker Pose Reading Test")
    print("=" * 50)
    passed = 0
    failed = 0

    tracker = ViveTracker(tracker_serial=args.serial)
    try:
        tracker.connect()
    except Exception as e:
        print(f"[FAIL] Cannot connect: {e}")
        print("       Run Step 1 first to verify OpenVR connection")
        sys.exit(1)

    dt = 1.0 / args.hz
    total_reads = int(args.duration * args.hz)
    positions = []
    quat_norms = []
    valid_count = 0
    t_start = time.perf_counter()

    print(f"\nReading {total_reads} frames at {args.hz} Hz ({args.duration}s)...")
    for i in range(total_reads):
        frame_start = time.perf_counter()
        result = tracker.get_pose()
        if result is not None:
            pos, quat = result
            positions.append(pos.copy())
            quat_norms.append(np.linalg.norm(quat))
            valid_count += 1
        elapsed = time.perf_counter() - frame_start
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)

    t_elapsed = time.perf_counter() - t_start
    tracker.disconnect()

    # Test 1: Valid pose percentage
    pct = 100.0 * valid_count / total_reads
    print(f"\n[TEST] Valid poses: {valid_count}/{total_reads} ({pct:.1f}%)", end=" ")
    if pct >= 90:
        print("[PASS]")
        passed += 1
    elif pct >= 50:
        print("[WARN] Low tracking rate — check base station line of sight")
        failed += 1
    else:
        print("[FAIL] Very low tracking rate")
        failed += 1

    if valid_count == 0:
        print("[FAIL] No valid poses received — cannot proceed")
        _summary(passed, failed + 1)
        return

    positions = np.array(positions)

    # Test 2: Position range (sanity)
    pos_min = positions.min(axis=0)
    pos_max = positions.max(axis=0)
    pos_range = pos_max - pos_min
    print(f"[TEST] Position range:")
    print(f"       X: [{pos_min[0]:+.3f}, {pos_max[0]:+.3f}] (range {pos_range[0]:.3f}m)")
    print(f"       Y: [{pos_min[1]:+.3f}, {pos_max[1]:+.3f}] (range {pos_range[1]:.3f}m)")
    print(f"       Z: [{pos_min[2]:+.3f}, {pos_max[2]:+.3f}] (range {pos_range[2]:.3f}m)", end=" ")

    # Check all positions are within 10m (sanity)
    if np.all(np.abs(positions) < 10.0):
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Positions outside 10m — sensor error?")
        failed += 1

    # Test 3: Quaternion normalization
    quat_norms = np.array(quat_norms)
    qn_min, qn_max = quat_norms.min(), quat_norms.max()
    print(f"[TEST] Quaternion norm: min={qn_min:.6f}, max={qn_max:.6f}", end=" ")
    if 0.999 < qn_min and qn_max < 1.001:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Quaternions not unit — OpenVR data error")
        failed += 1

    # Test 4: Frequency
    actual_hz = total_reads / t_elapsed
    print(f"[TEST] Frequency: {actual_hz:.1f} Hz (target: {args.hz} Hz)", end=" ")
    if abs(actual_hz - args.hz) / args.hz < 0.1:  # within 10%
        print("[PASS]")
        passed += 1
    else:
        print("[WARN] Frequency off target")
        failed += 1

    # Test 5: Noise (if stationary)
    pos_std = positions.std(axis=0)
    print(f"[INFO] Position std dev (hold still for accuracy):")
    print(f"       X: {pos_std[0]*1000:.2f}mm, Y: {pos_std[1]*1000:.2f}mm, Z: {pos_std[2]*1000:.2f}mm")

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 2 complete — proceed to Step 3")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 50}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
