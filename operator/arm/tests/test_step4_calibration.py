#!/usr/bin/env python3
"""Step 4: Calibration math verification.

Tests the coordinate transform computation with known inputs (NO SteamVR required).
Can be run on any machine.

Usage: python3 -m vive.tests.test_step4_calibration
"""

import math
import sys

import numpy as np

from teleop_dev.operator.arm.calibrate import (
    compute_calibration,
    transform_pose,
    _rot_to_quat,
    _quat_multiply,
)


def main():
    print("=" * 50)
    print("  Step 4: Calibration Math Verification")
    print("=" * 50)
    passed = 0
    failed = 0

    # Test 1: Identity calibration
    # If Vive coords == Robot coords, R should be identity, t should be zero
    print("\n[TEST] Identity calibration (Vive == Robot)...", end=" ")
    origin = np.array([0.0, 0.0, 0.0])
    x_axis = np.array([1.0, 0.0, 0.0])
    y_axis = np.array([0.0, 1.0, 0.0])
    R, t = compute_calibration(origin, x_axis, y_axis)
    err_R = np.linalg.norm(R - np.eye(3))
    err_t = np.linalg.norm(t)
    if err_R < 1e-10 and err_t < 1e-10:
        print(f"[PASS] (R error={err_R:.2e}, t error={err_t:.2e})")
        passed += 1
    else:
        print(f"[FAIL] R error={err_R:.2e}, t error={err_t:.2e}")
        failed += 1

    # Test 2: Translation only (offset origin)
    print("[TEST] Translation-only calibration...", end=" ")
    offset = np.array([2.0, 3.0, 1.0])
    R2, t2 = compute_calibration(offset, offset + x_axis, offset + y_axis)
    err_R2 = np.linalg.norm(R2 - np.eye(3))
    err_t2 = np.linalg.norm(t2 + offset)  # t should be -offset
    if err_R2 < 1e-10 and err_t2 < 1e-10:
        print(f"[PASS] (R error={err_R2:.2e}, t error={err_t2:.2e})")
        passed += 1
    else:
        print(f"[FAIL] R error={err_R2:.2e}, t error={err_t2:.2e}")
        failed += 1

    # Test 3: 90-degree rotation around Z
    # SteamVR Y-up → Robot Z-up: classic swap
    print("[TEST] 90-degree Z rotation calibration...", end=" ")
    # In Vive space, robot X=(0,0,-1), robot Y=(1,0,0)
    p_origin_v = np.array([0.0, 0.0, 0.0])
    p_xaxis_v = np.array([0.0, 0.0, -1.0])  # robot +X is Vive -Z
    p_yaxis_v = np.array([1.0, 0.0, 0.0])    # robot +Y is Vive +X
    R3, t3 = compute_calibration(p_origin_v, p_xaxis_v, p_yaxis_v)

    # Verify: (0,0,-1) in Vive → (1,0,0) in Robot (i.e. +X direction)
    test_pt = R3 @ p_xaxis_v + t3
    expected = np.array([1.0, 0.0, 0.0])  # should be along +X, normalized
    test_pt_norm = test_pt / np.linalg.norm(test_pt)
    err3 = np.linalg.norm(test_pt_norm - expected)
    if err3 < 1e-6:
        print(f"[PASS] (direction error={err3:.2e})")
        passed += 1
    else:
        print(f"[FAIL] Expected {expected}, got direction {test_pt_norm}")
        failed += 1

    # Test 4: Quaternion rotation
    print("[TEST] Quaternion transform...", end=" ")
    # Identity rotation: q = [1,0,0,0]
    R_ident = np.eye(3)
    t_zero = np.zeros(3)
    pos_in = np.array([1.0, 2.0, 3.0])
    quat_in = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz identity
    pos_out, quat_out = transform_pose(pos_in, quat_in, R_ident, t_zero)
    err_pos = np.linalg.norm(pos_out - pos_in)
    err_quat = min(
        np.linalg.norm(quat_out - quat_in),
        np.linalg.norm(quat_out + quat_in),  # q and -q are same rotation
    )
    if err_pos < 1e-10 and err_quat < 1e-6:
        print(f"[PASS] (pos error={err_pos:.2e}, quat error={err_quat:.2e})")
        passed += 1
    else:
        print(f"[FAIL] pos error={err_pos:.2e}, quat error={err_quat:.2e}")
        failed += 1

    # Test 5: Round-trip (transform → verify known points)
    print("[TEST] Round-trip: calibrate then transform...", end=" ")
    # Create arbitrary calibration
    p0 = np.array([1.5, 0.8, -0.3])
    p1 = np.array([1.8, 0.8, -0.3])  # +X is 0.3m along Vive X
    p2 = np.array([1.5, 0.8, -0.6])  # +Y is 0.3m along Vive -Z
    R5, t5 = compute_calibration(p0, p1, p2)

    # Origin should map to (0,0,0)
    origin_robot = R5 @ p0 + t5
    err_origin = np.linalg.norm(origin_robot)

    # p1 should map to (+d, 0, 0)
    p1_robot = R5 @ p1 + t5
    err_p1_y = abs(p1_robot[1])
    err_p1_z = abs(p1_robot[2])
    err_p1_sign = 1 if p1_robot[0] > 0 else -1

    if err_origin < 1e-6 and err_p1_y < 1e-6 and err_p1_z < 1e-6 and err_p1_sign > 0:
        print(f"[PASS] (origin error={err_origin:.2e})")
        passed += 1
    else:
        print(f"[FAIL] origin={origin_robot}, p1_robot={p1_robot}")
        failed += 1

    # Test 6: _quat_multiply associativity
    print("[TEST] Quaternion multiply correctness...", end=" ")
    # 90-deg rotation around Z: q = [cos(45), 0, 0, sin(45)]
    q_z90 = np.array([math.cos(math.pi/4), 0, 0, math.sin(math.pi/4)])
    # Applying twice should give 180-deg around Z
    q_z180 = _quat_multiply(q_z90, q_z90)
    expected_z180 = np.array([0.0, 0.0, 0.0, 1.0])  # cos(90)=0, sin(90)=1
    err_qm = min(
        np.linalg.norm(q_z180 - expected_z180),
        np.linalg.norm(q_z180 + expected_z180),
    )
    if err_qm < 1e-6:
        print(f"[PASS] (error={err_qm:.2e})")
        passed += 1
    else:
        print(f"[FAIL] Expected ±{expected_z180}, got {q_z180}")
        failed += 1

    # Summary
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 4 complete — proceed to Step 5")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 50}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
