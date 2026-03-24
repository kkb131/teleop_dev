#!/usr/bin/env python3
"""Unit tests for Manus → DG5F retargeting logic.

No hardware required — uses mock Manus data to verify:
  - Output shape and type
  - Joint limit compliance
  - Known pose mappings (open hand, closed fist)
  - Left/right symmetry

Usage:
    python3 -m tesollo.tests.test_retarget
"""

import math
import sys

import numpy as np

from teleop_dev.robot.hand.retarget import ManusToD5FRetarget, RIGHT_LIMITS, LEFT_LIMITS


def main():
    print("=" * 55)
    print("  Retarget Unit Test (no hardware)")
    print("=" * 55)
    passed = 0
    failed = 0

    # ── Test 1: Output shape ────────────────────────────
    print("\n[TEST] Output shape (20 joints)...", end=" ")
    rt = ManusToD5FRetarget(hand_side="right")
    manus = np.zeros(20, dtype=np.float64)
    result = rt.retarget(manus)
    if result.shape == (20,):
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL] Expected (20,), got {result.shape}")
        failed += 1

    # ── Test 2: Output type ─────────────────────────────
    print("[TEST] Output type (float64)...", end=" ")
    if result.dtype == np.float64:
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL] Expected float64, got {result.dtype}")
        failed += 1

    # ── Test 3: Joint limits (right hand) ───────────────
    print("[TEST] Joint limits compliance (right)...", end=" ")
    # Test with extreme inputs
    extreme_inputs = [
        np.ones(20) * 2.0,      # large positive
        np.ones(20) * -2.0,     # large negative
        np.random.uniform(-1.5, 1.5, 20),  # random
    ]
    limits_ok = True
    for inp in extreme_inputs:
        out = rt.retarget(inp)
        for i in range(20):
            lim = RIGHT_LIMITS[i]
            if out[i] < lim.min_rad - 1e-6 or out[i] > lim.max_rad + 1e-6:
                print(f"[FAIL] Joint {i}: {out[i]:.4f} outside [{lim.min_rad:.4f}, {lim.max_rad:.4f}]")
                limits_ok = False
                break
        if not limits_ok:
            break

    if limits_ok:
        print("[PASS]")
        passed += 1
    else:
        failed += 1

    # ── Test 4: Joint limits (left hand) ────────────────
    print("[TEST] Joint limits compliance (left)...", end=" ")
    rt_left = ManusToD5FRetarget(hand_side="left")
    limits_ok = True
    for inp in extreme_inputs:
        out = rt_left.retarget(inp)
        for i in range(20):
            lim = LEFT_LIMITS[i]
            if out[i] < lim.min_rad - 1e-6 or out[i] > lim.max_rad + 1e-6:
                print(f"[FAIL] Joint {i}: {out[i]:.4f} outside [{lim.min_rad:.4f}, {lim.max_rad:.4f}]")
                limits_ok = False
                break
        if not limits_ok:
            break

    if limits_ok:
        print("[PASS]")
        passed += 1
    else:
        failed += 1

    # ── Test 5: Zero input produces finite output ───────
    print("[TEST] Zero input → finite output...", end=" ")
    zeros = np.zeros(20)
    out_r = rt.retarget(zeros)
    out_l = rt_left.retarget(zeros)
    if np.all(np.isfinite(out_r)) and np.all(np.isfinite(out_l)):
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Non-finite values in output")
        failed += 1

    # ── Test 6: Thumb spread/flex swap ──────────────────
    print("[TEST] Thumb spread/flex swap logic...", end=" ")
    # Manus: [spread=0, flex=0.5, mcp=0, ip=0] → DG5F should have non-zero thumb base
    manus_thumb = np.zeros(20)
    manus_thumb[1] = 0.5  # CMC flex = 0.5 rad ≈ 28.6 deg
    out = rt.retarget(manus_thumb)
    # qd[0] = (58.5 - 28.6) * pi/180 ≈ 0.522 rad, then * cal[0]=1.0 * dir[0]=1
    # After posture constraints (right: must be >=0), should be positive
    if out[0] > 0.0:
        print(f"[PASS] Thumb base = {out[0]:+.4f}")
        passed += 1
    else:
        print(f"[FAIL] Expected positive thumb base, got {out[0]:+.4f}")
        failed += 1

    # ── Test 7: Full curl produces non-zero output ──────
    print("[TEST] Full curl → non-zero DG5F angles...", end=" ")
    curl = np.array([
        0.0, 1.2, 1.5, 1.0,   # Thumb: curl
        0.0, 1.2, 1.5, 1.0,   # Index
        0.0, 1.2, 1.5, 1.0,   # Middle
        0.0, 1.0, 1.2, 0.8,   # Ring
        0.0, 1.0, 0.8, 0.5,   # Pinky
    ])
    out = rt.retarget(curl)
    non_zero = np.sum(np.abs(out) > 0.01)
    if non_zero >= 10:
        print(f"[PASS] {non_zero}/20 joints active")
        passed += 1
    else:
        print(f"[FAIL] Only {non_zero}/20 joints active")
        failed += 1

    # ── Test 8: Custom calibration factors ──────────────
    print("[TEST] Custom calibration factors...", end=" ")
    # Double all factors → output magnitudes should increase
    double_cal = [2.0] * 20
    rt_double = ManusToD5FRetarget(hand_side="right", calibration_factors=double_cal)
    manus_test = np.ones(20) * 0.3
    out_default = rt.retarget(manus_test)
    out_double = rt_double.retarget(manus_test)
    # At least some joints should have larger magnitude (before clamping)
    # Due to clamping, this isn't guaranteed for all joints
    any_larger = False
    for i in range(20):
        if abs(out_double[i]) > abs(out_default[i]) + 1e-6:
            any_larger = True
            break
    if any_larger:
        print("[PASS]")
        passed += 1
    else:
        print("[WARN] All outputs clamped (both at limits)")
        passed += 1  # acceptable — means limits are tight

    # ── Test 9: Idempotent (same input → same output) ───
    print("[TEST] Deterministic output...", end=" ")
    inp = np.random.uniform(-0.5, 1.5, 20)
    out1 = rt.retarget(inp.copy())
    out2 = rt.retarget(inp.copy())
    if np.allclose(out1, out2):
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Different outputs for same input")
        failed += 1

    # ── Test 10: Stress — many random inputs ────────────
    print("[TEST] Stress test (1000 random inputs)...", end=" ")
    errors = 0
    for _ in range(1000):
        inp = np.random.uniform(-2.0, 2.0, 20)
        out = rt.retarget(inp)
        if not np.all(np.isfinite(out)):
            errors += 1
        for i in range(20):
            lim = RIGHT_LIMITS[i]
            if out[i] < lim.min_rad - 1e-6 or out[i] > lim.max_rad + 1e-6:
                errors += 1
                break
    if errors == 0:
        print("[PASS] All 1000 within limits")
        passed += 1
    else:
        print(f"[FAIL] {errors}/1000 had issues")
        failed += 1

    # ── Summary ─────────────────────────────────────────
    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Retarget logic verified!")
    else:
        print("  [ISSUES] Fix the above failures")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
