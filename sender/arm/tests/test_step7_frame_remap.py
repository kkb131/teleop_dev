#!/usr/bin/env python3
"""Step 7: per-arm frame remap 검증 (하드웨어 불필요).

XRRelativeFrameAligner 의 r_remap 파라미터화 검증:
  1. 기본값이 기존 검증 매핑 (손 +x→robot +x, +y→+z, +z→−y) 그대로인지
  2. remap_from_rpy_deg(90,0,0) == 기본 행렬
  3. remap_from_rpy_deg(90,0,180) — 미러(대면) 장착 시작값의 축 매핑
  4. validate_remap 이 reflection (det=−1) / 비직교 행렬 거부
  5. 회전 conjugation 이 위치 remap 과 일관되는지 (WebXR up 회전 → robot z 회전)

Usage: python3 -m sender.arm.tests.test_step7_frame_remap
"""

import math
import sys

import numpy as np

from sender.arm.xr_frame_align import (
    XRRelativeFrameAligner,
    _DEFAULT_R_REMAP,
    remap_from_rpy_deg,
    validate_remap,
    _quat_wxyz_to_rotmat,
)


def _pose(p, R=None):
    T = np.eye(4)
    T[:3, 3] = p
    if R is not None:
        T[:3, :3] = R
    return T


def _apply_delta(aligner, robot_origin, user_delta):
    """origin (identity) 캘리 후 user 를 user_delta 만큼 옮겼을 때 target pos 반환."""
    aligner.reset()
    aligner.calibrate(_pose([0.1, 0.2, 0.3]), np.array(robot_origin),
                      np.array([1.0, 0.0, 0.0, 0.0]))
    pos, _ = aligner.apply(_pose(np.array([0.1, 0.2, 0.3]) + np.array(user_delta)))
    return pos - np.array(robot_origin)


def main():
    print("=" * 50)
    print("  Step 7: Per-arm Frame Remap Verification")
    print("=" * 50)
    passed = 0
    failed = 0

    robot_origin = [0.5, -0.1, 0.4]

    # Test 1: default aligner reproduces current verified mapping
    print("\n[TEST] default remap: +x→+x, +y→+z, +z→−y ...", end=" ")
    al = XRRelativeFrameAligner(scale=1.0)
    d_x = _apply_delta(al, robot_origin, [0.1, 0, 0])
    d_y = _apply_delta(al, robot_origin, [0, 0.1, 0])
    d_z = _apply_delta(al, robot_origin, [0, 0, 0.1])
    ok = (np.allclose(d_x, [0.1, 0, 0], atol=1e-12)
          and np.allclose(d_y, [0, 0, 0.1], atol=1e-12)
          and np.allclose(d_z, [0, -0.1, 0], atol=1e-12))
    if ok:
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL] dx={d_x} dy={d_y} dz={d_z}")
        failed += 1

    # Test 2: remap_from_rpy_deg(90,0,0) == default matrix
    print("[TEST] remap_from_rpy_deg(90,0,0) == default ...", end=" ")
    R = remap_from_rpy_deg(90, 0, 0)
    if np.allclose(R, _DEFAULT_R_REMAP, atol=1e-12):
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL]\n{R}")
        failed += 1

    # Test 3: mirror mount [90,0,180]: +x→−x, +y→+z, +z→+y
    print("[TEST] mirror remap [90,0,180]: +x→−x, +y→+z, +z→+y ...", end=" ")
    R_mir = remap_from_rpy_deg(90, 0, 180)
    al_m = XRRelativeFrameAligner(scale=1.0, r_remap=R_mir)
    d_x = _apply_delta(al_m, robot_origin, [0.1, 0, 0])
    d_y = _apply_delta(al_m, robot_origin, [0, 0.1, 0])
    d_z = _apply_delta(al_m, robot_origin, [0, 0, 0.1])
    ok = (np.allclose(d_x, [-0.1, 0, 0], atol=1e-12)
          and np.allclose(d_y, [0, 0, 0.1], atol=1e-12)
          and np.allclose(d_z, [0, 0.1, 0], atol=1e-12))
    if ok and abs(np.linalg.det(R_mir) - 1.0) < 1e-9:
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL] dx={d_x} dy={d_y} dz={d_z} det={np.linalg.det(R_mir)}")
        failed += 1

    # Test 4: validate_remap rejects reflection & non-orthogonal
    print("[TEST] validate_remap rejects det=−1 / non-orthogonal ...", end=" ")
    bad_reflect = np.diag([1.0, 1.0, -1.0])          # det = −1
    bad_scale = np.eye(3) * 2.0                       # not orthogonal
    ok = True
    for bad in (bad_reflect, bad_scale, np.eye(4)):
        try:
            validate_remap(bad)
            ok = False
        except ValueError:
            pass
    # 정상 행렬은 통과
    try:
        validate_remap(remap_from_rpy_deg(90, 0, -90))
    except ValueError:
        ok = False
    if ok:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL]")
        failed += 1

    # Test 5: rotation conjugation consistency — WebXR +y(up) 축 회전이
    # robot +z(up) 축 회전으로 매핑 (default remap: y→z)
    print("[TEST] rotation conjugation: WebXR yaw → robot z-rot ...", end=" ")
    al = XRRelativeFrameAligner(scale=1.0)
    al.calibrate(_pose([0, 0, 0]), np.zeros(3), np.array([1.0, 0, 0, 0]))
    ang = math.radians(30)
    Ry = np.array([
        [math.cos(ang), 0, math.sin(ang)],
        [0, 1, 0],
        [-math.sin(ang), 0, math.cos(ang)],
    ])
    _, quat = al.apply(_pose([0, 0, 0], Ry))
    R_target = _quat_wxyz_to_rotmat(quat)
    ang_check = math.radians(30)
    Rz_expect = np.array([
        [math.cos(ang_check), -math.sin(ang_check), 0],
        [math.sin(ang_check), math.cos(ang_check), 0],
        [0, 0, 1],
    ])
    # conjugation: R_remap @ Ry @ R_remap.T == Rz (부호 포함) 확인
    if np.allclose(R_target, Rz_expect, atol=1e-9) or np.allclose(R_target, Rz_expect.T, atol=1e-9):
        sign = "+z" if np.allclose(R_target, Rz_expect, atol=1e-9) else "−z"
        print(f"[PASS] (robot {sign} 회전)")
        passed += 1
    else:
        print(f"[FAIL]\n{R_target}")
        failed += 1

    print("\n" + "=" * 50)
    print(f"  {passed} passed, {failed} failed")
    print("=" * 50)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
