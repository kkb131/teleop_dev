#!/usr/bin/env python3
"""Coordinate frame calibration: SteamVR → UR10e robot base frame.

3-point calibration method:
  1. Place tracker at robot base_link origin → record SteamVR coords
  2. Place tracker along robot +X axis → record SteamVR coords
  3. Place tracker along robot +Y axis (XY plane) → record SteamVR coords

Computes rotation R and translation t such that:
    pos_robot = R @ pos_vive + t

Saves calibration to JSON file for use by ViveNetworkInput.

Usage:
    python3 -m vive.calibrate --output calibration.json
    python3 -m vive.calibrate --output calibration.json --tracker-serial LHR-xxx
"""

import argparse
import json
import time
from datetime import datetime

import numpy as np

from teleop_dev.operator.arm.vive_tracker import ViveTracker


def _collect_point(tracker: ViveTracker, label: str,
                   samples: int = 50, hz: float = 50.0) -> np.ndarray:
    """Collect averaged position from tracker.

    Averages multiple samples to reduce noise.
    """
    print(f"\n>>> Place tracker at: {label}")
    input("    Press Enter when ready...")

    positions = []
    dt = 1.0 / hz
    print(f"    Collecting {samples} samples...", end="", flush=True)

    for _ in range(samples):
        t_start = time.perf_counter()
        result = tracker.get_pose()
        if result is not None:
            pos, _ = result
            positions.append(pos.copy())
        elapsed = time.perf_counter() - t_start
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)

    if len(positions) < samples // 2:
        raise RuntimeError(
            f"Only got {len(positions)}/{samples} valid samples. "
            "Check tracker tracking."
        )

    avg = np.mean(positions, axis=0)
    std = np.std(positions, axis=0)
    print(f" done ({len(positions)} samples)")
    print(f"    Position: ({avg[0]:+.4f}, {avg[1]:+.4f}, {avg[2]:+.4f})")
    print(f"    Std dev:  ({std[0]:.5f}, {std[1]:.5f}, {std[2]:.5f})")

    if np.any(std > 0.005):
        print("    [WARNING] High std dev — tracker may be unstable")

    return avg


def compute_calibration(
    p_origin_vive: np.ndarray,
    p_xaxis_vive: np.ndarray,
    p_yaxis_vive: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute R, t from 3 calibration points.

    Parameters
    ----------
    p_origin_vive : Robot base origin in SteamVR coords
    p_xaxis_vive  : Point along robot +X in SteamVR coords
    p_yaxis_vive  : Point along robot +Y in SteamVR coords

    Returns
    -------
    R : (3,3) rotation matrix, maps SteamVR → robot frame
    t : (3,) translation vector
    """
    # Vectors in Vive space
    vx = p_xaxis_vive - p_origin_vive
    vy_raw = p_yaxis_vive - p_origin_vive

    # Normalize X axis
    vx_norm = vx / np.linalg.norm(vx)

    # Z = X × Y_raw (right-hand rule), then normalize
    vz = np.cross(vx_norm, vy_raw)
    vz_norm = vz / np.linalg.norm(vz)

    # Recompute Y = Z × X for orthogonality
    vy_norm = np.cross(vz_norm, vx_norm)

    # R maps Vive → Robot:
    # Robot axes expressed in Vive frame are the rows of R
    # [vx_norm, vy_norm, vz_norm] as rows
    R = np.stack([vx_norm, vy_norm, vz_norm], axis=0)

    # Translation: robot origin in robot frame = 0, so t = -R @ p_origin_vive
    t = -R @ p_origin_vive

    return R, t


def save_calibration(filepath: str, R: np.ndarray, t: np.ndarray,
                     points_vive: list, points_robot: list):
    """Save calibration data to JSON."""
    data = {
        "R": R.tolist(),
        "t": t.tolist(),
        "timestamp": datetime.now().isoformat(),
        "points_vive": [p.tolist() for p in points_vive],
        "points_robot": [p.tolist() for p in points_robot],
        "description": "SteamVR → Robot base frame transform: pos_robot = R @ pos_vive + t",
    }
    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n[Calibration] Saved to {filepath}")


def load_calibration(filepath: str) -> tuple[np.ndarray, np.ndarray]:
    """Load calibration R, t from JSON file.

    Returns
    -------
    R : (3,3) rotation matrix
    t : (3,) translation vector
    """
    with open(filepath) as f:
        data = json.load(f)
    R = np.array(data["R"])
    t = np.array(data["t"])
    return R, t


def transform_pose(pos_vive: np.ndarray, quat_vive: np.ndarray,
                   R: np.ndarray, t: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray]:
    """Transform a Vive pose to robot base frame.

    Parameters
    ----------
    pos_vive : (3,) position in SteamVR coords
    quat_vive : (4,) quaternion wxyz in SteamVR coords
    R : (3,3) calibration rotation
    t : (3,) calibration translation

    Returns
    -------
    pos_robot : (3,) position in robot frame
    quat_robot : (4,) quaternion wxyz in robot frame
    """
    # Position transform
    pos_robot = R @ pos_vive + t

    # Quaternion transform: q_robot = q_R * q_vive
    # Convert R to quaternion for rotation composition
    q_R = _rot_to_quat(R)
    quat_robot = _quat_multiply(q_R, quat_vive)

    return pos_robot, quat_robot


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion (wxyz)."""
    import math
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return q


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions (wxyz convention)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def main():
    parser = argparse.ArgumentParser(
        description="Vive Tracker ↔ Robot coordinate calibration"
    )
    parser.add_argument("--output", "-o", default="calibration.json",
                        help="Output calibration file (default: calibration.json)")
    parser.add_argument("--tracker-serial", default=None,
                        help="Tracker serial number")
    parser.add_argument("--samples", type=int, default=50,
                        help="Samples per point (default: 50)")
    parser.add_argument("--verify", default=None,
                        help="Verify existing calibration file instead of creating new one")
    args = parser.parse_args()

    if args.verify:
        _verify_calibration(args.verify, args.tracker_serial)
        return

    print("=" * 60)
    print("  Vive Tracker → Robot Base Frame Calibration")
    print("=" * 60)
    print()
    print("You will place the tracker at 3 known positions relative to")
    print("the robot base frame. Use a ruler or known reference points.")
    print()
    print("Robot coordinate convention (UR10e):")
    print("  X = right, Y = forward (away from base), Z = up")
    print()

    with ViveTracker(tracker_serial=args.tracker_serial) as tracker:
        # Collect 3 calibration points
        p_origin = _collect_point(
            tracker,
            "ROBOT BASE ORIGIN (where base_link frame is)",
            samples=args.samples,
        )
        p_xaxis = _collect_point(
            tracker,
            "ROBOT +X AXIS (e.g., 30cm to the right of origin)",
            samples=args.samples,
        )
        p_yaxis = _collect_point(
            tracker,
            "ROBOT +Y AXIS (e.g., 30cm forward from origin)",
            samples=args.samples,
        )

    # Compute calibration
    R, t = compute_calibration(p_origin, p_xaxis, p_yaxis)

    print("\n--- Calibration Result ---")
    print(f"R =\n{R}")
    print(f"t = {t}")

    # Verify: transform calibration points → should give (0,0,0), (+d,0,0), (0,+d,0)
    print("\n--- Verification ---")
    p0 = R @ p_origin + t
    p1 = R @ p_xaxis + t
    p2 = R @ p_yaxis + t
    print(f"Origin → robot: ({p0[0]:+.4f}, {p0[1]:+.4f}, {p0[2]:+.4f})  [expect ~0,0,0]")
    print(f"X-axis → robot: ({p1[0]:+.4f}, {p1[1]:+.4f}, {p1[2]:+.4f})  [expect +d,0,0]")
    print(f"Y-axis → robot: ({p2[0]:+.4f}, {p2[1]:+.4f}, {p2[2]:+.4f})  [expect 0,+d,0]")

    # Save
    save_calibration(
        args.output, R, t,
        points_vive=[p_origin, p_xaxis, p_yaxis],
        points_robot=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
    )


def _verify_calibration(filepath: str, tracker_serial: str | None):
    """Verify an existing calibration by showing live transformed poses."""
    R, t = load_calibration(filepath)
    print(f"[Verify] Loaded calibration from {filepath}")
    print(f"[Verify] Showing live poses in robot frame. Ctrl+C to stop.\n")

    with ViveTracker(tracker_serial=tracker_serial) as tracker:
        count = 0
        try:
            while True:
                result = tracker.get_pose()
                if result is None:
                    print("\r[LOST]", end="", flush=True)
                else:
                    pos_v, quat_v = result
                    pos_r, quat_r = transform_pose(pos_v, quat_v, R, t)
                    print(
                        f"\r[{count:5d}] "
                        f"robot pos=({pos_r[0]:+7.4f}, {pos_r[1]:+7.4f}, {pos_r[2]:+7.4f})  "
                        f"vive pos=({pos_v[0]:+7.4f}, {pos_v[1]:+7.4f}, {pos_v[2]:+7.4f})",
                        end="", flush=True,
                    )
                count += 1
                time.sleep(0.02)
        except KeyboardInterrupt:
            print(f"\n[Verify] Done. {count} reads.")


if __name__ == "__main__":
    main()
