"""Trajectory conversion and streaming execution (backend-agnostic)."""

import time
from typing import Dict, List

import numpy as np

from teleop_dev.robot.config import (
    MAX_JOINT_ACCEL_RAD_S2,
    MAX_JOINT_VEL_RAD_S,
    SERVOJ_DT,
)
from teleop_dev.robot.core.robot_backend import RobotBackend


def resample_trajectory(
    positions: np.ndarray,
    source_dt: float,
    target_dt: float,
) -> np.ndarray:
    """Resample trajectory from source_dt to target_dt using linear interpolation.

    Args:
        positions: (N, n_joints) array
        source_dt: original time step (e.g. 0.025s from curobo)
        target_dt: target time step (e.g. 0.008s for 125Hz)

    Returns:
        (M, n_joints) resampled array
    """
    n_points, n_joints = positions.shape
    source_times = np.arange(n_points) * source_dt
    total_time = source_times[-1]
    target_times = np.arange(0, total_time + target_dt * 0.5, target_dt)

    resampled = np.zeros((len(target_times), n_joints))
    for j in range(n_joints):
        resampled[:, j] = np.interp(target_times, source_times, positions[:, j])

    return resampled


def validate_trajectory(trajectory: Dict, tolerance: float = 0.05) -> List[str]:
    """Check trajectory for safety violations. Returns list of warnings."""
    warnings = []
    positions = trajectory["positions"]
    dt = trajectory["dt"]
    n_points = len(positions)

    if n_points < 2:
        warnings.append("Trajectory has fewer than 2 points")
        return warnings

    # Check velocities
    vel = np.diff(positions, axis=0) / dt
    max_vel = np.max(np.abs(vel))
    if max_vel > MAX_JOINT_VEL_RAD_S:
        warnings.append(
            f"Max velocity {max_vel:.2f} rad/s exceeds limit {MAX_JOINT_VEL_RAD_S:.2f}"
        )

    # Check accelerations
    if n_points >= 3:
        acc = np.diff(vel, axis=0) / dt
        max_acc = np.max(np.abs(acc))
        if max_acc > MAX_JOINT_ACCEL_RAD_S2:
            warnings.append(
                f"Max acceleration {max_acc:.2f} rad/s² exceeds limit {MAX_JOINT_ACCEL_RAD_S2:.2f}"
            )

    return warnings


def check_start_match(
    planned_start: np.ndarray, actual_joints: List[float], tolerance: float = 0.05
) -> bool:
    """Verify robot is at the planned start position."""
    diff = np.abs(planned_start - np.array(actual_joints))
    max_diff = np.max(diff)
    if max_diff > tolerance:
        print(f"[Safety] Start state mismatch! Max diff: {max_diff:.4f} rad (tolerance: {tolerance})")
        for i, d in enumerate(diff):
            if d > tolerance:
                print(f"  Joint {i}: planned={planned_start[i]:.4f}, actual={actual_joints[i]:.4f}")
        return False
    return True


def execute_trajectory(
    robot: RobotBackend,
    trajectory: Dict,
    command_dt: float = SERVOJ_DT,
):
    """Stream trajectory to robot via send_joint_command() at specified rate.

    Resamples curobo trajectory to command_dt and streams each waypoint
    with precise timing. Works with any RobotBackend (RTDE or Sim).
    """
    resampled = resample_trajectory(trajectory["positions"], trajectory["dt"], command_dt)
    n_points = len(resampled)
    total_time = n_points * command_dt

    print(f"[Executor] Streaming {n_points} points ({total_time:.2f}s) at {1/command_dt:.0f}Hz...")

    t_start = time.perf_counter()
    for i in range(n_points):
        q = resampled[i].tolist()
        robot.send_joint_command(q)

        # Precise timing — busy-wait to next step
        t_target = t_start + (i + 1) * command_dt
        while time.perf_counter() < t_target:
            pass

    robot.on_trajectory_done()
    elapsed = time.perf_counter() - t_start
    print(f"[Executor] Done! Elapsed: {elapsed:.3f}s (expected: {total_time:.3f}s)")
