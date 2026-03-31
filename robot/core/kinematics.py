#!/usr/bin/env python3
"""Pinocchio-based kinematics utilities for UR10e Cartesian control.

Provides FK, Jacobian computation, and Damped Least Squares (DLS)
differential IK using the Pinocchio rigid-body dynamics library.

Dependencies:
  - pinocchio (ros-humble-pinocchio)
  - numpy
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from robot.config import URDF_PATH

# Default frames
DEFAULT_EE_FRAME = 'tool0'


class PinocchioIK:
    """Pinocchio-based FK/Jacobian/DLS for a serial robot."""

    def __init__(self, urdf_path: str = URDF_PATH, ee_frame: str = DEFAULT_EE_FRAME):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq  # Number of joints (6 for UR10e)

    def get_ee_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Compute FK: return (position[3], rotation[3x3]) of end-effector.

        Args:
            q: Joint positions (nq,).

        Returns:
            (translation, rotation_matrix)
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        oMf = self.data.oMf[self.ee_frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def get_ee_rpy(self, q: np.ndarray) -> np.ndarray:
        """Compute FK and return end-effector RPY angles (roll, pitch, yaw)."""
        _, R = self.get_ee_pose(q)
        return pin.rpy.matrixToRpy(R)

    def get_jacobian(self, q: np.ndarray, local: bool = False) -> np.ndarray:
        """Compute the 6xN geometric Jacobian.

        Args:
            q: Joint positions (nq,).
            local: If False (default), Jacobian in world-aligned frame (base_link).
                   If True, Jacobian in local frame (tool0).

        Returns:
            Jacobian matrix (6, nq).
        """
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        ref = pin.ReferenceFrame.LOCAL if local else pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        return pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, ref)

    def compute_joint_delta(
        self,
        q: np.ndarray,
        twist: np.ndarray,
        dt: float,
        damping: float = 0.05,
        local: bool = False,
    ) -> np.ndarray:
        """Compute joint position delta using Damped Least Squares.

        DLS formula: dq = J^T @ inv(J @ J^T + lambda^2 * I) @ twist * dt

        Args:
            q: Current joint positions (nq,).
            twist: Desired Cartesian velocity [vx, vy, vz, wx, wy, wz].
            dt: Time step (1/rate).
            damping: Damping factor (lambda). Larger = more stable near singularity.
            local: Jacobian frame (False=base_link, True=tool0).

        Returns:
            Joint position delta (nq,).
        """
        J = self.get_jacobian(q, local=local)
        JJt = J @ J.T
        JJt_damped = JJt + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt_damped, twist * dt)
        return dq

    def clamp_positions(self, q: np.ndarray) -> np.ndarray:
        """Clamp joint positions to model limits."""
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (lower_limits, upper_limits) arrays."""
        return self.model.lowerPositionLimit.copy(), self.model.upperPositionLimit.copy()
