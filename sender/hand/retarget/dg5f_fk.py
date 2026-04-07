#!/usr/bin/env python3
"""DG5F forward kinematics via Pinocchio.

Loads DG5F URDF and provides fingertip positions + Jacobians
for vector-based retargeting optimization.

Usage:
    from sender.hand.dg5f_fk import DG5FKinematics
    fk = DG5FKinematics(hand_side="right")
    tips = fk.fingertip_positions(q)  # (5, 3)
    J = fk.fingertip_jacobian(q)      # (15, 20)
"""

import os
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError("pinocchio required: pip install pin")


# Default URDF paths
_URDF_DIR = Path(__file__).resolve().parent.parent.parent.parent / "dg5f_ros2" / "dg5f_description" / "urdf"

RIGHT_URDF = str(_URDF_DIR / "dg5f_right.urdf")
LEFT_URDF = str(_URDF_DIR / "dg5f_left.urdf")

# Fingertip frame names
RIGHT_TIP_FRAMES = [f"rl_dg_{i}_tip" for i in range(1, 6)]
LEFT_TIP_FRAMES = [f"ll_dg_{i}_tip" for i in range(1, 6)]

# Palm frame (base for vectors)
RIGHT_PALM = "rl_dg_palm"
LEFT_PALM = "ll_dg_palm"

# Actuated joint names (canonical order)
RIGHT_JOINTS = [f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
LEFT_JOINTS = [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]


class DG5FKinematics:
    """Forward kinematics for DG5F hand using Pinocchio.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    urdf_path : str or None
        Override URDF path. None uses default.
    """

    def __init__(self, hand_side: str = "right", urdf_path: str = None):
        self._side = hand_side
        urdf = urdf_path or (RIGHT_URDF if hand_side == "right" else LEFT_URDF)

        if not os.path.exists(urdf):
            raise FileNotFoundError(f"URDF not found: {urdf}")

        # Load model
        self._model = pin.buildModelFromUrdf(urdf)
        self._data = self._model.createData()

        # Resolve frame IDs
        tip_names = RIGHT_TIP_FRAMES if hand_side == "right" else LEFT_TIP_FRAMES
        palm_name = RIGHT_PALM if hand_side == "right" else LEFT_PALM

        self._tip_frame_ids = []
        for name in tip_names:
            fid = self._model.getFrameId(name)
            if fid >= self._model.nframes:
                raise ValueError(f"Frame '{name}' not found in URDF")
            self._tip_frame_ids.append(fid)

        self._palm_frame_id = self._model.getFrameId(palm_name)

        # Build joint index mapping: canonical order → pinocchio q index
        joint_names = RIGHT_JOINTS if hand_side == "right" else LEFT_JOINTS
        self._q_indices = []
        for jname in joint_names:
            jid = self._model.getJointId(jname)
            if jid >= self._model.njoints:
                raise ValueError(f"Joint '{jname}' not found in URDF")
            idx_q = self._model.joints[jid].idx_q
            self._q_indices.append(idx_q)

        self._nq = self._model.nq
        self._nv = self._model.nv

        # Joint limits
        self._q_min = self._model.lowerPositionLimit.copy()
        self._q_max = self._model.upperPositionLimit.copy()

    @property
    def nq(self) -> int:
        return self._nq

    @property
    def num_actuated(self) -> int:
        return 20

    @property
    def q_min(self) -> np.ndarray:
        """Joint lower limits in canonical order (20,)."""
        return np.array([self._q_min[i] for i in self._q_indices])

    @property
    def q_max(self) -> np.ndarray:
        """Joint upper limits in canonical order (20,)."""
        return np.array([self._q_max[i] for i in self._q_indices])

    def _to_pin_q(self, q_canonical: np.ndarray) -> np.ndarray:
        """Convert 20-element canonical q to pinocchio q vector."""
        q_pin = pin.neutral(self._model)
        for i, idx in enumerate(self._q_indices):
            q_pin[idx] = q_canonical[i]
        return q_pin

    def fingertip_positions(self, q: np.ndarray) -> np.ndarray:
        """Compute 5 fingertip positions in world frame.

        Parameters
        ----------
        q : ndarray[20]
            Joint angles in canonical order (radians).

        Returns
        -------
        ndarray[5, 3]
            Fingertip (x, y, z) positions.
        """
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        tips = np.zeros((5, 3))
        for i, fid in enumerate(self._tip_frame_ids):
            tips[i] = self._data.oMf[fid].translation
        return tips

    def palm_position(self, q: np.ndarray) -> np.ndarray:
        """Compute palm position in world frame.

        Returns
        -------
        ndarray[3]
        """
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._palm_frame_id].translation.copy()

    def fingertip_vectors(self, q: np.ndarray) -> np.ndarray:
        """Compute palm→fingertip direction vectors (normalized).

        Returns
        -------
        ndarray[5, 3]
            Unit vectors from palm to each fingertip.
        """
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        palm_pos = self._data.oMf[self._palm_frame_id].translation
        vectors = np.zeros((5, 3))
        for i, fid in enumerate(self._tip_frame_ids):
            v = self._data.oMf[fid].translation - palm_pos
            norm = np.linalg.norm(v)
            if norm > 1e-6:
                vectors[i] = v / norm
            else:
                vectors[i] = v
        return vectors

    def fingertip_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute stacked fingertip position Jacobians.

        Returns
        -------
        ndarray[15, 20]
            Jacobian mapping 20 joint velocities to 5×3 fingertip velocities.
        """
        q_pin = self._to_pin_q(q)
        pin.computeJointJacobians(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        J_full = np.zeros((15, 20))
        for i, fid in enumerate(self._tip_frame_ids):
            # 6×nv Jacobian, take top 3 rows (linear velocity)
            J6 = pin.getFrameJacobian(
                self._model, self._data, fid, pin.LOCAL_WORLD_ALIGNED
            )[:3, :]  # (3, nv)
            # Map pinocchio columns to canonical joint order
            for j, idx_v in enumerate(self._q_indices):
                J_full[i * 3:(i + 1) * 3, j] = J6[:, idx_v]
        return J_full
