"""DG5F forward kinematics via Pinocchio.

Ported from retarget_dev/models/fingertip_ik/dg5f_fk.py with
per-finger FK and Jacobian methods for [2A] IK solver.
"""

import os
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError("pinocchio required: pip install pin")


_URDF_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent.parent
    / "dg5f_ros2" / "dg5f_description" / "urdf"
)

RIGHT_URDF = str(_URDF_DIR / "dg5f_right.urdf")
LEFT_URDF = str(_URDF_DIR / "dg5f_left.urdf")

RIGHT_TIP_FRAMES = [f"rl_dg_{i}_tip" for i in range(1, 6)]
LEFT_TIP_FRAMES = [f"ll_dg_{i}_tip" for i in range(1, 6)]
RIGHT_PALM = "rl_dg_palm"
LEFT_PALM = "ll_dg_palm"
RIGHT_JOINTS = [f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
LEFT_JOINTS = [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]


class DG5FKinematics:
    """Forward kinematics for DG5F hand using Pinocchio.

    Provides:
    - Full-hand FK (fingertip_positions)
    - Per-finger FK (finger_tip_position)
    - Per-finger Jacobian (finger_jacobian)
    """

    def __init__(self, hand_side: str = "right", urdf_path: str = None):
        self._side = hand_side
        urdf = urdf_path or (RIGHT_URDF if hand_side == "right" else LEFT_URDF)

        if not os.path.exists(urdf):
            raise FileNotFoundError(f"URDF not found: {urdf}")

        self._model = pin.buildModelFromUrdf(urdf)
        self._data = self._model.createData()

        tip_names = RIGHT_TIP_FRAMES if hand_side == "right" else LEFT_TIP_FRAMES
        palm_name = RIGHT_PALM if hand_side == "right" else LEFT_PALM

        self._tip_frame_ids = []
        for name in tip_names:
            fid = self._model.getFrameId(name)
            if fid >= self._model.nframes:
                raise ValueError(f"Frame '{name}' not found in URDF")
            self._tip_frame_ids.append(fid)

        self._palm_frame_id = self._model.getFrameId(palm_name)

        joint_names = RIGHT_JOINTS if hand_side == "right" else LEFT_JOINTS
        self._joint_names = joint_names
        self._q_indices = []
        for jname in joint_names:
            jid = self._model.getJointId(jname)
            if jid >= self._model.njoints:
                raise ValueError(f"Joint '{jname}' not found in URDF")
            self._q_indices.append(self._model.joints[jid].idx_q)

        # Per-finger joint indices in pinocchio v-space (for Jacobian column extraction)
        self._finger_v_indices = []
        for f in range(5):
            indices = []
            for j in range(4):
                jname = joint_names[f * 4 + j]
                jid = self._model.getJointId(jname)
                indices.append(self._model.joints[jid].idx_v)
            self._finger_v_indices.append(indices)

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def q_min(self) -> np.ndarray:
        return np.array([self._model.lowerPositionLimit[i] for i in self._q_indices])

    @property
    def q_max(self) -> np.ndarray:
        return np.array([self._model.upperPositionLimit[i] for i in self._q_indices])

    def _to_pin_q(self, q_canonical: np.ndarray) -> np.ndarray:
        q_pin = pin.neutral(self._model)
        for i, idx in enumerate(self._q_indices):
            q_pin[idx] = q_canonical[i]
        return q_pin

    def _update_fk(self, q_canonical: np.ndarray):
        q_pin = self._to_pin_q(q_canonical)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        return q_pin

    def fingertip_positions(self, q: np.ndarray) -> np.ndarray:
        """All 5 fingertip positions. Returns: ndarray[5, 3]"""
        self._update_fk(q)
        tips = np.zeros((5, 3))
        for i, fid in enumerate(self._tip_frame_ids):
            tips[i] = self._data.oMf[fid].translation
        return tips

    def palm_position(self, q: np.ndarray) -> np.ndarray:
        """Palm frame position. Returns: ndarray[3]"""
        self._update_fk(q)
        return self._data.oMf[self._palm_frame_id].translation.copy()

    def finger_tip_position(self, q: np.ndarray, finger_idx: int) -> np.ndarray:
        """Single fingertip position. Returns: ndarray[3]"""
        self._update_fk(q)
        return self._data.oMf[self._tip_frame_ids[finger_idx]].translation.copy()

    def finger_jacobian(self, q: np.ndarray, finger_idx: int) -> np.ndarray:
        """Position Jacobian for one fingertip w.r.t. its 4 joints.

        Returns: ndarray[3, 4] — 3D position rows × 4 joint columns
        """
        q_pin = self._to_pin_q(q)
        pin.computeJointJacobians(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        # Full 6×nv Jacobian in world frame (position rows = top 3)
        J_full = pin.getFrameJacobian(
            self._model, self._data,
            self._tip_frame_ids[finger_idx],
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        # Extract 3 position rows, 4 joint columns for this finger
        v_indices = self._finger_v_indices[finger_idx]
        J_finger = J_full[:3][:, v_indices]  # (3, 4)
        return J_finger
