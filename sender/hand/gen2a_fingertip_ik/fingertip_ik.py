"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.
Abduction fixed from Manus Ergonomics, 3-DOF IK for MCP/PIP/DIP.

Pipeline:
    skeleton (N, 7) + ergonomics (20,)
    → extract fingertip positions (wrist-relative)
    → align coordinate frame (SVD Procrustes at calibration)
    → scale by bone length ratio
    → offset to DG5F palm frame
    → per-finger DLS IK (abd=ergo, 3-DOF solve)
    → clamp → EMA filter → DG5F (20, rad)
"""

import numpy as np
from typing import Optional

from sender.hand.core.retarget_base import HandRetargetBase
from sender.hand.core.dg5f_config import NUM_JOINTS
from sender.hand.core.filters import EMAFilter
from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics
from sender.hand.gen2a_fingertip_ik.per_finger_ik import PerFingerIK
from sender.hand.gen2a_fingertip_ik.scale_calibrator import (
    ScaleCalibrator, MANUS_TIP_INDICES,
)


def _compute_procrustes(src: np.ndarray, dst: np.ndarray):
    """Compute rotation R and scale s that best align src→dst.

    src, dst: (N, 3) corresponding point sets.
    Returns R (3,3), s (scalar).
    """
    src_c = src - src.mean(axis=0)
    dst_c = dst - dst.mean(axis=0)
    H = src_c.T @ dst_c
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    s = np.sum(S) / np.sum(src_c ** 2)
    return R, s


class FingertipIKRetarget(HandRetargetBase):
    """[2A] Manus Skeleton → Fingertip IK → DG5F.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    damping : float
        DLS damping factor (0.01~0.1).
    ema_alpha : float
        EMA filter strength (0=frozen, 1=no filter).
    """

    def __init__(self, hand_side: str = "right",
                 damping: float = 0.05, ema_alpha: float = 0.4):
        super().__init__(hand_side)

        self._fk = DG5FKinematics(hand_side)
        self._solvers = [PerFingerIK(self._fk, i, damping) for i in range(5)]
        self._calibrator = ScaleCalibrator(self._fk)
        self._scale = np.ones(5)  # per-finger bone length scale
        self._is_calibrated = False

        # Coordinate alignment: Manus → DG5F frame
        self._R = np.eye(3)  # rotation matrix
        self._palm_offset = self._fk.palm_position(np.zeros(NUM_JOINTS))

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)

        # Debug
        self._last_targets = np.zeros((5, 3))
        self._last_achieved = np.zeros((5, 3))
        self._last_errors = np.zeros(5)

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Open-hand calibration.

        1. Compute bone length scale factors
        2. Compute SVD rotation alignment (Manus → DG5F frame)
        """
        # Scale factors
        self._scale = self._calibrator.calibrate(skeleton)
        print(f"[2A-Cal] Scale factors: {[f'{s:.3f}' for s in self._scale]}")

        # SVD alignment: match Manus tip directions to DG5F tip directions
        wrist = skeleton[0, :3]
        human_tips = np.array([skeleton[idx, :3] - wrist for idx in MANUS_TIP_INDICES])

        q_zero = np.zeros(NUM_JOINTS)
        robot_tips = self._fk.fingertip_positions(q_zero) - self._palm_offset

        self._R, _ = _compute_procrustes(human_tips, robot_tips)
        self._is_calibrated = True

        print(f"[2A-Cal] Rotation det={np.linalg.det(self._R):.3f}")
        print(f"[2A-Cal] Palm offset: {self._palm_offset}")

    def retarget(self, skeleton: np.ndarray = None,
                 ergonomics: np.ndarray = None, **kwargs) -> np.ndarray:
        """Manus skeleton + ergonomics → DG5F joint angles.

        Parameters
        ----------
        skeleton : ndarray[N, 7]
            Manus raw skeleton nodes [x,y,z,qw,qx,qy,qz].
        ergonomics : ndarray[20]
            Manus ergonomics (radians). Used for abduction angles.

        Returns
        -------
        ndarray[20] — DG5F joint angles (radians), clamped.
        """
        if skeleton is None or ergonomics is None:
            return self._q_prev.copy()

        wrist = skeleton[0, :3]
        q = self._q_prev.copy()  # warm-start

        for f in range(5):
            tip_idx = MANUS_TIP_INDICES[f]
            if tip_idx >= len(skeleton):
                continue

            # 1. Human tip: wrist-relative
            p_human_local = skeleton[tip_idx, :3] - wrist

            # 2. Transform to DG5F frame: rotate + scale + offset to palm
            p_aligned = self._R @ p_human_local
            p_scaled = p_aligned * self._scale[f]
            p_target = p_scaled + self._palm_offset  # in DG5F world frame
            self._last_targets[f] = p_target

            # 3. Abduction from Ergonomics
            abd_angle = ergonomics[f * 4]

            # 4. Per-finger IK (target in DG5F world frame = same as FK output)
            q_finger = self._solvers[f].solve(
                p_target=p_target,
                abd_angle=abd_angle,
                q_init=q[f * 4 + 1: f * 4 + 4],
                q_full=q,
            )
            q[f * 4: f * 4 + 4] = q_finger

            # Track error
            p_achieved = self._fk.finger_tip_position(q, f)
            self._last_achieved[f] = p_achieved
            self._last_errors[f] = np.linalg.norm(p_target - p_achieved)

        # 5. Clamp + EMA
        q = np.clip(q, self._fk.q_min, self._fk.q_max)
        q = self._ema.filter(q)
        self._q_prev = q.copy()
        return q

    def get_method_name(self) -> str:
        return "2A-fingertip-ik"

    def get_debug_info(self) -> Optional[dict]:
        return {
            "method": self.get_method_name(),
            "scale": self._scale.tolist(),
            "calibrated": self._is_calibrated,
            "tip_errors_mm": (self._last_errors * 1000).tolist(),
            "mean_error_mm": float(np.mean(self._last_errors) * 1000),
            "targets": self._last_targets.tolist(),
            "achieved": self._last_achieved.tolist(),
        }

    def reset_filter(self):
        self._ema.reset()
        self._q_prev = np.zeros(NUM_JOINTS)
