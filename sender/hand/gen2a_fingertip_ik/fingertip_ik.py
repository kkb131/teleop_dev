"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.
Abduction fixed from Manus Ergonomics, 3-DOF IK for MCP/PIP/DIP.

Pipeline:
    skeleton (N, 7) + ergonomics (20,)
    → extract fingertip positions (wrist-relative)
    → scale by bone length ratio
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


class FingertipIKRetarget(HandRetargetBase):
    """[2A] Manus Skeleton → Fingertip IK → DG5F.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    damping : float
        DLS damping factor (0.01~0.1). Higher = more stable, less precise.
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

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)

        # Debug
        self._last_targets = np.zeros((5, 3))
        self._last_errors = np.zeros(5)

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Open-hand calibration: compute bone length scale factors.

        Call once at startup with hand fully open.
        """
        self._scale = self._calibrator.calibrate(skeleton)
        self._is_calibrated = True
        print(f"[2A-Cal] Scale factors: {[f'{s:.3f}' for s in self._scale]}")

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
        q = self._q_prev.copy()  # warm-start from previous frame

        for f in range(5):
            tip_idx = MANUS_TIP_INDICES[f]
            if tip_idx >= len(skeleton):
                continue

            # 1. Target position: wrist-relative, scaled
            p_human = skeleton[tip_idx, :3] - wrist
            p_target = p_human * self._scale[f]
            self._last_targets[f] = p_target

            # 2. Abduction from Ergonomics (spread joint)
            abd_angle = ergonomics[f * 4]

            # 3. Per-finger IK
            q_init = q[f * 4 + 1: f * 4 + 4]  # previous MCP/PIP/DIP
            q_finger = self._solvers[f].solve(
                p_target=p_target + self._fk.fingertip_positions(np.zeros(20))[0] * 0,
                abd_angle=abd_angle,
                q_init=q_init,
                q_full=q,
            )
            q[f * 4: f * 4 + 4] = q_finger

            # Track error
            p_achieved = self._fk.finger_tip_position(q, f)
            self._last_errors[f] = np.linalg.norm(p_target - (p_achieved - wrist * 0))

        # 4. Clamp + EMA
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
        }

    def reset_filter(self):
        self._ema.reset()
        self._q_prev = np.zeros(NUM_JOINTS)
