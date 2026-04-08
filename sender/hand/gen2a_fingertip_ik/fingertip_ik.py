"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.
Abduction fixed from Manus Ergonomics, 3-DOF IK for MCP/PIP/DIP.

Coordinate strategy (validated at commit 853c174):
    - R = identity (no rotation — assumes Manus and DG5F share axes)
    - p_target = p_human_wrist_relative * scale + palm_offset
    - palm_offset shifts from wrist origin to palm region
    - Scale = robot_tip_dist / human_tip_dist (auto from first frame)

Pipeline:
    skeleton (N, 7) + ergonomics (20,)
    → extract fingertip positions (wrist-relative)
    → scale by bone length ratio
    → add palm_offset
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
        DLS damping factor (0.01~0.1).
    ema_alpha : float
        EMA filter strength (0=frozen, 1=no filter).
    """

    def __init__(self, hand_side: str = "right",
                 damping: float = 0.05, ema_alpha: float = 0.4):
        super().__init__(hand_side)

        self._fk = DG5FKinematics(hand_side)
        self._solvers = [PerFingerIK(self._fk, i, damping) for i in range(5)]
        self._scale = np.ones(5)
        self._is_calibrated = False

        # Palm offset: shift from URDF origin (wrist) to palm region
        # This is the key offset that makes IK work (validated at 853c174)
        self._palm_offset = self._fk.palm_position(np.zeros(NUM_JOINTS))

        # Robot tip distances at q=0 (for scale computation)
        self._robot_tips_at_zero = self._fk.fingertip_positions(np.zeros(NUM_JOINTS))
        self._robot_tip_dists = np.array([
            np.linalg.norm(self._robot_tips_at_zero[i])
            for i in range(5)
        ])

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)

        # Debug
        self._last_targets = np.zeros((5, 3))
        self._last_achieved = np.zeros((5, 3))
        self._last_errors = np.zeros(5)

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Compute per-finger bone length scale from open-hand skeleton.

        scale[i] = robot_tip_dist[i] / human_tip_dist[i]
        No rotation (R=I). Palm offset is fixed from URDF.
        """
        max_idx = max(MANUS_TIP_INDICES)
        if len(skeleton) <= max_idx:
            print(f"[2A-Cal] WARNING: skeleton has {len(skeleton)} nodes, "
                  f"need {max_idx + 1}. Skipping calibration.")
            return

        wrist = skeleton[0, :3]
        human_dists = np.array([
            np.linalg.norm(skeleton[idx, :3] - wrist)
            for idx in MANUS_TIP_INDICES
        ])

        self._scale = self._robot_tip_dists / np.maximum(human_dists, 1e-6)
        self._is_calibrated = True

        print(f"[2A-Cal] Scale: {[f'{s:.3f}' for s in self._scale]}")
        print(f"[2A-Cal] Human dists (m): {[f'{d:.4f}' for d in human_dists]}")
        print(f"[2A-Cal] Robot dists (m): {[f'{d:.4f}' for d in self._robot_tip_dists]}")
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

        # Auto-calibrate from first valid skeleton
        if not self._is_calibrated:
            max_idx = max(MANUS_TIP_INDICES)
            if len(skeleton) > max_idx:
                print("[2A] Auto-calibrating from first skeleton frame...")
                self.calibrate(skeleton=skeleton)

        wrist = skeleton[0, :3]
        q = self._q_prev.copy()

        for f in range(5):
            tip_idx = MANUS_TIP_INDICES[f]
            if tip_idx >= len(skeleton):
                continue

            # 1. Human tip: wrist-relative
            p_human_local = skeleton[tip_idx, :3] - wrist

            # 2. Scale + palm offset (NO rotation, validated at 853c174)
            p_scaled = p_human_local * self._scale[f]
            p_target = p_scaled + self._palm_offset
            self._last_targets[f] = p_target

            # 3. Abduction from Ergonomics
            abd_angle = ergonomics[f * 4]

            # 4. Per-finger IK
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
        }

    def reset_filter(self):
        self._ema.reset()
        self._q_prev = np.zeros(NUM_JOINTS)
