"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.

Coordinate strategy: RELATIVE MOTION SCALING
    p_target = (human_now - human_cal) * scale + robot_tip_at_q0

    - cal pose (spread): human_now == human_cal → target = robot_q0 → IK=0°
    - fist: human tip closer to wrist → negative delta → target closer to palm → IK flexes
    - scale = robot_tip_dist / human_tip_dist (compensates hand size difference)

Pipeline:
    skeleton + ergonomics
    → wrist-relative tip positions
    → relative delta from calibration pose
    → scale delta
    → add robot tip at q=0
    → per-finger DLS IK
    → clamp → EMA → DG5F (20, rad)
"""

import numpy as np
from typing import Optional

from sender.hand.core.retarget_base import HandRetargetBase
from sender.hand.core.dg5f_config import NUM_JOINTS
from sender.hand.core.filters import EMAFilter
from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics
from sender.hand.gen2a_fingertip_ik.per_finger_ik import PerFingerIK
from sender.hand.gen2a_fingertip_ik.scale_calibrator import MANUS_TIP_INDICES


class FingertipIKRetarget(HandRetargetBase):

    def __init__(self, hand_side: str = "right",
                 damping: float = 0.05, ema_alpha: float = 0.4):
        super().__init__(hand_side)

        self._fk = DG5FKinematics(hand_side)
        self._solvers = [PerFingerIK(self._fk, i, damping) for i in range(5)]
        self._is_calibrated = False

        # Robot reference: tip positions at q=0 (wrist frame)
        self._robot_tips_q0 = self._fk.fingertip_positions(np.zeros(NUM_JOINTS))
        self._robot_tip_dists = np.array([
            np.linalg.norm(self._robot_tips_q0[i]) for i in range(5)
        ])

        # Calibration data (set from first frame or --calibrate)
        self._human_cal_tips = None  # (5, 3) wrist-relative tip positions at cal
        self._scale = np.ones(5)     # per-finger scale

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)
        self._last_errors = np.zeros(5)

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Record open-hand reference + compute scale."""
        max_idx = max(MANUS_TIP_INDICES)
        if len(skeleton) <= max_idx:
            print(f"[2A-Cal] WARNING: skeleton {len(skeleton)} nodes < {max_idx+1}")
            return

        wrist = skeleton[0, :3]
        self._human_cal_tips = np.array([
            skeleton[idx, :3] - wrist for idx in MANUS_TIP_INDICES
        ])

        human_dists = np.array([np.linalg.norm(t) for t in self._human_cal_tips])
        self._scale = self._robot_tip_dists / np.maximum(human_dists, 1e-6)
        self._is_calibrated = True

        print(f"[2A-Cal] Scale: {[f'{s:.2f}' for s in self._scale]}")
        print(f"[2A-Cal] Human dists: {[f'{d:.4f}' for d in human_dists]}")
        print(f"[2A-Cal] Robot dists: {[f'{d:.4f}' for d in self._robot_tip_dists]}")

    def retarget(self, skeleton: np.ndarray = None,
                 ergonomics: np.ndarray = None, **kwargs) -> np.ndarray:
        if skeleton is None or ergonomics is None:
            return self._q_prev.copy()

        # Auto-calibrate from first valid frame
        if not self._is_calibrated:
            if len(skeleton) > max(MANUS_TIP_INDICES):
                print("[2A] Auto-calibrating from first frame...")
                self.calibrate(skeleton=skeleton)

        if not self._is_calibrated:
            return self._q_prev.copy()

        wrist = skeleton[0, :3]
        q = self._q_prev.copy()

        for f in range(5):
            tip_idx = MANUS_TIP_INDICES[f]
            if tip_idx >= len(skeleton):
                continue

            # 1. Current tip: wrist-relative
            p_human_now = skeleton[tip_idx, :3] - wrist

            # 2. RELATIVE MOTION: delta from calibration pose, scaled
            delta = (p_human_now - self._human_cal_tips[f]) * self._scale[f]
            p_target = self._robot_tips_q0[f] + delta

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

            p_achieved = self._fk.finger_tip_position(q, f)
            self._last_errors[f] = np.linalg.norm(p_target - p_achieved)

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
