"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.

Coordinate strategy (853c174 — validated on real hardware):
    p_target = p_human_wrist_relative * 1.0 + palm_offset
    R = identity, scale = 1. No calibration needed.
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

        # 853c174 strategy: R=I, scale=1, +palm_offset
        self._scale = np.ones(5)
        self._palm_offset = self._fk.palm_position(np.zeros(NUM_JOINTS))
        self._is_calibrated = False  # stays False unless --calibrate

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)
        self._last_errors = np.zeros(5)
        self._frame_count = 0

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Optional: compute per-finger scale from skeleton."""
        max_idx = max(MANUS_TIP_INDICES)
        if len(skeleton) <= max_idx:
            print(f"[2A-Cal] WARNING: skeleton {len(skeleton)} nodes < {max_idx+1}")
            return

        wrist = skeleton[0, :3]
        robot_tips = self._fk.fingertip_positions(np.zeros(NUM_JOINTS))

        for i, idx in enumerate(MANUS_TIP_INDICES):
            h_dist = np.linalg.norm(skeleton[idx, :3] - wrist)
            r_dist = np.linalg.norm(robot_tips[i])
            if h_dist > 0.01:
                self._scale[i] = r_dist / h_dist
            print(f"  [{['Thumb','Index','Middle','Ring','Pinky'][i]}] "
                  f"human={h_dist:.4f}m robot={r_dist:.4f}m scale={self._scale[i]:.3f}")

        self._is_calibrated = True
        print(f"[2A-Cal] Scale: {[f'{s:.2f}' for s in self._scale]}")

    def retarget(self, skeleton: np.ndarray = None,
                 ergonomics: np.ndarray = None, **kwargs) -> np.ndarray:
        if skeleton is None or ergonomics is None:
            return self._q_prev.copy()

        wrist = skeleton[0, :3]
        q = self._q_prev.copy()
        self._frame_count += 1

        for f in range(5):
            tip_idx = MANUS_TIP_INDICES[f]
            if tip_idx >= len(skeleton):
                continue

            # 853c174 formula: p_human_local * scale + palm_offset
            p_human_local = skeleton[tip_idx, :3] - wrist
            p_target = p_human_local * self._scale[f] + self._palm_offset

            # Abduction from Ergonomics
            abd_angle = ergonomics[f * 4]

            # Per-finger IK
            q_finger = self._solvers[f].solve(
                p_target=p_target,
                abd_angle=abd_angle,
                q_init=q[f * 4 + 1: f * 4 + 4],
                q_full=q,
            )
            q[f * 4: f * 4 + 4] = q_finger

            p_achieved = self._fk.finger_tip_position(q, f)
            self._last_errors[f] = np.linalg.norm(p_target - p_achieved)

            # Debug: first 3 frames, print per-finger info
            if self._frame_count <= 3:
                name = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky'][f]
                print(f"[2A-dbg] frame={self._frame_count} {name}: "
                      f"human_local=[{p_human_local[0]:+.4f},{p_human_local[1]:+.4f},{p_human_local[2]:+.4f}] "
                      f"target=[{p_target[0]:+.4f},{p_target[1]:+.4f},{p_target[2]:+.4f}] "
                      f"achieved=[{p_achieved[0]:+.4f},{p_achieved[1]:+.4f},{p_achieved[2]:+.4f}] "
                      f"err={self._last_errors[f]*1000:.1f}mm "
                      f"q=[{np.degrees(q_finger[0]):+.0f},{np.degrees(q_finger[1]):+.0f},"
                      f"{np.degrees(q_finger[2]):+.0f},{np.degrees(q_finger[3]):+.0f}]°")

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
