"""[2A] Manus Skeleton → Fingertip IK → DG5F.

Per-finger DLS IK with Ergonomics-guided redundancy resolution.
Abduction fixed from Manus Ergonomics, 3-DOF IK for MCP/PIP/DIP.

Coordinate frame strategy:
    ALL IK is done in DG5F URDF world frame (origin = wrist mount = [0,0,0]).
    Manus skeleton is transformed: wrist-relative → SVD rotate → scale.
    No palm offset needed since URDF origin IS the wrist.

Pipeline:
    skeleton (N, 7) + ergonomics (20,)
    → extract fingertip positions (wrist-relative)
    → SVD rotate (Manus frame → DG5F frame)
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


def _compute_procrustes(src: np.ndarray, dst: np.ndarray):
    """Compute rotation R that best aligns src→dst (no reflection).

    src, dst: (N, 3) corresponding point sets.
    Returns R (3,3) with det(R) = +1 (pure rotation, no reflection).
    """
    src_c = src - src.mean(axis=0)
    dst_c = dst - dst.mean(axis=0)
    H = src_c.T @ dst_c
    U, S, Vt = np.linalg.svd(H)
    # Force det(R) = +1 to prevent reflection
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, np.sign(d)])
    R = Vt.T @ D @ U.T
    return R


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
        self._scale = np.ones(5)
        self._is_calibrated = False

        # SVD rotation: Manus wrist-relative → DG5F wrist-relative
        self._R = np.eye(3)

        # DG5F fingertip positions at q=0 (wrist frame) — used for scale ref
        self._robot_tips_at_zero = self._fk.fingertip_positions(np.zeros(NUM_JOINTS))

        self._q_prev = np.zeros(NUM_JOINTS)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)

        # Debug
        self._last_targets = np.zeros((5, 3))
        self._last_achieved = np.zeros((5, 3))
        self._last_errors = np.zeros(5)

    def calibrate(self, skeleton: np.ndarray, **kwargs):
        """Open-hand calibration.

        1. Compute per-finger bone length scale
        2. Compute SVD rotation (Manus → DG5F wrist frame)
        """
        # Validate skeleton
        max_idx = max(MANUS_TIP_INDICES)
        if len(skeleton) <= max_idx:
            print(f"[2A-Cal] WARNING: skeleton has {len(skeleton)} nodes, "
                  f"need {max_idx + 1}. Using identity rotation.")
            self._R = np.eye(3)
            self._is_calibrated = False
            return

        wrist = skeleton[0, :3]

        # 1. Scale: wrist→tip distance ratio (human vs robot)
        human_tips = np.array([skeleton[idx, :3] - wrist for idx in MANUS_TIP_INDICES])
        human_dists = np.array([np.linalg.norm(h) for h in human_tips])
        robot_dists = np.array([np.linalg.norm(self._robot_tips_at_zero[i])
                                for i in range(5)])
        self._scale = robot_dists / np.maximum(human_dists, 1e-6)

        # 2. SVD rotation: align Manus wrist-relative tips → DG5F wrist-relative tips
        # Both are wrist-relative (origin = wrist = [0,0,0])
        self._R = _compute_procrustes(human_tips, self._robot_tips_at_zero)
        self._is_calibrated = True

        det_r = np.linalg.det(self._R)
        print(f"[2A-Cal] Scale: {[f'{s:.3f}' for s in self._scale]}")
        print(f"[2A-Cal] Rotation det={det_r:.4f} (should be +1.0)")
        print(f"[2A-Cal] Human tip dists (m): {[f'{d:.4f}' for d in human_dists]}")
        print(f"[2A-Cal] Robot tip dists (m): {[f'{d:.4f}' for d in robot_dists]}")

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

        # Auto-calibrate from first valid skeleton (prevents fist default)
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

            # 1. Human tip: wrist-relative (Manus frame)
            p_human_local = skeleton[tip_idx, :3] - wrist

            # 2. Transform to DG5F wrist frame:
            #    rotate (Manus→DG5F) + scale (human→robot)
            #    NO offset needed — URDF origin = wrist = [0,0,0]
            p_target = self._R @ p_human_local * self._scale[f]
            self._last_targets[f] = p_target

            # 3. Abduction from Ergonomics
            abd_angle = ergonomics[f * 4]

            # 4. Per-finger IK (target in DG5F wrist frame = FK frame)
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
