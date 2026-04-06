#!/usr/bin/env python3
"""Vector-based Manus→DG5F retargeting via optimization.

Uses Manus SDK Raw Skeleton 3D positions (not manual FK) for human
fingertip vectors. Finds DG5F joint angles that make robot fingertip
direction vectors match human fingertip direction vectors.

Usage:
    from sender.hand.vector_retarget import VectorRetarget
    rt = VectorRetarget(hand_side="right")
    dg5f_q = rt.retarget(skeleton)  # skeleton: ndarray[N, 7]
"""

import numpy as np
from scipy.optimize import minimize

from sender.hand.dg5f_fk import DG5FKinematics

# Skeleton node indices (Manus Raw Skeleton convention)
# [0]=wrist, [1-4]=thumb(CMC,MCP,IP,TIP), [5-8]=index, [9-12]=middle,
# [13-16]=ring, [17-20]=pinky
WRIST_IDX = 0
FINGERTIP_INDICES = [4, 8, 12, 16, 20]  # thumb, index, middle, ring, pinky
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]


def skeleton_to_fingertip_vectors(skeleton: np.ndarray) -> np.ndarray:
    """Extract palm→fingertip unit direction vectors from skeleton.

    Parameters
    ----------
    skeleton : ndarray[N, 7]
        Per-node: [x, y, z, qw, qx, qy, qz].

    Returns
    -------
    ndarray[5, 3]
        Normalized direction vectors from wrist to each fingertip.
    """
    wrist_pos = skeleton[WRIST_IDX, :3]
    vectors = np.zeros((5, 3))
    for i, tip_idx in enumerate(FINGERTIP_INDICES):
        if tip_idx < len(skeleton):
            v = skeleton[tip_idx, :3] - wrist_pos
            norm = np.linalg.norm(v)
            if norm > 1e-6:
                vectors[i] = v / norm
            else:
                vectors[i] = v
    return vectors


class VectorRetarget:
    """Vector optimization retargeting from Manus skeleton to DG5F.

    Uses Manus SDK Raw Skeleton 3D positions for human hand,
    Pinocchio FK for DG5F robot hand.

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    urdf_path : str or None
        Override DG5F URDF path.
    reg_weight : float
        Regularization weight toward previous solution (temporal smoothness).
    """

    def __init__(self, hand_side: str = "right", urdf_path: str = None,
                 reg_weight: float = 0.01):
        self._side = hand_side
        self._robot_fk = DG5FKinematics(hand_side=hand_side, urdf_path=urdf_path)
        self._reg_weight = reg_weight

        # Joint limits
        self._q_min = self._robot_fk.q_min
        self._q_max = self._robot_fk.q_max
        self._bounds = list(zip(self._q_min, self._q_max))

        # Warm-start: previous solution
        self._q_prev = np.zeros(20)

        # Per-finger weights (thumb needs more weight for accurate opposition)
        self._finger_weights = np.array([1.5, 1.0, 1.0, 1.0, 0.8])

        # Baseline skeleton (open hand) for offset subtraction
        self._baseline_vectors = None

    def calibrate_baseline(self, skeleton: np.ndarray):
        """Record open-hand baseline from skeleton.

        Call once when user has hand flat with fingers together.
        """
        self._baseline_vectors = skeleton_to_fingertip_vectors(skeleton)
        print(f"[VectorRetarget] Baseline set from skeleton ({len(skeleton)} nodes)")

    def retarget(self, skeleton: np.ndarray) -> np.ndarray:
        """Compute DG5F joint angles from Manus skeleton.

        Parameters
        ----------
        skeleton : ndarray[N, 7]
            Raw skeleton nodes: [x, y, z, qw, qx, qy, qz] per node.

        Returns
        -------
        ndarray[20]
            DG5F joint angles (radians), clamped to limits.
        """
        human_vecs = skeleton_to_fingertip_vectors(skeleton)

        def cost(q):
            robot_vecs = self._robot_fk.fingertip_vectors(q)
            err = 0.0
            for i in range(5):
                diff = robot_vecs[i] - human_vecs[i]
                err += self._finger_weights[i] * np.dot(diff, diff)
            delta = q - self._q_prev
            reg = self._reg_weight * np.dot(delta, delta)
            return err + reg

        result = minimize(
            cost,
            self._q_prev,
            method='SLSQP',
            bounds=self._bounds,
            options={'maxiter': 50, 'ftol': 1e-6},
        )

        q_opt = np.clip(result.x, self._q_min, self._q_max)
        self._q_prev = q_opt.copy()
        return q_opt

    def get_debug_info(self, skeleton: np.ndarray, dg5f_q: np.ndarray) -> dict:
        """Return debug info for diagnostics."""
        human_vecs = skeleton_to_fingertip_vectors(skeleton)
        robot_vecs = self._robot_fk.fingertip_vectors(dg5f_q)

        errors = []
        for i in range(5):
            cos_sim = np.dot(human_vecs[i], robot_vecs[i])
            errors.append(float(np.degrees(np.arccos(np.clip(cos_sim, -1, 1)))))

        return {
            "finger_names": FINGER_NAMES,
            "human_vectors": human_vecs.tolist(),
            "robot_vectors": robot_vecs.tolist(),
            "angle_errors_deg": errors,
            "mean_error_deg": float(np.mean(errors)),
        }
