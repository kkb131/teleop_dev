#!/usr/bin/env python3
"""Vector-based Manus→DG5F retargeting via optimization.

Finds DG5F joint angles that make robot fingertip direction vectors
match human fingertip direction vectors. Scale-invariant.

Usage:
    from sender.hand.vector_retarget import VectorRetarget
    rt = VectorRetarget(hand_side="right")
    rt.calibrate_baseline(manus_open_angles)  # once, when hand is flat
    dg5f_q = rt.retarget(manus_angles)        # every frame
"""

import numpy as np
from scipy.optimize import minimize

from sender.hand.hand_fk import HumanHandFK
from sender.hand.dg5f_fk import DG5FKinematics


class VectorRetarget:
    """Vector optimization retargeting from Manus to DG5F.

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
        self._human_fk = HumanHandFK()
        self._robot_fk = DG5FKinematics(hand_side=hand_side, urdf_path=urdf_path)
        self._reg_weight = reg_weight

        # Joint limits
        self._q_min = self._robot_fk.q_min
        self._q_max = self._robot_fk.q_max
        self._bounds = list(zip(self._q_min, self._q_max))

        # Warm-start: previous solution
        self._q_prev = np.zeros(20)

        # Per-finger weights (can tune: thumb often needs more weight)
        self._finger_weights = np.array([1.5, 1.0, 1.0, 1.0, 0.8])

    def calibrate_baseline(self, manus_open_angles: np.ndarray):
        """Set open-hand baseline for human FK.

        Call once when user has hand flat with fingers together.
        This becomes the zero-reference for the human hand model.
        """
        self._human_fk.set_baseline(manus_open_angles)
        print(f"[VectorRetarget] Baseline set from {len(manus_open_angles)} joint angles")

    def retarget(self, manus_angles: np.ndarray) -> np.ndarray:
        """Compute DG5F joint angles from Manus joint angles.

        Parameters
        ----------
        manus_angles : ndarray[20]
            Manus ergonomics joint angles (radians).

        Returns
        -------
        ndarray[20]
            DG5F joint angles (radians), clamped to limits.
        """
        # Human fingertip vectors
        human_vecs = self._human_fk.fingertip_vectors(manus_angles)

        # Objective: minimize vector direction error + regularization
        def cost(q):
            robot_vecs = self._robot_fk.fingertip_vectors(q)
            # Direction error per finger (weighted)
            err = 0.0
            for i in range(5):
                diff = robot_vecs[i] - human_vecs[i]
                err += self._finger_weights[i] * np.dot(diff, diff)
            # Regularization toward previous solution
            delta = q - self._q_prev
            reg = self._reg_weight * np.dot(delta, delta)
            return err + reg

        # Optimize (warm-start from previous solution)
        result = minimize(
            cost,
            self._q_prev,
            method='SLSQP',
            bounds=self._bounds,
            options={'maxiter': 50, 'ftol': 1e-6},
        )

        q_opt = result.x
        # Clamp to limits (redundant with bounds, but safe)
        q_opt = np.clip(q_opt, self._q_min, self._q_max)

        self._q_prev = q_opt.copy()
        return q_opt

    def get_debug_info(self, manus_angles: np.ndarray, dg5f_q: np.ndarray) -> dict:
        """Return debug info for visualization/diagnostics."""
        human_vecs = self._human_fk.fingertip_vectors(manus_angles)
        robot_vecs = self._robot_fk.fingertip_vectors(dg5f_q)

        errors = []
        for i in range(5):
            cos_sim = np.dot(human_vecs[i], robot_vecs[i])
            errors.append(float(np.degrees(np.arccos(np.clip(cos_sim, -1, 1)))))

        return {
            "human_vectors": human_vecs.tolist(),
            "robot_vectors": robot_vecs.tolist(),
            "angle_errors_deg": errors,
            "mean_error_deg": float(np.mean(errors)),
        }
