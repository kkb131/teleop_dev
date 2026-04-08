"""Per-finger Damped Least Squares IK solver.

Solves 3-DOF IK (MCP, PIP, DIP) with abduction fixed from Ergonomics.
Uses DLS: Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ error
"""

import numpy as np

from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics


class PerFingerIK:
    """DLS IK for one DG5F finger (4-DOF, abd fixed → 3-DOF solve).

    Parameters
    ----------
    fk : DG5FKinematics
        FK model.
    finger_idx : int
        0=Thumb, 1=Index, 2=Middle, 3=Ring, 4=Pinky.
    damping : float
        DLS damping factor λ (0.01~0.1).
    max_iter : int
        Maximum IK iterations per call.
    tol : float
        Position error tolerance (meters).
    """

    def __init__(self, fk: DG5FKinematics, finger_idx: int,
                 damping: float = 0.05, max_iter: int = 10,
                 tol: float = 1e-4):
        self._fk = fk
        self._finger_idx = finger_idx
        self._damping = damping
        self._max_iter = max_iter
        self._tol = tol

        # Per-finger joint limits (4 joints)
        base = finger_idx * 4
        self._q_min = fk.q_min[base:base + 4]
        self._q_max = fk.q_max[base:base + 4]

    def solve(self, p_target: np.ndarray, abd_angle: float,
              q_init: np.ndarray = None, q_full: np.ndarray = None) -> np.ndarray:
        """Solve IK for one finger.

        Parameters
        ----------
        p_target : ndarray[3]
            Target fingertip position (world frame).
        abd_angle : float
            Abduction angle (radians) from Manus Ergonomics. Fixed during IK.
        q_init : ndarray[3] or None
            Initial guess for joints 2-4 (warm-start from previous frame).
        q_full : ndarray[20]
            Full joint state (needed for FK). Modified in-place for this finger.

        Returns
        -------
        ndarray[4] — [abd, joint2, joint3, joint4] in radians, clamped.
        """
        base = self._finger_idx * 4

        # Initialize q_full with abd fixed
        if q_full is None:
            q_full = np.zeros(20)
        q_full[base] = np.clip(abd_angle, self._q_min[0], self._q_max[0])

        if q_init is not None:
            q_full[base + 1:base + 4] = q_init

        for _ in range(self._max_iter):
            # Current fingertip position
            p_current = self._fk.finger_tip_position(q_full, self._finger_idx)
            error = p_target - p_current

            if np.linalg.norm(error) < self._tol:
                break

            # Jacobian: 3×4 → take columns 1:4 (skip abd column 0)
            J_full = self._fk.finger_jacobian(q_full, self._finger_idx)
            J = J_full[:, 1:]  # (3, 3)

            # DLS: Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ error
            JJt = J @ J.T + self._damping**2 * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, error)

            # Update joints 2-4, clamp
            q_full[base + 1:base + 4] += dq
            q_full[base + 1:base + 4] = np.clip(
                q_full[base + 1:base + 4],
                self._q_min[1:],
                self._q_max[1:]
            )

        return q_full[base:base + 4].copy()
