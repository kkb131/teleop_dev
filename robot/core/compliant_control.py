"""Compliant control dynamics (admittance, future: impedance).

Provides reusable force-based control strategies:
  - AdmittanceController: M*xddot + D*xdot + K*x = f_ext
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass
class ComplianceParams:
    """Mass-Spring-Damper parameters for 6-DOF compliant control."""
    M: np.ndarray  # (6,) virtual mass [kg, kg, kg, kg*m^2, ...]
    D: np.ndarray  # (6,) damping
    K: np.ndarray  # (6,) stiffness


COMPLIANCE_PRESETS: Dict[str, ComplianceParams] = {
    "STIFF": ComplianceParams(
        M=np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]),
        D=np.array([200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),
        K=np.array([500.0, 500.0, 500.0, 50.0, 50.0, 50.0]),
    ),
    "MEDIUM": ComplianceParams(
        # Translation: heavier mass + higher damping → less responsive to push
        # Rotation: lighter mass + lower damping → more responsive to torque
        M=np.array([8.0, 8.0, 8.0, 0.3, 0.3, 0.3]),
        D=np.array([160.0, 160.0, 160.0, 6.0, 6.0, 6.0]),
        K=np.array([200.0, 200.0, 200.0, 15.0, 15.0, 15.0]),
    ),
    "SOFT": ComplianceParams(
        M=np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2]),
        D=np.array([40.0, 40.0, 40.0, 4.0, 4.0, 4.0]),
        K=np.array([50.0, 50.0, 50.0, 5.0, 5.0, 5.0]),
    ),
    "FREE": ComplianceParams(
        M=np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2]),
        D=np.array([30.0, 30.0, 30.0, 3.0, 3.0, 3.0]),
        K=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ),
}

DEFAULT_PRESET = "MEDIUM"


class CompliantController(ABC):
    """Base class for force-based compliant control strategies."""

    @abstractmethod
    def update(self, f_ext: np.ndarray, dt: float) -> np.ndarray:
        """Given external wrench in base frame, return Cartesian displacement (6,)."""

    @abstractmethod
    def reset(self):
        """Zero internal state (displacement, velocity)."""

    @abstractmethod
    def set_params(self, params: ComplianceParams):
        """Update compliance parameters."""


class AdmittanceController(CompliantController):
    """Second-order admittance: M*xddot + D*xdot + K*x = f_ext.

    Computes Cartesian displacement from external wrench using Euler integration.
    Output is a displacement vector suitable for adding to a target pose.
    """

    def __init__(
        self,
        params: ComplianceParams,
        max_disp_trans: float = 0.15,
        max_disp_rot: float = 0.3,
        force_deadzone: np.ndarray = None,
        force_saturation: float = 100.0,
        torque_saturation: float = 10.0,
    ):
        self._params = params
        self._max_disp_trans = max_disp_trans
        self._max_disp_rot = max_disp_rot
        self._force_deadzone = (
            force_deadzone if force_deadzone is not None
            else np.array([3.0, 3.0, 3.0, 0.3, 0.3, 0.3])
        )
        self._force_saturation = force_saturation
        self._torque_saturation = torque_saturation

        # State
        self._x = np.zeros(6)      # displacement
        self._xdot = np.zeros(6)   # velocity

    @property
    def displacement(self) -> np.ndarray:
        return self._x.copy()

    @property
    def velocity(self) -> np.ndarray:
        return self._xdot.copy()

    def reset(self):
        self._x[:] = 0.0
        self._xdot[:] = 0.0

    def set_params(self, params: ComplianceParams):
        self._params = params
        self._xdot[:] = 0.0  # zero velocity only; keep displacement for smooth transition

    def update(self, f_ext: np.ndarray, dt: float) -> np.ndarray:
        """Compute admittance displacement from external wrench.

        Args:
            f_ext: External wrench [fx, fy, fz, tx, ty, tz] in base frame.
            dt: Time step (seconds).

        Returns:
            Displacement (6,): [dx, dy, dz, drx, dry, drz] in base frame.
        """
        # Saturation check — reset if exceeded
        force_mag = np.linalg.norm(f_ext[:3])
        torque_mag = np.linalg.norm(f_ext[3:])
        if force_mag > self._force_saturation or torque_mag > self._torque_saturation:
            self.reset()
            return self._x.copy()

        # Apply deadzone
        f = f_ext.copy()
        mask = np.abs(f) < self._force_deadzone
        f[mask] = 0.0
        f[~mask] -= np.sign(f[~mask]) * self._force_deadzone[~mask]

        # M*xddot + D*xdot + K*x = f_ext
        p = self._params
        xddot = (f - p.D * self._xdot - p.K * self._x) / p.M

        # Euler integration
        self._xdot += xddot * dt
        self._x += self._xdot * dt

        # Clamp translation displacement
        disp_norm = np.linalg.norm(self._x[:3])
        if disp_norm > self._max_disp_trans:
            self._x[:3] *= self._max_disp_trans / disp_norm
            self._xdot[:3] *= 0.5

        # Clamp rotation displacement
        rot_norm = np.linalg.norm(self._x[3:])
        if rot_norm > self._max_disp_rot:
            self._x[3:] *= self._max_disp_rot / rot_norm
            self._xdot[3:] *= 0.5

        return self._x.copy()
