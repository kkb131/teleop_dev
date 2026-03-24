"""Joint-space PD impedance gain presets and runtime tuning for UR10e."""

from dataclasses import dataclass, field
from typing import Dict

import numpy as np


@dataclass
class ImpedanceGains:
    """Joint-space PD gains for impedance control.

    Kp: position stiffness [Nm/rad] per joint
    Kd: velocity damping [Nm·s/rad] per joint
    """
    Kp: np.ndarray = field(default_factory=lambda: np.zeros(6))
    Kd: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def __post_init__(self):
        self.Kp = np.asarray(self.Kp, dtype=float)
        self.Kd = np.asarray(self.Kd, dtype=float)

    def scaled(self, factor: float) -> "ImpedanceGains":
        """Return a copy with gains scaled by factor."""
        return ImpedanceGains(Kp=self.Kp * factor, Kd=self.Kd * factor)


# UR10e joint-specific defaults:
#   joints 0-1 (shoulder): high inertia → high gains
#   joints 2-3 (elbow/wrist1): medium inertia
#   joints 4-5 (wrist2/wrist3): low inertia → low gains
# Damping ratio ≈ 0.7–1.0 (critically damped to overdamped)
IMPEDANCE_PRESETS: Dict[str, ImpedanceGains] = {
    "STIFF": ImpedanceGains(
        Kp=np.array([800.0, 800.0, 400.0, 200.0, 100.0, 50.0]),
        Kd=np.array([80.0, 80.0, 40.0, 20.0, 10.0, 5.0]),
    ),
    "MEDIUM": ImpedanceGains(
        Kp=np.array([600.0, 600.0, 300.0, 150.0, 75.0, 37.5]),
        Kd=np.array([64.0, 64.0, 32.0, 16.0, 8.0, 4.0]),
    ),
    "SOFT": ImpedanceGains(
        Kp=np.array([280.0, 280.0, 140.0, 70.0, 35.0, 17.5]),
        Kd=np.array([36.0, 36.0, 18.0, 9.0, 4.5, 2.25]),
    ),
    "COMPLIANT": ImpedanceGains(
        Kp=np.array([20.0, 20.0, 10.0, 5.0, 2.5, 1.25]),
        Kd=np.array([8.0, 8.0, 4.0, 2.0, 1.0, 0.5]),
    ),
    "GRAVITY_COMP": ImpedanceGains(
        Kp=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        Kd=np.array([2.0, 2.0, 1.0, 0.5, 0.25, 0.125]),
    ),
}

DEFAULT_IMPEDANCE_PRESET = "SOFT"

# Runtime gain scale bounds
GAIN_SCALE_MIN = 0.25
GAIN_SCALE_MAX = 2.0
GAIN_SCALE_STEP = 0.25


class ImpedanceController:
    """Manages impedance gain selection and runtime tuning."""

    def __init__(self, preset: str = DEFAULT_IMPEDANCE_PRESET):
        if preset not in IMPEDANCE_PRESETS:
            preset = DEFAULT_IMPEDANCE_PRESET
        self._base_gains = IMPEDANCE_PRESETS[preset]
        self._preset_name = preset
        self._scale = 1.0

    @property
    def preset_name(self) -> str:
        return self._preset_name

    @property
    def scale(self) -> float:
        return self._scale

    @property
    def gains(self) -> ImpedanceGains:
        """Current gains (base × scale)."""
        return self._base_gains.scaled(self._scale)

    @property
    def Kp(self) -> np.ndarray:
        return self._base_gains.Kp * self._scale

    @property
    def Kd(self) -> np.ndarray:
        return self._base_gains.Kd * self._scale

    def set_preset(self, name: str):
        """Switch to a named preset, reset scale to 1.0."""
        if name in IMPEDANCE_PRESETS:
            self._base_gains = IMPEDANCE_PRESETS[name]
            self._preset_name = name
            self._scale = 1.0

    def scale_up(self):
        """Increase gain scale by one step."""
        self._scale = min(self._scale + GAIN_SCALE_STEP, GAIN_SCALE_MAX)

    def scale_down(self):
        """Decrease gain scale by one step."""
        self._scale = max(self._scale - GAIN_SCALE_STEP, GAIN_SCALE_MIN)

    _CYCLE_PRESETS = ["STIFF", "MEDIUM", "SOFT"]

    def cycle_preset(self):
        """Cycle: STIFF → MEDIUM → SOFT → STIFF."""
        try:
            idx = self._CYCLE_PRESETS.index(self._preset_name)
            next_idx = (idx + 1) % len(self._CYCLE_PRESETS)
        except ValueError:
            next_idx = 0
        self.set_preset(self._CYCLE_PRESETS[next_idx])
