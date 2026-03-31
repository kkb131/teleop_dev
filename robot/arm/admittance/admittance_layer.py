"""Admittance control layer for the teleop pipeline.

Wraps core AdmittanceController + FTSource for use in the teleop pipeline.
Manages toggle on/off, preset switching, and sensor zeroing.

Note: UR getActualTCPForce() returns wrench in the TCP frame.
BaseFrameFTSource handles TCP→base conversion by negating X,Y components.
"""

import numpy as np

from robot.core.compliant_control import (
    AdmittanceController,
    ComplianceParams,
    COMPLIANCE_PRESETS,
    DEFAULT_PRESET,
)
from robot.core.ft_source import FTSource, NullFTSource, RTDEFTSource, BaseFrameFTSource
from robot.core.robot_backend import RobotBackend
from robot.arm.admittance.teleop_config import AdmittanceConfig


class AdmittanceLayer:
    """Teleop pipeline integration for admittance control.

    Reads F/T sensor, runs admittance dynamics, and returns a Cartesian
    displacement to add to the target pose.
    """

    def __init__(
        self,
        config: AdmittanceConfig,
        backend: RobotBackend,
        mode: str,
    ):
        self._config = config

        # Create F/T source based on backend mode
        # RTDEFTSource returns wrench in TCP frame; BaseFrameFTSource
        # wraps it to negate X,Y for base frame conversion.
        if mode == "rtde":
            raw_ft = RTDEFTSource(backend)
            self._ft_source: FTSource = BaseFrameFTSource(raw_ft)
        else:
            self._ft_source = NullFTSource()

        # Create admittance controller
        preset_name = config.default_preset
        if preset_name not in COMPLIANCE_PRESETS:
            preset_name = DEFAULT_PRESET
        self._preset_name = preset_name

        self._controller = AdmittanceController(
            params=COMPLIANCE_PRESETS[self._preset_name],
            max_disp_trans=config.max_displacement_trans,
            max_disp_rot=config.max_displacement_rot,
            force_deadzone=np.array(config.force_deadzone),
            force_saturation=config.force_saturation,
            torque_saturation=config.torque_saturation,
        )

        # Enabled state
        self._enabled = config.enabled_by_default
        # In sim mode, admittance has no effect (NullFTSource returns zeros)
        self._has_sensor = mode == "rtde"

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def has_sensor(self) -> bool:
        return self._has_sensor

    @property
    def preset_name(self) -> str:
        return self._preset_name

    def compute_displacement(self, q: np.ndarray, dt: float) -> np.ndarray:
        """Compute admittance displacement in base frame.

        Args:
            q: Current joint positions (unused, kept for API compat).
            dt: Time step.

        Returns:
            Displacement (6,): [dx, dy, dz, drx, dry, drz] in base frame.
            Returns zeros if disabled or no sensor.
        """
        if not self._enabled:
            return np.zeros(6)

        # FTSource returns wrench in base frame (BaseFrameFTSource handles
        # TCP→base rotation; NullFTSource returns zeros)
        wrench_base = self._ft_source.get_wrench()

        return self._controller.update(wrench_base, dt)

    def toggle(self) -> bool:
        """Toggle admittance on/off. Returns new enabled state."""
        if not self._has_sensor:
            return False
        self._enabled = not self._enabled
        if not self._enabled:
            self._controller.reset()
        return self._enabled

    def set_preset(self, name: str):
        """Switch compliance preset."""
        if name in COMPLIANCE_PRESETS:
            self._preset_name = name
            self._controller.set_params(COMPLIANCE_PRESETS[name])

    _CYCLE_PRESETS = ["STIFF", "MEDIUM", "SOFT"]

    def cycle_preset(self):
        """Cycle through presets: STIFF → MEDIUM → SOFT → STIFF."""
        try:
            idx = self._CYCLE_PRESETS.index(self._preset_name)
            next_idx = (idx + 1) % len(self._CYCLE_PRESETS)
        except ValueError:
            next_idx = 0
        self.set_preset(self._CYCLE_PRESETS[next_idx])

    def zero_sensor(self):
        """Zero the F/T sensor and reset admittance state."""
        self._ft_source.zero_sensor()
        self._controller.reset()

    def reset(self):
        """Reset admittance state (e.g., after e-stop)."""
        self._controller.reset()

    def get_wrench(self) -> np.ndarray:
        """Get current raw wrench (for display)."""
        return self._ft_source.get_wrench()

    @property
    def displacement(self) -> np.ndarray:
        """Current admittance displacement (for display)."""
        return self._controller.displacement
