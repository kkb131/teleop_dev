"""Safety monitor specialized for torque-mode impedance teleop.

Levels (priority high → low):
  4: E-Stop       — immediate stop, manual reset required
  3: Position deviation — q_desired vs q_actual too large
  2: Velocity limit — joint velocity exceeds threshold
  1: Communication timeout — no input for too long
  0: Workspace clamping — clamp EE position to bounds
"""

import time
from dataclasses import dataclass

import numpy as np

from teleop_dev.robot.arm.impedance.impedance_config import SafetyConfig


@dataclass
class TorqueSafetyResult:
    is_safe: bool
    level: str = "OK"  # "OK", "ESTOP", "DEVIATION", "VEL_LIMIT", "TIMEOUT", "WS_CLAMP"
    message: str = ""


class TorqueSafetyMonitor:
    """Safety monitor for torque-mode teleop.

    Unlike position-mode safety, this monitor:
    - Checks position deviation (q_desired vs q_actual) as a critical safety signal
    - Uses tighter timeouts (torque mode is inherently riskier)
    - Does not scale velocities (that's handled by PD gains in URScript)
    """

    def __init__(self, config: SafetyConfig):
        self._config = config
        self._estop_active = False
        self._last_input_time = time.monotonic()
        self._timeout_active = False
        self._ws_clamped = False

    @property
    def estop_active(self) -> bool:
        return self._estop_active

    @property
    def timeout_active(self) -> bool:
        return self._timeout_active

    @property
    def ws_clamped(self) -> bool:
        return self._ws_clamped

    @property
    def time_since_input_ms(self) -> float:
        return (time.monotonic() - self._last_input_time) * 1000.0

    def update_input_timestamp(self):
        """Call when valid input is received."""
        self._last_input_time = time.monotonic()
        self._timeout_active = False

    def trigger_estop(self):
        """Level 4: immediate emergency stop."""
        self._estop_active = True

    def reset_estop(self):
        """Reset e-stop. Requires explicit operator action."""
        self._estop_active = False

    def clamp_workspace(self, ee_pos: np.ndarray) -> np.ndarray:
        """Level 0: clamp EE position to workspace bounds."""
        ws = self._config.workspace
        clamped = ee_pos.copy()
        self._ws_clamped = False

        for i, (lo, hi) in enumerate([(ws.x[0], ws.x[1]),
                                       (ws.y[0], ws.y[1]),
                                       (ws.z[0], ws.z[1])]):
            if clamped[i] < lo:
                clamped[i] = lo
                self._ws_clamped = True
            elif clamped[i] > hi:
                clamped[i] = hi
                self._ws_clamped = True

        return clamped

    def check(
        self,
        q_desired: np.ndarray,
        q_actual: np.ndarray,
        qd_actual: np.ndarray,
    ) -> TorqueSafetyResult:
        """Run safety checks (levels 1-4).

        Level 0 (workspace) should be called separately via clamp_workspace().

        Returns:
            TorqueSafetyResult with is_safe flag and diagnostic info.
        """
        # Level 4: E-Stop (highest priority)
        if self._estop_active:
            return TorqueSafetyResult(
                is_safe=False, level="ESTOP", message="E-STOP ACTIVE"
            )

        # Level 1: Communication timeout
        elapsed_ms = self.time_since_input_ms
        if elapsed_ms > self._config.packet_timeout_ms:
            self._timeout_active = True
            return TorqueSafetyResult(
                is_safe=False, level="TIMEOUT",
                message=f"No input {elapsed_ms:.0f}ms",
            )

        # Level 3: Position deviation check
        deviation = np.abs(q_desired - q_actual)
        max_dev = np.max(deviation)
        if max_dev > self._config.max_position_deviation:
            joint_idx = np.argmax(deviation)
            return TorqueSafetyResult(
                is_safe=False, level="DEVIATION",
                message=f"J{joint_idx+1} dev={np.degrees(max_dev):.1f}deg",
            )

        # Level 2: Velocity limit
        max_vel = np.max(np.abs(qd_actual))
        if max_vel > self._config.max_joint_vel:
            joint_idx = np.argmax(np.abs(qd_actual))
            return TorqueSafetyResult(
                is_safe=False, level="VEL_LIMIT",
                message=f"J{joint_idx+1} vel={np.degrees(max_vel):.1f}deg/s",
            )

        # All OK
        level = "WS_CLAMP" if self._ws_clamped else "OK"
        msg = "workspace boundary" if self._ws_clamped else ""
        return TorqueSafetyResult(is_safe=True, level=level, message=msg)
