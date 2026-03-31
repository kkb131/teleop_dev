"""4-level safety monitor for teleop servo control."""

import time
from dataclasses import dataclass

import numpy as np

from robot.core.robot_backend import RobotBackend
from robot.arm.admittance.teleop_config import SafetyConfig


@dataclass
class SafetyResult:
    is_safe: bool
    q_safe: np.ndarray
    level: str = "OK"  # "OK", "TIMEOUT", "VEL_LIMIT", "WS_CLAMP", "ESTOP"
    message: str = ""  # detail message for display


class SafetyMonitor:
    """4-level safety system for teleop.

    Level 1: Packet timeout -- stop if no input for too long
    Level 2: Velocity limiting -- scale joint velocities to stay within limits
    Level 3: Workspace clamping -- clamp EE position to workspace bounds
    Level 4: E-Stop -- immediate stop, requires manual reset

    All status messages are returned via SafetyResult/properties, not printed.
    """

    def __init__(self, config: SafetyConfig, backend: RobotBackend):
        self._config = config
        self._backend = backend
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
        if not self._estop_active:
            self._backend.emergency_stop()
            self._estop_active = True

    def reset_estop(self):
        """Reset e-stop. Requires explicit operator action."""
        if self._estop_active:
            self._estop_active = False

    def clamp_workspace(self, ee_pos: np.ndarray) -> np.ndarray:
        """Level 3: clamp EE position to workspace bounds.

        Returns clamped position. Does NOT reject -- just constrains.
        """
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

    def check_and_apply(
        self, q_target: np.ndarray, q_current: np.ndarray, dt: float
    ) -> SafetyResult:
        """Run safety checks (levels 1, 2, 4).

        Level 3 (workspace) should be called separately before IK.
        """
        # Level 4: E-Stop (highest priority)
        if self._estop_active:
            return SafetyResult(
                is_safe=False, q_safe=q_current.copy(),
                level="ESTOP", message="E-STOP ACTIVE"
            )

        # Level 1: Packet timeout
        elapsed_ms = self.time_since_input_ms
        if elapsed_ms > self._config.packet_timeout_ms:
            if not self._timeout_active:
                self._timeout_active = True
            return SafetyResult(
                is_safe=False, q_safe=q_current.copy(),
                level="TIMEOUT", message=f"No input {elapsed_ms:.0f}ms"
            )

        # Level 2: Velocity limiting
        q_safe = q_target.copy()
        joint_vel = (q_target - q_current) / dt
        max_vel = np.max(np.abs(joint_vel))
        msg = ""

        if max_vel > self._config.max_joint_vel:
            scale = self._config.max_joint_vel / max_vel
            q_safe = q_current + (q_target - q_current) * scale
            msg = f"vel scaled {scale:.2f}x"

        level = "OK"
        if self._ws_clamped:
            level = "WS_CLAMP"
            msg = "workspace boundary" + (f" + {msg}" if msg else "")
        elif msg:
            level = "VEL_LIMIT"

        return SafetyResult(is_safe=True, q_safe=q_safe, level=level, message=msg)

    def get_status_string(self) -> str:
        """Return a human-readable status string for display."""
        if self._estop_active:
            return "!! E-STOP !!"
        if self._timeout_active:
            return f"TIMEOUT ({self.time_since_input_ms:.0f}ms)"
        parts = ["OK"]
        if self._ws_clamped:
            parts = ["WS_CLAMP"]
        return " | ".join(parts)
