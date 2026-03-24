"""Common robot backend interface (ABC) and factory."""

from abc import ABC, abstractmethod
from typing import List


class RobotBackend(ABC):
    """Abstract base class for robot communication backends."""

    @abstractmethod
    def connect(self):
        """Establish connection to the robot."""

    @abstractmethod
    def disconnect(self):
        """Clean shutdown."""

    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        """Read current joint positions (radians)."""

    @abstractmethod
    def get_joint_velocities(self) -> List[float]:
        """Read current joint velocities (rad/s)."""

    @abstractmethod
    def send_joint_command(self, positions: List[float]):
        """Send joint position command to the robot."""

    def on_trajectory_done(self):
        """Called when trajectory streaming is complete. Override if needed."""

    def emergency_stop(self):
        """E-Stop. Override for hardware-specific behavior (default: no-op)."""

    def get_tcp_force(self) -> List[float]:
        """Read TCP force/torque [fx,fy,fz,tx,ty,tz] (N, Nm). Default: zeros."""
        return [0.0] * 6

    def speed_stop(self, deceleration: float = 2.0):
        """Decelerate to stop. Override for hardware-specific behavior (default: no-op)."""

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()


def create_backend(mode: str, **kwargs) -> RobotBackend:
    """Factory function to create a robot backend by mode name."""
    if mode == "rtde":
        from teleop_dev.robot.core.ur_robot import RTDEBackend
        return RTDEBackend(robot_ip=kwargs["robot_ip"])
    elif mode == "sim":
        from teleop_dev.robot.core.sim_robot import SimBackend
        return SimBackend(**{k: v for k, v in kwargs.items() if k != "robot_ip"})
    raise ValueError(f"Unknown mode: {mode}")
