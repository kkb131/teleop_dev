"""Core infrastructure — shared across all standalone feature modules."""

from teleop_dev.robot.core.robot_backend import RobotBackend, create_backend
from teleop_dev.robot.core.trajectory_executor import (
    execute_trajectory,
    resample_trajectory,
    validate_trajectory,
    check_start_match,
)
from teleop_dev.robot.core.kinematics import PinocchioIK
from teleop_dev.robot.core.controller_utils import ControllerSwitcher

__all__ = [
    "RobotBackend",
    "create_backend",
    "execute_trajectory",
    "resample_trajectory",
    "validate_trajectory",
    "check_start_match",
    "PinocchioIK",
    "ControllerSwitcher",
]
