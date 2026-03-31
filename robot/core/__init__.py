"""Core infrastructure — shared across all standalone feature modules."""

from robot.core.robot_backend import RobotBackend, create_backend
from robot.core.trajectory_executor import (
    execute_trajectory,
    resample_trajectory,
    validate_trajectory,
    check_start_match,
)
from robot.core.kinematics import PinocchioIK
from robot.core.controller_utils import ControllerSwitcher

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
