"""DG5F 관절 이름, 한계, 링크 길이 등 상수.

URDF(`dg5f_right.urdf`)에서 추출한 값. 모든 세대에서 공통 사용.
"""

import math
import numpy as np
from dataclasses import dataclass


NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",  # Thumb
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",  # Index
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",  # Middle
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",  # Ring
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",  # Pinky
]

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]


@dataclass
class JointLimit:
    lower: float  # radians
    upper: float  # radians


# URDF에서 직접 추출한 값
RIGHT_JOINT_LIMITS = [
    # Thumb
    JointLimit(-0.384, 0.890),   # rj_dg_1_1 X abd
    JointLimit(-math.pi, 0.0),   # rj_dg_1_2 Z opp
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_1_3 X flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_1_4 X flex
    # Index
    JointLimit(-0.419, 0.611),   # rj_dg_2_1 X abd
    JointLimit(0.0, 2.007),      # rj_dg_2_2 Y MCP flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_2_3 Y PIP flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_2_4 Y DIP flex
    # Middle
    JointLimit(-0.611, 0.611),
    JointLimit(0.0, 1.955),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Ring
    JointLimit(-0.611, 0.419),
    JointLimit(0.0, 1.902),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Pinky
    JointLimit(-0.017, 1.047),   # rj_dg_5_1 Z abd
    JointLimit(-0.419, 0.611),   # rj_dg_5_2 X spread
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
]

LEFT_JOINT_LIMITS = [
    # Thumb (mirrored)
    JointLimit(-0.890, 0.384),
    JointLimit(0.0, math.pi),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Index
    JointLimit(-0.611, 0.419),
    JointLimit(0.0, 2.007),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Middle
    JointLimit(-0.611, 0.611),
    JointLimit(0.0, 1.955),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Ring
    JointLimit(-0.419, 0.611),
    JointLimit(0.0, 1.902),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Pinky
    JointLimit(-1.047, 0.017),
    JointLimit(-0.611, 0.419),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
]


def get_joint_names(hand_side: str) -> list[str]:
    return RIGHT_JOINT_NAMES if hand_side == "right" else LEFT_JOINT_NAMES

def get_joint_limits(hand_side: str) -> list[JointLimit]:
    return RIGHT_JOINT_LIMITS if hand_side == "right" else LEFT_JOINT_LIMITS

def get_limits_arrays(hand_side: str) -> tuple[np.ndarray, np.ndarray]:
    """(lower[20], upper[20]) numpy 배열 반환."""
    limits = get_joint_limits(hand_side)
    lower = np.array([lim.lower for lim in limits])
    upper = np.array([lim.upper for lim in limits])
    return lower, upper
