"""Hand teleop protocol — UDP 9872 wire format.

Shared between operator PC (manus_sender) and robot PC (receiver).
Dependencies: stdlib + numpy only (no ROS2, no device SDKs).
"""

from dataclasses import dataclass, field

import numpy as np

# ─────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES_PER_FINGER = {
    "Thumb": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Index": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Middle": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Ring": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Pinky": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
}

NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

HAND_UDP_PORT = 9872

# ─────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────

@dataclass
class HandData:
    """Hand tracking data from a single Manus glove."""
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    finger_spread: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    wrist_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wrist_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    hand_side: str = "right"
    timestamp: float = 0.0
    valid: bool = False
