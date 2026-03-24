"""Unified teleop protocol for Operator PC ↔ Robot PC communication.

All input devices (Vive, keyboard, joystick) on the Operator PC send the same
packet format — an absolute target pose in the robot's base_link frame.

Dependencies: stdlib + numpy only (no ROS, no pinocchio).
This file can be copied to the Operator PC conda environment as-is.

Protocol packets (JSON over UDP, port 9871):

    teleop_pose   — Operator → Robot: absolute target pose + buttons
    query_pose    — Operator → Robot: request current TCP pose (startup)
    pose_response — Robot → Operator: current TCP pose reply
"""

import json
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

PROTOCOL_VERSION = 1
DEFAULT_PORT = 9871


@dataclass
class ButtonState:
    """Teleop button/command flags."""
    estop: bool = False
    reset: bool = False
    quit: bool = False
    speed_up: bool = False
    speed_down: bool = False
    ft_zero: bool = False
    admittance_toggle: bool = False
    admittance_preset: str = ""       # "STIFF"|"MEDIUM"|"SOFT"|"FREE"|""
    admittance_cycle: bool = False
    impedance_preset: str = ""        # "STIFF"|"MEDIUM"|"SOFT"|""
    gain_scale_up: bool = False
    gain_scale_down: bool = False


@dataclass
class TeleopPosePacket:
    """Operator → Robot: absolute target pose in base_link frame.

    Quaternion convention: wxyz (consistent with vive_sender / SteamVR).
    The robot-side receiver converts to xyzw for pinocchio internally.
    """
    pos: np.ndarray            # [x, y, z] metres, base_link frame
    quat: np.ndarray           # [w, x, y, z] base_link frame
    buttons: ButtonState = field(default_factory=ButtonState)
    gripper: float = 0.0       # 0.0–1.0 (future use)
    timestamp: float = 0.0

    def to_bytes(self) -> bytes:
        """Serialize to UTF-8 JSON bytes for UDP send."""
        d = {
            "v": PROTOCOL_VERSION,
            "type": "teleop_pose",
            "pos": self.pos.tolist(),
            "quat": self.quat.tolist(),
            "buttons": asdict(self.buttons),
            "gripper": self.gripper,
            "timestamp": self.timestamp or time.time(),
        }
        return json.dumps(d, separators=(",", ":")).encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional["TeleopPosePacket"]:
        """Deserialize from UDP payload. Returns None on parse error."""
        try:
            d = json.loads(data)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None
        if d.get("type") != "teleop_pose":
            return None
        if d.get("v", 0) != PROTOCOL_VERSION:
            return None
        try:
            btn_dict = d.get("buttons", {})
            buttons = ButtonState(**{
                k: btn_dict[k] for k in ButtonState.__dataclass_fields__
                if k in btn_dict
            })
            return cls(
                pos=np.array(d["pos"], dtype=np.float64),
                quat=np.array(d["quat"], dtype=np.float64),
                buttons=buttons,
                gripper=float(d.get("gripper", 0.0)),
                timestamp=float(d.get("timestamp", 0.0)),
            )
        except (KeyError, TypeError, ValueError):
            return None


# ---------------------------------------------------------------------------
# Pose query / response (startup handshake)
# ---------------------------------------------------------------------------

def make_query_pose_bytes() -> bytes:
    """Create a pose query packet."""
    return json.dumps({"type": "query_pose"}).encode("utf-8")


def make_pose_response_bytes(pos: np.ndarray, quat: np.ndarray) -> bytes:
    """Create a pose response packet (robot → operator).

    Args:
        pos: [x, y, z] in base_link frame.
        quat: [w, x, y, z] in base_link frame.
    """
    d = {
        "type": "pose_response",
        "pos": pos.tolist(),
        "quat": quat.tolist(),
    }
    return json.dumps(d, separators=(",", ":")).encode("utf-8")


def parse_pose_response(data: bytes) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Parse a pose response packet.

    Returns:
        (pos, quat) as numpy arrays, or None on error.
    """
    try:
        d = json.loads(data)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    if d.get("type") != "pose_response":
        return None
    try:
        pos = np.array(d["pos"], dtype=np.float64)
        quat = np.array(d["quat"], dtype=np.float64)
        return pos, quat
    except (KeyError, TypeError, ValueError):
        return None


def is_query_pose(data: bytes) -> bool:
    """Check if raw UDP payload is a query_pose request."""
    try:
        d = json.loads(data)
        return d.get("type") == "query_pose"
    except (json.JSONDecodeError, UnicodeDecodeError):
        return False
