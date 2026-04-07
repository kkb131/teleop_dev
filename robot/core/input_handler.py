"""Input handlers for teleop: keyboard and unified protocol."""

import json
import socket
import sys
import select
import termios
import time
import tty
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

import numpy as np


SPEED_SCALES = [0.5, 1.0, 2.0, 4.0, 8.0]
DEFAULT_SPEED_IDX = 1  # 1.0x


@dataclass
class TeleopCommand:
    """Output from input handler."""
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    estop: bool = False
    reset: bool = False
    quit: bool = False
    speed_scale: float = 1.0
    target_pos: Optional[np.ndarray] = None
    target_quat: Optional[np.ndarray] = None
    mode: str = "velocity"
    admittance_toggle: bool = False
    admittance_preset: str = ""
    admittance_cycle: bool = False
    ft_zero: bool = False
    impedance_preset: str = ""
    gain_scale_up: bool = False
    gain_scale_down: bool = False
    tool_z_delta: float = 0.0


class InputHandler(ABC):
    """Abstract base for teleop input devices."""

    @abstractmethod
    def setup(self):
        """Initialize the input device."""

    @abstractmethod
    def cleanup(self):
        """Restore terminal / release device."""

    @abstractmethod
    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        """Read input and return a teleop command."""

    @property
    def speed_scale(self) -> float:
        return 1.0

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, *args):
        self.cleanup()


# Key -> (vx, vy, vz, wx, wy, wz) unit direction
KEY_MAP = {
    "w": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    "s": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    "a": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "d": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "q": (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    "e": (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
    "u": (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    "o": (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
    "i": (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    "k": (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
    "j": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    "l": (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
}


class KeyboardInput(InputHandler):
    """Non-blocking keyboard input via termios."""

    def __init__(self, cartesian_step: float = 0.005, rotation_step: float = 0.05):
        self._base_cart_step = cartesian_step
        self._base_rot_step = rotation_step
        self._old_settings = None
        self._speed_idx = DEFAULT_SPEED_IDX

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cleanup(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def _read_key(self, timeout: float) -> str | None:
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[" and select.select([sys.stdin], [], [], 0.01)[0]:
                        sys.stdin.read(1)
                return "ESC"
            return ch
        return None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        key = self._read_key(timeout)

        if key is not None:
            while True:
                next_key = self._read_key(0)
                if next_key is None:
                    break
                key = next_key

        if key is None:
            return cmd

        if key in ("x", "ESC"):
            cmd.quit = True
            return cmd

        if key == " ":
            cmd.estop = True
            return cmd

        if key == "r":
            cmd.reset = True
            return cmd

        if key in ("+", "="):
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
            return cmd
        if key == "-":
            self._speed_idx = max(self._speed_idx - 1, 0)
            cmd.speed_scale = self.speed_scale
            return cmd

        if key == "t":
            cmd.admittance_toggle = True
            return cmd
        if key == "z":
            cmd.ft_zero = True
            return cmd
        if key in ("1", "2", "3", "4"):
            cmd.admittance_preset = {"1": "STIFF", "2": "MEDIUM", "3": "SOFT", "4": "FREE"}[key]
            return cmd

        if key == "[":
            cmd.gain_scale_down = True
            return cmd
        if key == "]":
            cmd.gain_scale_up = True
            return cmd

        if key == "c":
            cmd.tool_z_delta = self._base_cart_step * self.speed_scale
            return cmd
        if key == "v":
            cmd.tool_z_delta = -self._base_cart_step * self.speed_scale
            return cmd

        if key in KEY_MAP:
            direction = np.array(KEY_MAP[key])
            scale = np.array(
                [self._base_cart_step * self.speed_scale] * 3
                + [self._base_rot_step * self.speed_scale] * 3
            )
            cmd.velocity = direction * scale

        return cmd


class UnifiedNetworkInput(InputHandler):
    """Receives unified teleop_pose packets and responds to query_pose requests.

    All input devices on the Operator PC send the same packet format:
    absolute target pose in robot base_link frame (via arm_protocol.py).

    Also serves as a pose query responder: when a sender starts up and sends
    a query_pose packet, this handler replies with the robot's current TCP pose.
    """

    def __init__(self, port: int = 9871,
                 pose_provider: Optional[Callable[[], Tuple[np.ndarray, np.ndarray]]] = None):
        self._port = port
        self._pose_provider = pose_provider
        self._sock: Optional[socket.socket] = None
        self._speed_idx = DEFAULT_SPEED_IDX

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.setblocking(False)
        print(f"[UnifiedInput] Listening on UDP port {self._port}")

    def cleanup(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        from protocol.arm_protocol import (
            TeleopPosePacket,
            is_query_pose,
            make_pose_response_bytes,
        )

        cmd = TeleopCommand(speed_scale=self.speed_scale)
        if self._sock is None:
            return cmd

        latest_data = None
        try:
            while True:
                data, addr = self._sock.recvfrom(4096)
                if is_query_pose(data):
                    self._handle_pose_query(addr, make_pose_response_bytes)
                    continue
                latest_data = data
        except BlockingIOError:
            pass

        if latest_data is None:
            return cmd

        pkt = TeleopPosePacket.from_bytes(latest_data)
        if pkt is None:
            return cmd

        btn = pkt.buttons
        if btn.estop:
            cmd.estop = True
            return cmd
        if btn.reset:
            cmd.reset = True
            return cmd
        if btn.quit:
            cmd.quit = True
            return cmd

        if btn.speed_up:
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
        if btn.speed_down:
            self._speed_idx = max(self._speed_idx - 1, 0)
        cmd.speed_scale = self.speed_scale

        cmd.ft_zero = btn.ft_zero
        cmd.admittance_toggle = btn.admittance_toggle
        cmd.admittance_preset = btn.admittance_preset
        cmd.admittance_cycle = btn.admittance_cycle
        cmd.impedance_preset = btn.impedance_preset
        cmd.gain_scale_up = btn.gain_scale_up
        cmd.gain_scale_down = btn.gain_scale_down

        w, x, y, z = pkt.quat
        cmd.target_pos = pkt.pos.copy()
        cmd.target_quat = np.array([x, y, z, w])
        cmd.mode = "absolute"

        return cmd

    def _handle_pose_query(self, sender_addr, make_response_fn):
        if self._pose_provider is None:
            print("[UnifiedInput] Pose query received but no pose_provider set")
            return
        try:
            pos, quat_xyzw = self._pose_provider()
            quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0],
                                  quat_xyzw[1], quat_xyzw[2]])
            response = make_response_fn(pos, quat_wxyz)
            self._sock.sendto(response, sender_addr)
            print(f"[UnifiedInput] Pose query response sent to {sender_addr}")
        except Exception as e:
            print(f"[UnifiedInput] Failed to respond to pose query: {e}")


def create_input(input_type: str, cartesian_step: float = 0.005,
                 rotation_step: float = 0.05,
                 **kwargs) -> InputHandler:
    """Factory to create input handler by type."""
    if input_type == "keyboard":
        return KeyboardInput(cartesian_step, rotation_step)
    elif input_type == "unified":
        return UnifiedNetworkInput(
            port=kwargs.get("unified_port", 9871),
            pose_provider=kwargs.get("pose_provider"),
        )
    raise ValueError(f"Unknown input type: {input_type!r}. Valid: 'keyboard', 'unified'")
