"""Input handlers for teleop: keyboard, Xbox controller, network (UDP), Vive Tracker, and unified protocol."""

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
    speed_scale: float = 1.0  # current speed multiplier
    # Absolute pose mode (unified protocol — when set, velocity is ignored)
    target_pos: Optional[np.ndarray] = None    # [x,y,z] in base_link (metres)
    target_quat: Optional[np.ndarray] = None   # [x,y,z,w] in base_link (pinocchio xyzw)
    mode: str = "velocity"                      # "velocity" | "absolute"
    # Admittance control
    admittance_toggle: bool = False
    admittance_preset: str = ""  # "STIFF", "MEDIUM", "SOFT", "FREE", or "" (no change)
    admittance_cycle: bool = False  # cycle through presets (STIFF→MEDIUM→SOFT)
    ft_zero: bool = False
    # Impedance control
    impedance_preset: str = ""  # "STIFF", "MEDIUM", "SOFT", or "" (no change)
    gain_scale_up: bool = False
    gain_scale_down: bool = False
    # Tool-frame Z-axis translation
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
# UR10e base frame: X=right, Y=forward (away from base), Z=up
KEY_MAP = {
    "w": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),   # forward  (+Y)
    "s": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),   # backward (-Y)
    "a": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # left     (-X)
    "d": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # right    (+X)
    "q": (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),    # up       (+Z)
    "e": (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),   # down     (-Z)
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
                # Consume escape sequence
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
            # Drain buffered keys (from key repeat), keep only the latest
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

        # Speed adjustment
        if key in ("+", "="):
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
            return cmd
        if key == "-":
            self._speed_idx = max(self._speed_idx - 1, 0)
            cmd.speed_scale = self.speed_scale
            return cmd

        # Admittance controls
        if key == "t":
            cmd.admittance_toggle = True
            return cmd
        if key == "z":
            cmd.ft_zero = True
            return cmd
        if key in ("1", "2", "3", "4"):
            cmd.admittance_preset = {"1": "STIFF", "2": "MEDIUM", "3": "SOFT", "4": "FREE"}[key]
            return cmd

        # Impedance gain scaling
        if key == "[":
            cmd.gain_scale_down = True
            return cmd
        if key == "]":
            cmd.gain_scale_up = True
            return cmd

        # Tool-frame Z-axis translation
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


class XboxInput(InputHandler):
    """Xbox controller input via pygame."""

    def __init__(self, linear_scale: float = 0.02, angular_scale: float = 0.05):
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._joystick = None
        self._pygame = None
        self._speed_idx = DEFAULT_SPEED_IDX
        self._prev_hat_y = 0
        self._prev_b = False

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        import pygame
        self._pygame = pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No Xbox controller found. Connect a controller and retry.")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

    def cleanup(self):
        if self._pygame:
            self._pygame.quit()

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        pg = self._pygame
        if pg is None:
            return cmd

        for event in pg.event.get():
            if event.type == pg.JOYBUTTONDOWN:
                if event.button == 8:  # Logitech = E-Stop
                    cmd.estop = True
                    return cmd
                if event.button == 7:  # Start = Reset
                    cmd.reset = True
                    return cmd
                if event.button == 6:  # Back = Quit
                    cmd.quit = True
                    return cmd
                if event.button == 3:  # Y = Zero F/T
                    cmd.ft_zero = True
                    return cmd

        js = self._joystick
        if js is None:
            return cmd

        # B button edge-detection (cycle mode once per press)
        b_now = bool(js.get_button(1))
        if b_now and not self._prev_b:
            cmd.admittance_cycle = True
        self._prev_b = b_now

        def dz(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx = dz(js.get_axis(0))
        ly = dz(-js.get_axis(1))
        lt = (js.get_axis(2) + 1.0) / 2.0 if js.get_numaxes() > 2 else 0.0
        rt = (js.get_axis(5) + 1.0) / 2.0 if js.get_numaxes() > 5 else 0.0
        vy = dz(rt - lt, 0.05)
        rx = dz(js.get_axis(3)) if js.get_numaxes() > 3 else 0.0
        ry = dz(-js.get_axis(4)) if js.get_numaxes() > 4 else 0.0
        lb = 1.0 if js.get_button(4) else 0.0
        rb = 1.0 if js.get_button(5) else 0.0
        wyaw = rb - lb

        s = self.speed_scale
        cmd.velocity = np.array([
            lx * self._linear_scale * s,       # L-Stick X → X (좌우)
            vy * self._linear_scale * s,        # LT/RT    → Y (앞뒤)
            ly * self._linear_scale * s,        # L-Stick Y → Z (상하)
            -ry * self._angular_scale * s,      # R-Stick Y → Roll  (-90° 회전)
            -rx * self._angular_scale * s,      # R-Stick X → Pitch (-90° 회전)
            wyaw * self._angular_scale * s,
        ])

        # D-pad for tool-frame Z-axis and speed (edge-triggered)
        if js.get_numhats() > 0:
            hx, hy = js.get_hat(0)
            if hy > 0 and self._prev_hat_y <= 0:  # rising edge only
                self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
                cmd.speed_scale = self.speed_scale
            elif hy < 0 and self._prev_hat_y >= 0:  # falling edge only
                self._speed_idx = max(self._speed_idx - 1, 0)
                cmd.speed_scale = self.speed_scale
            self._prev_hat_y = hy
            if hx != 0:  # D-pad left/right = tool Z-axis
                cmd.tool_z_delta = hx * self._linear_scale * s

        return cmd



class NetworkInput(InputHandler):
    """Receives joystick commands via UDP from a remote joystick_sender.py.

    Expects JSON packets with raw pygame axes/buttons:
        {"axes": [a0, a1, ...], "buttons": [b0, b1, ...]}
    Axis/button mapping matches XboxInput (XInput mode).
    """

    def __init__(self, port: int = 9870, linear_scale: float = 0.02,
                 angular_scale: float = 0.05):
        self._port = port
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._sock: socket.socket | None = None
        self._speed_idx = DEFAULT_SPEED_IDX
        self._prev_hat_y = 0
        self._prev_b = False

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.setblocking(False)
        print(f"[NetworkInput] Listening on UDP port {self._port}")

    def cleanup(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        if self._sock is None:
            return cmd

        # Drain buffer, keep only latest packet
        data = None
        try:
            while True:
                data, _ = self._sock.recvfrom(4096)
        except BlockingIOError:
            pass

        if data is None:
            # No packet this cycle — return zero velocity
            return cmd

        try:
            pkt = json.loads(data)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return cmd

        axes = pkt.get("axes", [])
        buttons = pkt.get("buttons", [])

        def axis(idx, default=0.0):
            return axes[idx] if idx < len(axes) else default

        def btn(idx):
            return buttons[idx] if idx < len(buttons) else 0
        # Button events
        if btn(8):  # Logitech = E-Stop
            cmd.estop = True
            return cmd
        if btn(7):  # Start = Reset
            cmd.reset = True
            return cmd
        if btn(6):  # Back = Quit
            cmd.quit = True
            return cmd
        # B button edge-detection (cycle mode once per press)
        b_now = bool(btn(1))
        if b_now and not self._prev_b:
            cmd.admittance_cycle = True
        self._prev_b = b_now

        if btn(3):  # Y = Zero F/T
            cmd.ft_zero = True
            return cmd

        # Speed adjustment via D-pad (hat, edge-triggered)
        hat = pkt.get("hat", [0, 0])
        hx = hat[0] if len(hat) > 0 else 0
        hy = hat[1] if len(hat) > 1 else 0
        if hy > 0 and self._prev_hat_y <= 0:  # rising edge only
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
        elif hy < 0 and self._prev_hat_y >= 0:  # falling edge only
            self._speed_idx = max(self._speed_idx - 1, 0)
        self._prev_hat_y = hy
        # D-pad left/right = tool-frame Z-axis translation
        if hx != 0:
            cmd.tool_z_delta = hx * self._linear_scale * self.speed_scale
            cmd.speed_scale = self.speed_scale

        # Analog sticks → velocity (same mapping as XboxInput)
        def dz(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx = dz(axis(0))
        ly = dz(-axis(1))
        lt = (axis(2) + 1.0) / 2.0
        rt = (axis(5) + 1.0) / 2.0
        vy = dz(rt - lt, 0.05)
        rx = dz(axis(3))
        ry = dz(-axis(4))
        lb = 1.0 if btn(4) else 0.0
        rb = 1.0 if btn(5) else 0.0
        wyaw = rb - lb

        s = self.speed_scale
        cmd.velocity = np.array([
            lx * self._linear_scale * s,       # L-Stick X → X (좌우)
            vy * self._linear_scale * s,        # LT/RT    → Y (앞뒤)
            ly * self._linear_scale * s,        # L-Stick Y → Z (상하)
            -ry * self._angular_scale * s,      # R-Stick Y → Roll  (-90° 회전)
            -rx * self._angular_scale * s,      # R-Stick X → Pitch (-90° 회전)
            wyaw * self._angular_scale * s,
        ])

        return cmd


class ViveNetworkInput(InputHandler):
    """Receives Vive Tracker pose via UDP and converts to velocity commands.

    Expects JSON packets from vive_sender.py:
        {"type": "vive", "pos": [x,y,z], "quat": [w,x,y,z],
         "tracking": true, "buttons": {...}, "timestamp": ...}

    Computes Cartesian velocity from pose differences (delta pose / dt).
    Optionally applies coordinate frame calibration (SteamVR → robot base).
    """

    def __init__(self, port: int = 9871, linear_scale: float = 1.0,
                 angular_scale: float = 1.0,
                 calibration_file: str | None = None,
                 deadzone: float = 0.002):
        self._port = port
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._calibration_file = calibration_file
        self._deadzone = deadzone
        self._sock: socket.socket | None = None
        self._speed_idx = DEFAULT_SPEED_IDX
        # Calibration transform
        self._cal_R: np.ndarray | None = None  # (3,3)
        self._cal_t: np.ndarray | None = None  # (3,)
        # Previous pose for velocity computation
        self._prev_pos: np.ndarray | None = None
        self._prev_rot: np.ndarray | None = None  # (3,3) rotation matrix
        self._prev_time: float | None = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.setblocking(False)
        print(f"[ViveNetworkInput] Listening on UDP port {self._port}")

        # Load calibration if provided
        if self._calibration_file:
            self._load_calibration(self._calibration_file)
        else:
            print("[ViveNetworkInput] No calibration file — using raw SteamVR coords")
            print("[ViveNetworkInput] Default mapping: SteamVR(X,Y,Z) → Robot(X,Z,Y)")
            # Default: SteamVR is Y-up right-handed, UR10e is Z-up
            # SteamVR X → Robot X, SteamVR Y → Robot Z, SteamVR Z → Robot -Y
            self._cal_R = np.array([
                [1.0,  0.0,  0.0],
                [0.0,  0.0, -1.0],
                [0.0,  1.0,  0.0],
            ])
            self._cal_t = np.zeros(3)

    def cleanup(self):
        if self._sock:
            self._sock.close()
            self._sock = None
        self._prev_pos = None
        self._prev_rot = None
        self._prev_time = None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        if self._sock is None:
            return cmd

        # Drain buffer, keep latest packet
        data = None
        try:
            while True:
                data, _ = self._sock.recvfrom(4096)
        except BlockingIOError:
            pass

        if data is None:
            return cmd

        try:
            pkt = json.loads(data)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return cmd

        if pkt.get("type") != "vive":
            return cmd

        # Process buttons
        buttons = pkt.get("buttons", {})
        if buttons.get("estop"):
            cmd.estop = True
            self._prev_pos = None  # Reset velocity tracking
            self._prev_time = None
            return cmd
        if buttons.get("reset"):
            cmd.reset = True
            self._prev_pos = None
            self._prev_time = None
            return cmd
        if buttons.get("quit"):
            cmd.quit = True
            return cmd
        if buttons.get("speed_up"):
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
        if buttons.get("speed_down"):
            self._speed_idx = max(self._speed_idx - 1, 0)
            cmd.speed_scale = self.speed_scale

        # Check tracking
        if not pkt.get("tracking", False):
            self._prev_pos = None
            self._prev_time = None
            return cmd

        # Parse pose
        pos_vive = np.array(pkt["pos"])
        quat_vive = np.array(pkt["quat"])  # wxyz
        timestamp = pkt.get("timestamp", time.time())

        # Apply calibration transform
        pos = self._cal_R @ pos_vive + self._cal_t
        rot = self._cal_R @ self._quat_to_rot(quat_vive)

        # Compute velocity from pose delta
        if self._prev_pos is not None and self._prev_time is not None:
            dt = timestamp - self._prev_time
            if 0.001 < dt < 0.5:  # Guard against stale or crazy timestamps
                # Linear velocity
                lin_vel = (pos - self._prev_pos) / dt

                # Angular velocity from rotation delta
                # R_delta = R_curr @ R_prev^T
                R_delta = rot @ self._prev_rot.T
                ang_vel = self._rot_to_angular_vel(R_delta, dt)

                # Apply deadzone
                lin_mag = np.linalg.norm(lin_vel)
                ang_mag = np.linalg.norm(ang_vel)

                if lin_mag < self._deadzone:
                    lin_vel = np.zeros(3)
                if ang_mag < self._deadzone:
                    ang_vel = np.zeros(3)

                s = self.speed_scale
                cmd.velocity = np.array([
                    lin_vel[0] * self._linear_scale * s,
                    lin_vel[1] * self._linear_scale * s,
                    lin_vel[2] * self._linear_scale * s,
                    ang_vel[0] * self._angular_scale * s,
                    ang_vel[1] * self._angular_scale * s,
                    ang_vel[2] * self._angular_scale * s,
                ])

        self._prev_pos = pos
        self._prev_rot = rot
        self._prev_time = timestamp

        return cmd

    def _load_calibration(self, filepath: str):
        """Load R, t from calibration JSON."""
        try:
            with open(filepath) as f:
                data = json.load(f)
            self._cal_R = np.array(data["R"])
            self._cal_t = np.array(data["t"])
            print(f"[ViveNetworkInput] Loaded calibration from {filepath}")
        except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
            print(f"[ViveNetworkInput] WARNING: Failed to load calibration: {e}")
            print("[ViveNetworkInput] Falling back to default axis mapping")
            self._cal_R = np.array([
                [1.0,  0.0,  0.0],
                [0.0,  0.0, -1.0],
                [0.0,  1.0,  0.0],
            ])
            self._cal_t = np.zeros(3)

    @staticmethod
    def _quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Quaternion (wxyz) to 3x3 rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _rot_to_angular_vel(R_delta: np.ndarray, dt: float) -> np.ndarray:
        """Extract angular velocity from rotation delta and time step.

        Uses axis-angle: angle = arccos((tr(R)-1)/2), axis from skew-sym part.
        """
        import math
        tr = R_delta[0, 0] + R_delta[1, 1] + R_delta[2, 2]
        cos_angle = (tr - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = math.acos(cos_angle)

        if abs(angle) < 1e-6:
            return np.zeros(3)

        # Axis from skew-symmetric part of R_delta
        axis = np.array([
            R_delta[2, 1] - R_delta[1, 2],
            R_delta[0, 2] - R_delta[2, 0],
            R_delta[1, 0] - R_delta[0, 1],
        ])
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-8:
            return np.zeros(3)

        axis /= axis_norm
        return axis * (angle / dt)


class UnifiedNetworkInput(InputHandler):
    """Receives unified teleop_pose packets and responds to query_pose requests.

    All input devices on the Operator PC send the same packet format:
    absolute target pose in robot base_link frame (via teleop_protocol.py).

    Also serves as a pose query responder: when a sender starts up and sends
    a query_pose packet, this handler replies with the robot's current TCP pose.
    """

    def __init__(self, port: int = 9871,
                 pose_provider: Optional[Callable[[], Tuple[np.ndarray, np.ndarray]]] = None):
        """
        Args:
            port: UDP port to listen on (default 9871).
            pose_provider: Callback returning (pos_xyz, quat_xyzw) for pose queries.
                           If None, pose queries are ignored.
        """
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

        # Drain buffer, keep latest teleop_pose; respond to query_pose inline
        latest_data = None
        try:
            while True:
                data, addr = self._sock.recvfrom(4096)
                # Handle pose query immediately
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

        # Map buttons
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

        # Speed control
        if btn.speed_up:
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
        if btn.speed_down:
            self._speed_idx = max(self._speed_idx - 1, 0)
        cmd.speed_scale = self.speed_scale

        # Admittance / impedance controls
        cmd.ft_zero = btn.ft_zero
        cmd.admittance_toggle = btn.admittance_toggle
        cmd.admittance_preset = btn.admittance_preset
        cmd.admittance_cycle = btn.admittance_cycle
        cmd.impedance_preset = btn.impedance_preset
        cmd.gain_scale_up = btn.gain_scale_up
        cmd.gain_scale_down = btn.gain_scale_down

        # Absolute pose — convert quat from wxyz (protocol) to xyzw (pinocchio)
        w, x, y, z = pkt.quat
        cmd.target_pos = pkt.pos.copy()
        cmd.target_quat = np.array([x, y, z, w])  # xyzw
        cmd.mode = "absolute"

        return cmd

    def _handle_pose_query(self, sender_addr, make_response_fn):
        """Respond to a pose query from an operator PC sender."""
        if self._pose_provider is None:
            print("[UnifiedInput] Pose query received but no pose_provider set")
            return
        try:
            pos, quat_xyzw = self._pose_provider()
            # Convert xyzw → wxyz for protocol
            quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0],
                                  quat_xyzw[1], quat_xyzw[2]])
            response = make_response_fn(pos, quat_wxyz)
            self._sock.sendto(response, sender_addr)
            print(f"[UnifiedInput] Pose query response sent to {sender_addr}")
        except Exception as e:
            print(f"[UnifiedInput] Failed to respond to pose query: {e}")


def create_input(input_type: str, cartesian_step: float = 0.005,
                 rotation_step: float = 0.05,
                 linear_scale: float = 0.02,
                 angular_scale: float = 0.05,
                 network_port: int = 9870,
                 **kwargs) -> InputHandler:
    """Factory to create input handler by type."""
    if input_type == "keyboard":
        return KeyboardInput(cartesian_step, rotation_step)
    elif input_type == "xbox":
        return XboxInput(linear_scale, angular_scale)
    elif input_type == "network":
        return NetworkInput(port=network_port, linear_scale=linear_scale,
                            angular_scale=angular_scale)
    elif input_type == "vive":
        return ViveNetworkInput(
            port=kwargs.get("vive_port", 9871),
            linear_scale=kwargs.get("vive_linear_scale", 1.0),
            angular_scale=kwargs.get("vive_angular_scale", 1.0),
            calibration_file=kwargs.get("calibration_file"),
            deadzone=kwargs.get("vive_deadzone", 0.002),
        )
    elif input_type == "unified":
        return UnifiedNetworkInput(
            port=kwargs.get("unified_port", 9871),
            pose_provider=kwargs.get("pose_provider"),
        )
    raise ValueError(f"Unknown input type: {input_type}")
