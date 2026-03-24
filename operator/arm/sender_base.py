"""Base class for all operator-side teleop senders.

Provides common functionality:
  - UDP socket management
  - Initial pose query from robot PC
  - Virtual pose accumulation (relative mapping)
  - Rate-controlled send loop
  - Quaternion math for delta application

Subclasses implement _read_input() for device-specific input.

Dependencies: numpy only (no ROS, no pinocchio).
"""

import json
import math
import socket
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from teleop_dev.protocol.arm_protocol import (
    ButtonState,
    TeleopPosePacket,
    make_query_pose_bytes,
    parse_pose_response,
)

# Default home pose (fallback if pose query fails)
DEFAULT_HOME_POS = np.array([0.0, -0.4, 0.4])      # metres, base_link
DEFAULT_HOME_QUAT = np.array([0.0, 0.707, 0.707, 0.0])  # wxyz, pointing down


@dataclass
class InputResult:
    """Result from a single _read_input() call."""
    delta_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    delta_rot_axis_angle: np.ndarray = field(default_factory=lambda: np.zeros(3))  # axis * angle (rad)
    buttons: ButtonState = field(default_factory=ButtonState)
    quit: bool = False


class TeleopSenderBase(ABC):
    """Abstract base for operator-side teleop senders.

    Manages a virtual target pose that starts at the robot's current TCP pose
    (queried at startup) and is updated each loop by device-specific deltas.
    The accumulated absolute pose is sent to the robot PC via the unified protocol.
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50):
        self._target_ip = target_ip
        self._port = port
        self._hz = hz
        self._dt = 1.0 / hz

        self._sock: Optional[socket.socket] = None
        self._target_addr: Tuple[str, int] = (target_ip, port)

        # Virtual pose (accumulated)
        self._virtual_pos: Optional[np.ndarray] = None
        self._virtual_quat: Optional[np.ndarray] = None  # wxyz

    # ------------------------------------------------------------------
    # Pose query
    # ------------------------------------------------------------------

    def query_initial_pose(self, timeout: float = 5.0,
                           retries: int = 3) -> Tuple[np.ndarray, np.ndarray]:
        """Query robot PC for current TCP pose.

        Sends query_pose packets and waits for pose_response.
        Falls back to DEFAULT_HOME_POS/QUAT on failure.

        Returns:
            (pos, quat_wxyz) in base_link frame.
        """
        query_bytes = make_query_pose_bytes()
        per_try_timeout = timeout / retries

        for attempt in range(retries):
            try:
                self._sock.sendto(query_bytes, self._target_addr)
                self._sock.settimeout(per_try_timeout)
                data, _ = self._sock.recvfrom(4096)
                result = parse_pose_response(data)
                if result is not None:
                    pos, quat = result
                    print(f"[Sender] Initial pose received: "
                          f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                    return pos, quat
                print(f"[Sender] Attempt {attempt+1}: invalid response, retrying...")
            except socket.timeout:
                print(f"[Sender] Attempt {attempt+1}/{retries}: no response, retrying...")
            except OSError as e:
                print(f"[Sender] Attempt {attempt+1}/{retries}: socket error: {e}")

        print(f"[Sender] WARNING: Pose query failed. Using default home pose.")
        return DEFAULT_HOME_POS.copy(), DEFAULT_HOME_QUAT.copy()

    # ------------------------------------------------------------------
    # Virtual pose management
    # ------------------------------------------------------------------

    def _apply_delta(self, delta_pos: np.ndarray,
                     delta_rot_axis_angle: np.ndarray):
        """Apply position + rotation delta to virtual pose.

        Args:
            delta_pos: [dx, dy, dz] in base_link frame (metres).
            delta_rot_axis_angle: axis * angle (radians) in base_link frame.
        """
        self._virtual_pos = self._virtual_pos + delta_pos

        angle = np.linalg.norm(delta_rot_axis_angle)
        if angle > 1e-8:
            axis = delta_rot_axis_angle / angle
            dq = _axis_angle_to_quat(axis, angle)
            self._virtual_quat = _quat_multiply(dq, self._virtual_quat)
            # Re-normalize
            self._virtual_quat /= np.linalg.norm(self._virtual_quat)

    def _send_packet(self, buttons: ButtonState):
        """Send current virtual pose as a TeleopPosePacket."""
        pkt = TeleopPosePacket(
            pos=self._virtual_pos,
            quat=self._virtual_quat,
            buttons=buttons,
            timestamp=time.time(),
        )
        self._sock.sendto(pkt.to_bytes(), self._target_addr)

    # ------------------------------------------------------------------
    # Abstract: device-specific input
    # ------------------------------------------------------------------

    @abstractmethod
    def _read_input(self) -> InputResult:
        """Read device-specific input and return deltas + buttons.

        Called once per loop iteration at self._hz rate.
        Must not block longer than self._dt.
        """

    def _setup_device(self):
        """Optional: device-specific setup (called before main loop)."""

    def _cleanup_device(self):
        """Optional: device-specific cleanup (called after main loop)."""

    def _get_speed_label(self) -> str:
        """Return speed label for status display. Override in subclass."""
        if hasattr(self, 'speed_scale'):
            return f"{self.speed_scale:.1f}x"
        return ""

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        """Main sender loop: query pose → read input → accumulate → send."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"[Sender] Target: {self._target_ip}:{self._port} at {self._hz} Hz")
        print("[Sender] Querying robot for initial pose...")

        pos, quat = self.query_initial_pose()
        self._virtual_pos = pos.copy()
        self._virtual_quat = quat.copy()

        # Switch to non-blocking for the main loop
        self._sock.setblocking(False)

        self._setup_device()

        send_count = 0
        print("[Sender] Running. Press Q or Ctrl+C to stop.")

        try:
            while True:
                t_start = time.perf_counter()

                result = self._read_input()

                if result.quit or result.buttons.quit:
                    print("\n[Sender] Quit requested")
                    break

                # Handle reset: re-query robot pose
                if result.buttons.reset:
                    print("\n[Sender] Reset — re-querying robot pose...")
                    self._sock.setblocking(True)
                    pos, quat = self.query_initial_pose()
                    self._virtual_pos = pos.copy()
                    self._virtual_quat = quat.copy()
                    self._sock.setblocking(False)
                    # Still send reset button to robot
                    self._send_packet(result.buttons)
                    continue

                # Accumulate delta
                self._apply_delta(result.delta_pos, result.delta_rot_axis_angle)

                # Send
                self._send_packet(result.buttons)

                send_count += 1
                # Real-time single-line status (overwrite with \r)
                p = self._virtual_pos
                print(f"\r[Sender] #{send_count:>7d}  "
                      f"pos=[{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}]  "
                      f"spd={self._get_speed_label()}",
                      end="", flush=True)

                # Rate control
                elapsed = time.perf_counter() - t_start
                remaining = self._dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            print(f"\n[Sender] Stopped. Total packets sent: {send_count}")
        finally:
            self._cleanup_device()
            if self._sock:
                self._sock.close()
                self._sock = None


# ---------------------------------------------------------------------------
# Quaternion utilities (wxyz convention, no external deps)
# ---------------------------------------------------------------------------

def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2. Both in wxyz format."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _axis_angle_to_quat(axis: np.ndarray, angle: float) -> np.ndarray:
    """Convert axis-angle to quaternion (wxyz)."""
    half = angle / 2.0
    s = math.sin(half)
    return np.array([math.cos(half), axis[0]*s, axis[1]*s, axis[2]*s])
