#!/usr/bin/env python3
"""UDP Vive Tracker sender — run on the operator PC with SteamVR.

Reads Vive Tracker 3.0 pose via OpenVR, applies calibration transform
(SteamVR → robot base_link), and sends absolute target pose via the
unified teleop protocol.

Uses relative mapping: starts at the robot's current TCP pose (queried
at startup) and applies tracker movement deltas.

Requirements: openvr, numpy, pynput, pyyaml
    pip install openvr numpy pynput pyyaml

Usage:
    python3 -m sender.arm.vive_sender --target-ip <ROBOT_PC_IP>
    python3 -m sender.arm.vive_sender --config sender/arm/config/default.yaml
    python3 -m sender.arm.vive_sender --config sender/arm/config/default.yaml --target-ip 10.0.0.5
"""

import argparse
import threading

import numpy as np

from sender.arm.calibrate import load_calibration, transform_pose
from sender.arm.sender_base import (
    InputResult,
    TeleopSenderBase,
    _quat_multiply,
)
from sender.arm.vive_config import ViveConfig
from sender.arm.vive_tracker import ViveTracker

from protocol.arm_protocol import ButtonState

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


class KeyboardState:
    """Thread-safe keyboard state tracker using pynput.

    Key mappings:
        Space   = E-Stop
        R       = Reset
        Q/Esc   = Quit
        +/=     = Speed up
        -       = Speed down
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._estop = False
        self._reset = False
        self._quit = False
        self._speed_up = False
        self._speed_down = False
        self._listener = None

    def start(self):
        if keyboard is None:
            raise ImportError(
                "pynput not installed. Run: pip install pynput"
            )
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.daemon = True
        self._listener.start()
        print("[Keyboard] Listener started")
        print("[Keyboard] Space=E-Stop, R=Reset, Q/Esc=Quit, +/-=Speed")

    def stop(self):
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    def get_and_clear(self) -> ButtonState:
        """Get current button state and clear edge-triggered flags."""
        with self._lock:
            state = ButtonState(
                estop=self._estop,
                reset=self._reset,
                quit=self._quit,
                speed_up=self._speed_up,
                speed_down=self._speed_down,
            )
            self._estop = False
            self._reset = False
            self._quit = False
            self._speed_up = False
            self._speed_down = False
        return state

    def _on_press(self, key):
        with self._lock:
            try:
                if key == keyboard.Key.space:
                    self._estop = True
                elif key == keyboard.Key.esc:
                    self._quit = True
                elif hasattr(key, "char") and key.char is not None:
                    ch = key.char.lower()
                    if ch == "r":
                        self._reset = True
                    elif ch == "q":
                        self._quit = True
                    elif ch in ("+", "="):
                        self._speed_up = True
                    elif ch == "-":
                        self._speed_down = True
            except AttributeError:
                pass

    def _on_release(self, key):
        pass


class ViveSender(TeleopSenderBase):
    """Vive Tracker sender using unified teleop protocol.

    Relative mapping: tracks delta movement of the Vive Tracker and
    applies it to a virtual pose that starts at the robot's current TCP.

    The calibration transform (SteamVR → robot base_link) is applied
    to both the current and previous tracker poses before computing
    the delta, so the delta is in robot-frame coordinates.
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50,
                 tracker_serial: str = None,
                 calibration_file: str = None):
        super().__init__(target_ip, port, hz)
        self._tracker_serial = tracker_serial
        self._calibration_file = calibration_file
        self._tracker: ViveTracker = None
        self._kb: KeyboardState = None

        # Calibration
        self._cal_R = None  # (3,3)
        self._cal_t = None  # (3,)

        # Previous frame (in robot coords, for delta computation)
        self._prev_pos_robot = None   # (3,)
        self._prev_quat_robot = None  # (4,) wxyz

    def _setup_device(self):
        # Tracker
        self._tracker = ViveTracker(tracker_serial=self._tracker_serial)
        self._tracker.connect()
        print(f"[ViveSender] Tracker connected (serial={self._tracker_serial})")

        # Calibration
        if self._calibration_file:
            self._cal_R, self._cal_t = load_calibration(self._calibration_file)
            print(f"[ViveSender] Calibration loaded from {self._calibration_file}")
        else:
            # Default: SteamVR Y-up → UR Z-up
            self._cal_R = np.array([
                [1.0,  0.0,  0.0],
                [0.0,  0.0, -1.0],
                [0.0,  1.0,  0.0],
            ])
            self._cal_t = np.zeros(3)
            print("[ViveSender] No calibration file — using default Y-up→Z-up mapping")

        # Keyboard
        self._kb = KeyboardState()
        self._kb.start()

    def _cleanup_device(self):
        if self._kb:
            self._kb.stop()
        if self._tracker:
            self._tracker.disconnect()

    def _read_input(self) -> InputResult:
        result = InputResult()
        result.buttons = self._kb.get_and_clear()

        # Read tracker pose
        pose = self._tracker.get_pose()
        if pose is None:
            # Tracking lost — send zero delta
            self._prev_pos_robot = None
            self._prev_quat_robot = None
            return result

        pos_vive, quat_vive = pose  # quat is wxyz

        # Transform to robot frame
        pos_robot, quat_robot = transform_pose(
            pos_vive, quat_vive, self._cal_R, self._cal_t
        )

        # Compute delta from previous frame
        if self._prev_pos_robot is not None:
            result.delta_pos = pos_robot - self._prev_pos_robot

            # Rotation delta: dq = q_curr * q_prev_inv
            q_prev_inv = _quat_conjugate(self._prev_quat_robot)
            dq = _quat_multiply(quat_robot, q_prev_inv)
            # Convert delta quaternion to axis-angle
            result.delta_rot_axis_angle = _quat_to_axis_angle(dq)

        self._prev_pos_robot = pos_robot
        self._prev_quat_robot = quat_robot

        return result


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate (inverse for unit quaternion). wxyz format."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_to_axis_angle(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (wxyz) to axis * angle (radians)."""
    import math
    w = np.clip(q[0], -1.0, 1.0)
    angle = 2.0 * math.acos(abs(w))
    if angle < 1e-8:
        return np.zeros(3)
    sin_half = math.sin(angle / 2.0)
    axis = q[1:4] / sin_half
    # Ensure shortest path
    if w < 0:
        axis = -axis
        angle = 2.0 * math.pi - angle
    return axis * angle


def main():
    parser = argparse.ArgumentParser(description="UDP Vive Tracker sender (unified protocol)")
    parser.add_argument("--config", default=None,
                        help="YAML config file (default: sender/arm/config/default.yaml)")
    parser.add_argument("--target-ip", default=None,
                        help="Robot PC IP (overrides config)")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP port (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Send rate in Hz (overrides config)")
    parser.add_argument("--tracker-serial", default=None,
                        help="Tracker serial (overrides config)")
    parser.add_argument("--calibration", default=None,
                        help="Calibration JSON file path")
    parser.add_argument("--list-trackers", action="store_true",
                        help="List all trackers and exit")
    args = parser.parse_args()

    # Load config
    cfg = ViveConfig.load(args.config)

    # CLI overrides
    target_ip = args.target_ip or cfg.network.target_ip
    port = args.port or cfg.network.port
    hz = args.hz or cfg.network.hz
    teleop_tracker = cfg.get_teleop_tracker()
    tracker_serial = args.tracker_serial or teleop_tracker.serial
    calibration_file = args.calibration or cfg.calibration.file

    if args.list_trackers:
        tracker = ViveTracker(tracker_serial=tracker_serial)
        tracker.connect()
        trackers = tracker.get_all_trackers()
        if not trackers:
            print("[Sender] No trackers found")
        for t in trackers:
            status = "TRACKING" if t["tracking"] else "NOT TRACKING"
            print(f"  index={t['index']}  serial={t['serial']}  [{status}]")
        tracker.disconnect()
        return

    if target_ip is None:
        print("[ERROR] --target-ip required (or set network.target_ip in config)")
        return

    print(f"[ViveSender] Config: tracker={teleop_tracker.name} serial={tracker_serial}")

    sender = ViveSender(
        target_ip=target_ip,
        port=port,
        hz=hz,
        tracker_serial=tracker_serial,
        calibration_file=calibration_file,
    )
    sender.run()


if __name__ == "__main__":
    main()
