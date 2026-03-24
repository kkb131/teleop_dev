#!/usr/bin/env python3
"""Keyboard teleop with Admittance control via RobotBackend.

Supports two modes:
  --mode rtde : Direct ur_rtde communication + RTDE F/T sensor reading
  --mode sim  : Isaac Sim / mock hardware via ROS2 topics (admittance disabled)

Pipeline:
  Keyboard -> twist -> Pinocchio DLS -> dq_teleop --+
                                                     +-> target_q -> backend
  F/T sensor -> deadzone -> Admittance -> J_pinv ---+

Key mappings:
  W/S       : X forward/backward
  A/D       : Y left/right
  Q/E       : Z up/down
  U/O       : Roll  (RX) +/-
  I/K       : Pitch (RY) +/-
  J/L       : Yaw   (RZ) +/-
  +/=       : Increase speed scale
  -         : Decrease speed scale
  f         : Toggle frame (base_link / tool0)
  p         : Print current EE pose (FK)
  Space     : Stop (hold current position)
  Esc / x   : Quit

  --- Admittance ---
  z         : Zero F/T sensor
  t         : Toggle admittance ON/OFF
  1 / 2 / 3 : Stiff / Medium / Soft compliance preset

Usage:
  python3 -m standalone.servo.keyboard_servo_admittance --mode sim
  python3 -m standalone.servo.keyboard_servo_admittance --mode rtde --robot-ip 192.168.0.2
"""

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty
from typing import Protocol

import numpy as np

from teleop_dev.robot.config import (
    DEFAULT_MODE,
    DEFAULT_ROBOT_IP,
    JOINT_NAMES,
    SERVO_RATE_HZ,
)
from teleop_dev.robot.core.robot_backend import RobotBackend, create_backend
from teleop_dev.robot.core.kinematics import PinocchioIK

# ──────────────────────────── Frames ────────────────────────────
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# ──────────────────────────── Speed ─────────────────────────────
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# ──────────────────────────── Control ───────────────────────────
DAMPING = 0.05

# ──────────────────────── Admittance Presets ────────────────────
PRESETS = {
    'STIFF': {
        'M': np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]),
        'D': np.array([200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),
        'K': np.array([500.0, 500.0, 500.0, 50.0, 50.0, 50.0]),
    },
    'MEDIUM': {
        'M': np.array([5.0, 5.0, 5.0, 0.5, 0.5, 0.5]),
        'D': np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]),
        'K': np.array([200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),
    },
    'SOFT': {
        'M': np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2]),
        'D': np.array([40.0, 40.0, 40.0, 4.0, 4.0, 4.0]),
        'K': np.array([50.0, 50.0, 50.0, 5.0, 5.0, 5.0]),
    },
}
DEFAULT_PRESET = 'MEDIUM'

# ──────────────────────── Safety Limits ─────────────────────────
FORCE_DEADZONE = np.array([3.0, 3.0, 3.0, 0.3, 0.3, 0.3])
MAX_CART_DISP = 0.05      # 5 cm max translation offset
MAX_CART_ROT = 0.15       # ~8.6 deg max rotation offset
FORCE_SATURATION = 100.0  # N
TORQUE_SATURATION = 10.0  # Nm

# ──────────────────────── Key Mappings ──────────────────────────
# UR10e base frame: X=right, Y=forward (away from base), Z=up
KEY_MAP = {
    'w': (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),    # forward  (+Y)
    's': (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),    # backward (-Y)
    'a': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # left     (-X)
    'd': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # right    (+X)
    'q': (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    'e': (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
    'u': (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    'o': (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
    'i': (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    'k': (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
    'j': (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    'l': (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
}

HELP_TEXT = """
╔══════════════════════════════════════════════════════════╗
║   Admittance Cartesian Controller - Keyboard Teleop      ║
║   (Pinocchio DLS + F/T Admittance)                       ║
╠══════════════════════════════════════════════════════════╣
║  --- Translation ---                                     ║
║  W / S     : X forward / backward                        ║
║  A / D     : Y left / right                              ║
║  Q / E     : Z up / down                                 ║
║                                                          ║
║  --- Rotation ---                                        ║
║  U / O     : Roll  (RX) +/-                              ║
║  I / K     : Pitch (RY) +/-                              ║
║  J / L     : Yaw   (RZ) +/-                              ║
║                                                          ║
║  --- Control ---                                         ║
║  + / =     : Increase speed                              ║
║  -         : Decrease speed                              ║
║  f         : Toggle frame (base_link / tool0)            ║
║  p         : Print EE pose (FK)                          ║
║  Space     : Stop                                        ║
║  Esc / x   : Quit                                        ║
║                                                          ║
║  --- Admittance ---                                      ║
║  z         : Zero F/T sensor                             ║
║  t         : Toggle admittance ON/OFF                    ║
║  1 / 2 / 3 : Stiff / Medium / Soft compliance            ║
╚══════════════════════════════════════════════════════════╝
"""


# ──────────────────── F/T Source Abstraction ────────────────────


class FTSource(Protocol):
    """Protocol for F/T sensor data source."""
    def get_wrench(self) -> np.ndarray: ...
    def zero_sensor(self) -> None: ...


class RTDEFTSource:
    """Read F/T data directly from ur_rtde."""

    def __init__(self, backend):
        self._backend = backend
        self._bias = np.zeros(6)

    def get_wrench(self) -> np.ndarray:
        raw = np.array(self._backend.get_tcp_force())
        return raw - self._bias

    def zero_sensor(self) -> None:
        self._bias = np.array(self._backend.get_tcp_force())
        print('\r  >>> F/T sensor zeroed (RTDE)          ')


class NullFTSource:
    """Dummy F/T source for sim mode — always returns zero."""

    def get_wrench(self) -> np.ndarray:
        return np.zeros(6)

    def zero_sensor(self) -> None:
        print('\r  >>> F/T zeroed (sim — no sensor)          ')


# ──────────────────── Main Controller Class ─────────────────────


class KeyboardAdmittance:
    def __init__(self, backend: RobotBackend, ft_source: FTSource):
        self.backend = backend
        self.ft_source = ft_source

        # State
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False
        self.current_twist = np.zeros(6)
        self.current_positions = None

        # Pinocchio IK
        self.ik = PinocchioIK()
        print(f'Pinocchio loaded: {self.ik.nq} joints, EE frame_id={self.ik.ee_frame_id}')

        # Admittance state
        self.admittance_enabled = not isinstance(ft_source, NullFTSource)
        self.preset_name = DEFAULT_PRESET
        self._load_preset(DEFAULT_PRESET)
        self.admittance_x = np.zeros(6)
        self.admittance_xdot = np.zeros(6)

        # Terminal
        self._old_settings = None

    # ─────────────────── Properties ─────────────────────

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    @property
    def frame_name(self) -> str:
        return EE_FRAME if self.use_local_frame else BASE_FRAME

    # ─────────────────── Admittance ─────────────────────

    def _load_preset(self, name: str):
        p = PRESETS[name]
        self.M = p['M'].copy()
        self.D = p['D'].copy()
        self.K = p['K'].copy()
        self.preset_name = name

    def _transform_wrench_to_base(self, wrench_tool: np.ndarray,
                                   R: np.ndarray) -> np.ndarray:
        f_base = R @ wrench_tool[:3]
        t_base = R @ wrench_tool[3:]
        return np.concatenate([f_base, t_base])

    def _apply_deadzone(self, wrench: np.ndarray) -> np.ndarray:
        result = wrench.copy()
        mask = np.abs(result) < FORCE_DEADZONE
        result[mask] = 0.0
        result[~mask] -= np.sign(result[~mask]) * FORCE_DEADZONE[~mask]
        return result

    def _check_saturation(self, wrench: np.ndarray) -> bool:
        force_mag = np.linalg.norm(wrench[:3])
        torque_mag = np.linalg.norm(wrench[3:])
        return force_mag > FORCE_SATURATION or torque_mag > TORQUE_SATURATION

    def _update_admittance(self, dt: float):
        if not self.admittance_enabled or self.current_positions is None:
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            return

        _, R = self.ik.get_ee_pose(self.current_positions)
        f_ext = self._transform_wrench_to_base(self.ft_source.get_wrench(), R)

        if self._check_saturation(f_ext):
            print(f'\r  WARNING: F/T saturation! F={np.linalg.norm(f_ext[:3]):.1f}N '
                  f'T={np.linalg.norm(f_ext[3:]):.1f}Nm          ')
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            return

        f_ext = self._apply_deadzone(f_ext)

        xddot = (f_ext - self.D * self.admittance_xdot
                 - self.K * self.admittance_x) / self.M

        self.admittance_xdot += xddot * dt
        self.admittance_x += self.admittance_xdot * dt

        disp_norm = np.linalg.norm(self.admittance_x[:3])
        if disp_norm > MAX_CART_DISP:
            self.admittance_x[:3] *= MAX_CART_DISP / disp_norm
            self.admittance_xdot[:3] *= 0.5

        rot_norm = np.linalg.norm(self.admittance_x[3:])
        if rot_norm > MAX_CART_ROT:
            self.admittance_x[3:] *= MAX_CART_ROT / rot_norm
            self.admittance_xdot[3:] *= 0.5

    def _compute_admittance_dq(self, q: np.ndarray, dt: float) -> np.ndarray:
        if not self.admittance_enabled:
            return np.zeros(6)
        J = self.ik.get_jacobian(q, local=False)
        JJt = J @ J.T + (DAMPING ** 2) * np.eye(6)
        return J.T @ np.linalg.solve(JJt, self.admittance_xdot * dt)

    # ─────────────────── Terminal ───────────────────────

    def setup_terminal(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def get_key(self, timeout: float = 0.02) -> str | None:
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            sys.stdin.read(1)
                return 'ESC'
            return ch
        return None

    # ─────────────────── Display ────────────────────────

    def print_ee_pose(self):
        if self.current_positions is None:
            print('\r  [No joint states received yet]')
            return
        pos, _ = self.ik.get_ee_pose(self.current_positions)
        rpy = self.ik.get_ee_rpy(self.current_positions)
        print('\r' + ' ' * 80, end='')
        print(f'\r  EE Position: x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f} [m]')
        print(f'  EE Rotation: R={math.degrees(rpy[0]):.1f}  '
              f'P={math.degrees(rpy[1]):.1f}  Y={math.degrees(rpy[2]):.1f}')
        print(f'  Joints: [{", ".join(f"{math.degrees(j):.1f}" for j in self.current_positions)}]')

    def print_admittance_status(self):
        state = 'ON' if self.admittance_enabled else 'OFF'
        f = self.ft_source.get_wrench()
        dx = self.admittance_x
        print(
            f'\r  Admittance: {state} [{self.preset_name}] | '
            f'F: [{f[0]:.1f}, {f[1]:.1f}, {f[2]:.1f}]N | '
            f'dx: [{dx[0]*1000:.1f}, {dx[1]*1000:.1f}, {dx[2]*1000:.1f}]mm'
            '          ', end='', flush=True
        )

    # ─────────────────── Key Processing ─────────────────

    def process_key(self, key: str):
        if key in ('x', 'ESC'):
            self.running = False
            return

        if key in KEY_MAP:
            self.current_twist = np.array(KEY_MAP[key])
            return

        if key in ('+', '='):
            self.speed_idx = min(self.speed_idx + 1, len(SPEED_SCALES) - 1)
            print(f'\r  Speed: {self.speed_scale:.1f}          ')
            return
        if key == '-':
            self.speed_idx = max(self.speed_idx - 1, 0)
            print(f'\r  Speed: {self.speed_scale:.1f}          ')
            return

        if key == 'f':
            self.use_local_frame = not self.use_local_frame
            print(f'\r  Frame: {self.frame_name}          ')
            return

        if key == 'p':
            self.print_ee_pose()
            return

        if key == ' ':
            self.current_twist = np.zeros(6)
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print('\r  >>> STOP                    ')
            return

        # Admittance controls
        if key == 'z':
            self.ft_source.zero_sensor()
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            return

        if key == 't':
            if isinstance(self.ft_source, NullFTSource):
                print('\r  Admittance not available in sim mode          ')
                return
            self.admittance_enabled = not self.admittance_enabled
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            state = 'ON' if self.admittance_enabled else 'OFF'
            print(f'\r  Admittance: {state} [{self.preset_name}]          ')
            return

        if key in ('1', '2', '3'):
            preset = {'1': 'STIFF', '2': 'MEDIUM', '3': 'SOFT'}[key]
            self._load_preset(preset)
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print(f'\r  Preset: {preset}          ')
            return

    # ─────────────────── Main Loop ──────────────────────

    def update_positions(self):
        positions = self.backend.get_joint_positions()
        self.current_positions = np.array(positions)

    def send_command(self, positions: np.ndarray):
        self.backend.send_joint_command(positions.tolist())

    def run(self):
        dt = 1.0 / SERVO_RATE_HZ

        # Wait for valid joint state
        print('Waiting for joint states...')
        for _ in range(100):
            self.update_positions()
            if self.current_positions is not None and np.any(self.current_positions != 0.0):
                break
            time.sleep(0.1)

        if self.current_positions is None:
            print('ERROR: No joint states received. Exiting.')
            return

        admittance_state = 'ON' if self.admittance_enabled else 'OFF (no F/T sensor)'
        print('Joint states received. Starting admittance teleop.')
        print(HELP_TEXT)
        print(f'  Speed: {self.speed_scale:.1f}  |  Frame: {self.frame_name}  |  '
              f'Admittance: {admittance_state} [{self.preset_name}]')
        print('  Ready! Press keys to move the robot.\n')

        self.setup_terminal()
        status_counter = 0

        try:
            while self.running:
                self.update_positions()

                key = self.get_key(timeout=dt)
                if key:
                    self.process_key(key)

                if self.current_positions is None:
                    time.sleep(dt)
                    continue

                current_q = self.current_positions

                # 1) Keyboard teleop -> DLS -> joint delta
                if np.any(self.current_twist != 0.0):
                    twist = self.current_twist * self.speed_scale
                    dq_teleop = self.ik.compute_joint_delta(
                        current_q, twist, dt,
                        damping=DAMPING, local=self.use_local_frame,
                    )
                    self.current_twist = np.zeros(6)
                else:
                    dq_teleop = np.zeros(6)

                # 2) Admittance dynamics update
                self._update_admittance(dt)

                # 3) Convert admittance velocity to joint delta
                dq_admittance = self._compute_admittance_dq(current_q, dt)

                # 4) Combine and send
                target = self.ik.clamp_positions(
                    current_q + dq_teleop + dq_admittance
                )
                self.send_command(target)

                # Periodic status display (~2Hz)
                status_counter += 1
                if status_counter >= SERVO_RATE_HZ // 2:
                    status_counter = 0
                    self.print_admittance_status()

                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            print('\n\nDone.')


def main():
    parser = argparse.ArgumentParser(description='Admittance Cartesian keyboard teleop')
    parser.add_argument('--mode', choices=['rtde', 'sim'], default=DEFAULT_MODE,
                        help='Backend mode (default: %(default)s)')
    parser.add_argument('--robot-ip', default=DEFAULT_ROBOT_IP,
                        help='Robot IP for RTDE mode (default: %(default)s)')
    args = parser.parse_args()

    print(f'[keyboard_servo_admittance] mode={args.mode}')
    backend = create_backend(args.mode, robot_ip=args.robot_ip)

    def signal_handler(sig, frame):
        ctrl.running = False

    with backend:
        # Create F/T source based on mode
        if args.mode == 'rtde':
            ft_source = RTDEFTSource(backend)
        else:
            ft_source = NullFTSource()
            print('  NOTE: F/T sensor not available in sim mode. '
                  'Admittance control disabled.')

        ctrl = KeyboardAdmittance(backend, ft_source)
        signal.signal(signal.SIGINT, signal_handler)
        ctrl.run()


if __name__ == '__main__':
    main()
