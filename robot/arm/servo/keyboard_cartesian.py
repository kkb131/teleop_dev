#!/usr/bin/env python3
"""Keyboard teleop — Cartesian control via Pinocchio DLS IK + RobotBackend.

Supports two modes:
  --mode rtde : Direct ur_rtde communication (no ROS2 needed)
  --mode sim  : Isaac Sim / mock hardware via ROS2 topics (SimBackend)

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

Usage:
  python3 -m standalone.servo.keyboard_cartesian --mode sim
  python3 -m standalone.servo.keyboard_cartesian --mode rtde --robot-ip 192.168.0.2
"""

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty

import numpy as np

from teleop_dev.robot.config import (
    DEFAULT_MODE,
    DEFAULT_ROBOT_IP,
    SERVO_RATE_HZ,
)
from teleop_dev.robot.core.robot_backend import create_backend
from teleop_dev.robot.core.kinematics import PinocchioIK

# Frames
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# Speed scales (applied to twist input)
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# DLS damping factor
DAMPING = 0.05

HELP_TEXT = """
╔════════════════════════════════════════════════════════╗
║   Cartesian Keyboard Teleop (Pinocchio DLS IK)         ║
╠════════════════════════════════════════════════════════╣
║  --- Translation ---                                   ║
║  W / S     : X forward / backward                      ║
║  A / D     : Y left / right                            ║
║  Q / E     : Z up / down                               ║
║                                                        ║
║  --- Rotation ---                                      ║
║  U / O     : Roll  (RX) +/-                            ║
║  I / K     : Pitch (RY) +/-                            ║
║  J / L     : Yaw   (RZ) +/-                            ║
║                                                        ║
║  --- Control ---                                       ║
║  + / =     : Increase speed                            ║
║  -         : Decrease speed                            ║
║  f         : Toggle frame (base_link / tool0)          ║
║  p         : Print EE pose (FK)                        ║
║  Space     : Stop                                      ║
║  Esc / x   : Quit                                      ║
╚════════════════════════════════════════════════════════╝
"""

# Key -> (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
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


class KeyboardCartesian:
    def __init__(self, backend):
        self.backend = backend
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False
        self.current_twist = np.zeros(6)
        self.current_positions = None

        self.ik = PinocchioIK()
        print(f'Pinocchio loaded: {self.ik.nq} joints, EE frame_id={self.ik.ee_frame_id}')

        self._old_settings = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    @property
    def frame_name(self) -> str:
        return EE_FRAME if self.use_local_frame else BASE_FRAME

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
                            sys.stdin.read(1)  # consume arrow key code
                return 'ESC'
            return ch
        return None

    def update_positions(self):
        positions = self.backend.get_joint_positions()
        self.current_positions = np.array(positions)

    def send_command(self, positions: np.ndarray):
        self.backend.send_joint_command(positions.tolist())

    def print_ee_pose(self):
        if self.current_positions is None:
            print('\r  [No joint states received yet]')
            return

        pos, _ = self.ik.get_ee_pose(self.current_positions)
        rpy = self.ik.get_ee_rpy(self.current_positions)

        print('\r' + ' ' * 80, end='')
        print(f'\r  EE Position: x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f} [m]')
        print(f'  EE Rotation: R={math.degrees(rpy[0]):.1f}°  '
              f'P={math.degrees(rpy[1]):.1f}°  Y={math.degrees(rpy[2]):.1f}°')
        print(f'  Joints: [{", ".join(f"{math.degrees(j):.1f}" for j in self.current_positions)}]°')

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
            print('\r  >>> STOP                    ')
            return

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

        print('Joint states received. Starting Cartesian teleop.')
        print(HELP_TEXT)
        print(f'  Speed: {self.speed_scale:.1f}  |  Frame: {self.frame_name}')
        print('  Ready! Press keys to move the robot.\n')

        self.setup_terminal()
        try:
            while self.running:
                self.update_positions()

                key = self.get_key(timeout=dt)
                if key:
                    self.process_key(key)

                if self.current_positions is None:
                    continue

                if np.any(self.current_twist != 0.0):
                    twist = self.current_twist * self.speed_scale
                    dq = self.ik.compute_joint_delta(
                        self.current_positions, twist, dt,
                        damping=DAMPING, local=self.use_local_frame,
                    )
                    target = self.ik.clamp_positions(self.current_positions + dq)
                    self.send_command(target)
                    self.current_twist = np.zeros(6)
                else:
                    self.send_command(self.current_positions)

                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            print('\n\nDone.')


def main():
    parser = argparse.ArgumentParser(description='Cartesian keyboard teleop')
    parser.add_argument('--mode', choices=['rtde', 'sim'], default=DEFAULT_MODE,
                        help='Backend mode (default: %(default)s)')
    parser.add_argument('--robot-ip', default=DEFAULT_ROBOT_IP,
                        help='Robot IP for RTDE mode (default: %(default)s)')
    args = parser.parse_args()

    print(f'[keyboard_cartesian] mode={args.mode}')
    backend = create_backend(args.mode, robot_ip=args.robot_ip)

    def signal_handler(sig, frame):
        ctrl.running = False

    with backend:
        ctrl = KeyboardCartesian(backend)
        signal.signal(signal.SIGINT, signal_handler)
        ctrl.run()


if __name__ == '__main__':
    main()
