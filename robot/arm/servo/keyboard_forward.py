#!/usr/bin/env python3
"""Keyboard teleop — direct joint-space control via RobotBackend.

Supports two modes:
  --mode rtde : Direct ur_rtde communication (no ROS2 needed)
  --mode sim  : Isaac Sim / mock hardware via ROS2 topics (SimBackend)

Key mappings:
  1-6     : Select joint 1~6
  w / UP  : Increase selected joint position
  s / DOWN: Decrease selected joint position
  +/=     : Increase step size
  -       : Decrease step size
  h       : Go to home position
  p       : Print current joint states
  Space   : Stop (hold current position)
  Esc / q : Quit

Usage:
  python3 -m standalone.servo.keyboard_forward --mode sim
  python3 -m standalone.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2
"""

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty

from teleop_dev.robot.config import (
    DEFAULT_MODE,
    DEFAULT_ROBOT_IP,
    JOINT_NAMES,
    SERVO_RATE_HZ,
)
from teleop_dev.robot.core.robot_backend import create_backend

# Step sizes (radians)
STEP_SIZES = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]
DEFAULT_STEP_IDX = 2  # 0.01 rad

# Home position
HOME_POSITION = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

HELP_TEXT = """
╔═══════════════════════════════════════════════════╗
║     Joint-Space Keyboard Teleop                    ║
╠═══════════════════════════════════════════════════╣
║  1-6       : Select joint                          ║
║  w / UP    : Increase selected joint (+step)       ║
║  s / DOWN  : Decrease selected joint (-step)       ║
║  + / =     : Increase step size                    ║
║  -         : Decrease step size                    ║
║  h         : Go to home position                   ║
║  p         : Print current joint states            ║
║  Space     : Stop (hold current position)          ║
║  Esc / q   : Quit                                  ║
╚═══════════════════════════════════════════════════╝
"""


class KeyboardForward:
    def __init__(self, backend):
        self.backend = backend
        self.current_positions = None
        self.target_positions = None
        self.selected_joint = 0
        self.step_idx = DEFAULT_STEP_IDX
        self.running = True
        self._old_settings = None

    @property
    def step_size(self) -> float:
        return STEP_SIZES[self.step_idx]

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
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A':
                                return 'UP'
                            elif ch3 == 'B':
                                return 'DOWN'
                            elif ch3 == 'C':
                                return 'RIGHT'
                            elif ch3 == 'D':
                                return 'LEFT'
                return 'ESC'
            return ch
        return None

    def update_positions(self):
        positions = self.backend.get_joint_positions()
        self.current_positions = list(positions)
        if self.target_positions is None:
            self.target_positions = list(positions)

    def send_command(self):
        if self.target_positions is not None:
            self.backend.send_joint_command(self.target_positions)

    def print_status(self):
        if self.current_positions is None:
            print('\r  [No joint states received yet]')
            return

        print('\r' + ' ' * 80, end='')
        print(f'\r  Joint {self.selected_joint + 1} ({JOINT_NAMES[self.selected_joint]})'
              f'  Step: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')

        for i in range(6):
            marker = '>>>' if i == self.selected_joint else '   '
            cur = self.current_positions[i] if self.current_positions else 0.0
            tgt = self.target_positions[i] if self.target_positions else 0.0
            print(f'  {marker} J{i+1} ({JOINT_NAMES[i]:>22s}): '
                  f'cur={math.degrees(cur):7.2f}°  tgt={math.degrees(tgt):7.2f}°')

    def process_key(self, key: str):
        if key in ('q', 'ESC'):
            self.running = False
            return

        if self.target_positions is None:
            return

        if key in '123456':
            self.selected_joint = int(key) - 1
            self.print_status()
            return

        if key in ('w', 'UP'):
            self.target_positions[self.selected_joint] += self.step_size
            self.print_status()
            return
        if key in ('s', 'DOWN'):
            self.target_positions[self.selected_joint] -= self.step_size
            self.print_status()
            return

        if key in ('+', '='):
            self.step_idx = min(self.step_idx + 1, len(STEP_SIZES) - 1)
            print(f'\r  Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')
            return
        if key == '-':
            self.step_idx = max(self.step_idx - 1, 0)
            print(f'\r  Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')
            return

        if key == 'h':
            self.target_positions = list(HOME_POSITION)
            print('\r  >>> Moving to HOME position')
            return

        if key == 'p':
            self.print_status()
            return

        if key == ' ':
            if self.current_positions:
                self.target_positions = list(self.current_positions)
                print('\r  >>> Holding current position')
            return

    def run(self):
        dt = 1.0 / SERVO_RATE_HZ

        # Wait for valid joint state
        print('Waiting for joint states...')
        for _ in range(100):  # ~10s timeout
            self.update_positions()
            if self.current_positions and any(p != 0.0 for p in self.current_positions):
                break
            time.sleep(0.1)

        if self.current_positions is None:
            print('ERROR: No joint states received. Exiting.')
            return

        print(f'Joint states received. Mode ready.')
        print(HELP_TEXT)
        self.print_status()

        self.setup_terminal()
        try:
            while self.running:
                self.update_positions()

                key = self.get_key(timeout=dt)
                if key:
                    self.process_key(key)

                self.send_command()
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            print('\n\nDone.')


def main():
    parser = argparse.ArgumentParser(description='Joint-space keyboard teleop')
    parser.add_argument('--mode', choices=['rtde', 'sim'], default=DEFAULT_MODE,
                        help='Backend mode (default: %(default)s)')
    parser.add_argument('--robot-ip', default=DEFAULT_ROBOT_IP,
                        help='Robot IP for RTDE mode (default: %(default)s)')
    args = parser.parse_args()

    print(f'[keyboard_forward] mode={args.mode}')
    backend = create_backend(args.mode, robot_ip=args.robot_ip)

    def signal_handler(sig, frame):
        ctrl.running = False

    with backend:
        ctrl = KeyboardForward(backend)
        signal.signal(signal.SIGINT, signal_handler)
        ctrl.run()


if __name__ == '__main__':
    main()
