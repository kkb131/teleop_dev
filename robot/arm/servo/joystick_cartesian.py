#!/usr/bin/env python3
"""Xbox joystick teleop — Cartesian control via Pinocchio DLS IK + RobotBackend.

Supports two modes:
  --mode rtde : Direct ur_rtde communication (ROS2 only for /joy topic)
  --mode sim  : Isaac Sim / mock hardware via ROS2 topics (SimBackend)

Requires:
  - joy_node running: ros2 run joy joy_node

Xbox controller mapping:
  Left Stick X/Y    : Y/X translation
  Right Stick X/Y   : Yaw/Pitch rotation
  LT / RT (triggers): Z down / up
  LB / RB (bumpers) : Roll -/+
  A button          : Decrease speed
  B button          : Increase speed
  X button          : Toggle frame (base_link / tool0)
  Y button          : Print EE pose (FK)
  Start button      : Quit

Usage:
  python3 -m standalone.servo.joystick_cartesian --mode sim
  python3 -m standalone.servo.joystick_cartesian --mode rtde --robot-ip 192.168.0.2
"""

import argparse
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

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

# Speed scales
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# DLS damping factor
DAMPING = 0.05

# Xbox controller axis/button indices
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_LT = 2
AXIS_RT = 5

BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5
BTN_START = 7

# Deadzone for analog sticks
DEADZONE = 0.1


class _JoySubscriber(Node):
    """Minimal ROS2 node that only subscribes to /joy."""

    def __init__(self):
        super().__init__('joystick_cartesian_joy')
        self.latest_joy = None
        self._sub = self.create_subscription(Joy, '/joy', self._cb, 10)

    def _cb(self, msg: Joy):
        self.latest_joy = msg


class JoystickCartesian:
    def __init__(self, backend, joy_node: _JoySubscriber):
        self.backend = backend
        self.joy_node = joy_node
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False
        self.current_positions = None
        self.current_twist = np.zeros(6)
        self._prev_buttons = []

        self.ik = PinocchioIK()
        print(f'Pinocchio loaded: {self.ik.nq} joints, EE frame_id={self.ik.ee_frame_id}')

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    @property
    def frame_name(self) -> str:
        return EE_FRAME if self.use_local_frame else BASE_FRAME

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < DEADZONE:
            return 0.0
        return value

    def _button_pressed(self, buttons, btn_idx: int) -> bool:
        if btn_idx >= len(buttons):
            return False
        current = buttons[btn_idx]
        prev = self._prev_buttons[btn_idx] if btn_idx < len(self._prev_buttons) else 0
        return current == 1 and prev == 0

    def process_joy(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        # Button edge detection
        if self._button_pressed(buttons, BTN_A):
            self.speed_idx = max(self.speed_idx - 1, 0)
            print(f'  Speed: {self.speed_scale:.1f}')

        if self._button_pressed(buttons, BTN_B):
            self.speed_idx = min(self.speed_idx + 1, len(SPEED_SCALES) - 1)
            print(f'  Speed: {self.speed_scale:.1f}')

        if self._button_pressed(buttons, BTN_X):
            self.use_local_frame = not self.use_local_frame
            print(f'  Frame: {self.frame_name}')

        if self._button_pressed(buttons, BTN_Y):
            if self.current_positions is not None:
                pos, _ = self.ik.get_ee_pose(self.current_positions)
                rpy = self.ik.get_ee_rpy(self.current_positions)
                print(f'  EE: x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f} | '
                      f'R={math.degrees(rpy[0]):.1f} P={math.degrees(rpy[1]):.1f} '
                      f'Y={math.degrees(rpy[2]):.1f}')

        if self._button_pressed(buttons, BTN_START):
            self.running = False

        self._prev_buttons = list(buttons)

        if len(axes) < 6:
            return

        lx = self._apply_deadzone(axes[AXIS_LEFT_STICK_X])
        ly = self._apply_deadzone(axes[AXIS_LEFT_STICK_Y])
        rx = self._apply_deadzone(axes[AXIS_RIGHT_STICK_X])
        ry = self._apply_deadzone(axes[AXIS_RIGHT_STICK_Y])
        lt = (1.0 - axes[AXIS_LT]) / 2.0
        rt = (1.0 - axes[AXIS_RT]) / 2.0

        roll = 0.0
        if BTN_RB < len(buttons) and buttons[BTN_RB]:
            roll = 1.0
        if BTN_LB < len(buttons) and buttons[BTN_LB]:
            roll = -1.0

        self.current_twist = np.array([
            ly, lx, rt - lt, roll, ry, rx,
        ])

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
            rclpy.spin_once(self.joy_node, timeout_sec=0.0)
            if self.current_positions is not None and np.any(self.current_positions != 0.0):
                break
            time.sleep(0.1)

        if self.current_positions is None:
            print('ERROR: No joint states received. Exiting.')
            return

        print(f'Joystick Cartesian teleop ready. Speed: {self.speed_scale:.1f}  '
              f'Frame: {self.frame_name}')
        print('Waiting for /joy messages (run: ros2 run joy joy_node)...')
        print()
        print('  Xbox Controller Mapping (Cartesian):')
        print('  Left Stick    : X/Y translation')
        print('  Right Stick   : Yaw/Pitch rotation')
        print('  LT/RT         : Z down/up')
        print('  LB/RB         : Roll -/+')
        print('  A/B           : Speed -/+')
        print('  X             : Toggle frame')
        print('  Y             : Print EE pose')
        print('  Start         : Quit')
        print()

        try:
            while self.running:
                # Spin ROS2 for /joy subscription
                rclpy.spin_once(self.joy_node, timeout_sec=0.0)

                # Process latest joy message
                if self.joy_node.latest_joy is not None:
                    self.process_joy(self.joy_node.latest_joy)
                    self.joy_node.latest_joy = None

                self.update_positions()

                if self.current_positions is None:
                    time.sleep(dt)
                    continue

                has_input = np.any(np.abs(self.current_twist) > 0.01)
                if has_input:
                    twist = self.current_twist * self.speed_scale
                    dq = self.ik.compute_joint_delta(
                        self.current_positions, twist, dt,
                        damping=DAMPING, local=self.use_local_frame,
                    )
                    target = self.ik.clamp_positions(self.current_positions + dq)
                    self.send_command(target)
                else:
                    self.send_command(self.current_positions)

                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            print('\nDone.')


def main():
    parser = argparse.ArgumentParser(description='Xbox joystick Cartesian teleop')
    parser.add_argument('--mode', choices=['rtde', 'sim'], default=DEFAULT_MODE,
                        help='Backend mode (default: %(default)s)')
    parser.add_argument('--robot-ip', default=DEFAULT_ROBOT_IP,
                        help='Robot IP for RTDE mode (default: %(default)s)')
    args = parser.parse_args()

    print(f'[joystick_cartesian] mode={args.mode}')

    # ROS2 is needed in both modes for /joy topic
    if not rclpy.ok():
        rclpy.init()
    joy_node = _JoySubscriber()

    backend = create_backend(args.mode, robot_ip=args.robot_ip)

    def signal_handler(sig, frame):
        ctrl.running = False

    with backend:
        ctrl = JoystickCartesian(backend, joy_node)
        signal.signal(signal.SIGINT, signal_handler)
        ctrl.run()

    joy_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
