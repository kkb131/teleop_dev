#!/usr/bin/env python3
"""Keyboard teleop sender — run on the operator PC.

Reads keyboard input via termios (non-blocking), accumulates velocity
deltas into a virtual target pose, and sends absolute pose via the
unified teleop protocol.

Requirements: numpy
    pip install numpy

Usage:
    python3 -m vive.keyboard_sender --target-ip <ROBOT_PC_IP>
    python3 -m vive.keyboard_sender --target-ip 192.168.0.10 --hz 50
"""

import argparse
import select
import sys
import termios
import tty

import numpy as np

from teleop_dev.operator.arm.sender_base import InputResult, TeleopSenderBase

from teleop_dev.protocol.arm_protocol import ButtonState


# Key → direction mapping (matches standalone/core/input_handler.py KEY_MAP)
# [dx, dy, dz, drx, dry, drz] — unit direction
KEY_MAP = {
    "w": [0, 1, 0, 0, 0, 0],    # +Y (forward)
    "s": [0, -1, 0, 0, 0, 0],   # -Y (backward)
    "a": [-1, 0, 0, 0, 0, 0],   # -X (left)
    "d": [1, 0, 0, 0, 0, 0],    # +X (right)
    "q": [0, 0, 1, 0, 0, 0],    # +Z (up)
    "e": [0, 0, -1, 0, 0, 0],   # -Z (down)
    "u": [0, 0, 0, 1, 0, 0],    # +Roll
    "o": [0, 0, 0, -1, 0, 0],   # -Roll
    "i": [0, 0, 0, 0, 1, 0],    # +Pitch
    "k": [0, 0, 0, 0, -1, 0],   # -Pitch
    "j": [0, 0, 0, 0, 0, 1],    # +Yaw
    "l": [0, 0, 0, 0, 0, -1],   # -Yaw
}

# Speed scale presets
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3x


class KeyboardSender(TeleopSenderBase):
    """Keyboard-based teleop sender using termios (non-blocking).

    Each key press generates a single-step delta. Hold a key for repeated
    deltas via terminal key repeat.
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50,
                 cartesian_step: float = 0.005,
                 rotation_step: float = 0.05):
        super().__init__(target_ip, port, hz)
        self._cart_step = cartesian_step    # metres per tick
        self._rot_step = rotation_step      # radians per tick
        self._speed_idx = DEFAULT_SPEED_IDX
        self._old_settings = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def _setup_device(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print("[KeyboardSender] Controls:")
        print("  W/S=Y  A/D=X  Q/E=Z  U/O=Roll  I/K=Pitch  J/L=Yaw")
        print("  Space=E-Stop  R=Reset  X/Esc=Quit  +/-=Speed")
        print(f"  Speed: {self.speed_scale:.1f}x")

    def _cleanup_device(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _read_key(self) -> str | None:
        """Non-blocking single character read."""
        if select.select([sys.stdin], [], [], 0)[0]:
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

    def _read_input(self) -> InputResult:
        result = InputResult()

        # Read key (drain buffer, keep latest)
        key = self._read_key()
        if key is not None:
            while True:
                next_key = self._read_key()
                if next_key is None:
                    break
                key = next_key

        if key is None:
            return result

        buttons = ButtonState()

        # Control keys
        if key in ("x", "ESC"):
            buttons.quit = True
            result.buttons = buttons
            return result

        if key == " ":
            buttons.estop = True
            result.buttons = buttons
            return result

        if key == "r":
            buttons.reset = True
            result.buttons = buttons
            return result

        # Speed adjustment
        if key in ("+", "="):
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            buttons.speed_up = True
            print(f"\n[KeyboardSender] Speed: {self.speed_scale:.1f}x")
            result.buttons = buttons
            return result
        if key == "-":
            self._speed_idx = max(self._speed_idx - 1, 0)
            buttons.speed_down = True
            print(f"\n[KeyboardSender] Speed: {self.speed_scale:.1f}x")
            result.buttons = buttons
            return result

        # Admittance controls
        if key == "t":
            buttons.admittance_toggle = True
            result.buttons = buttons
            return result
        if key == "z":
            buttons.ft_zero = True
            result.buttons = buttons
            return result

        # Movement keys
        if key in KEY_MAP:
            direction = np.array(KEY_MAP[key], dtype=float)
            s = self.speed_scale
            result.delta_pos = direction[:3] * self._cart_step * s
            result.delta_rot_axis_angle = direction[3:] * self._rot_step * s

        result.buttons = buttons
        return result


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop sender (unified protocol)")
    parser.add_argument("--target-ip", required=True,
                        help="Robot PC IP address")
    parser.add_argument("--port", type=int, default=9871,
                        help="UDP port (default: 9871)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Send rate in Hz (default: 50)")
    parser.add_argument("--cart-step", type=float, default=0.005,
                        help="Cartesian step per tick in metres (default: 0.005)")
    parser.add_argument("--rot-step", type=float, default=0.05,
                        help="Rotation step per tick in radians (default: 0.05)")
    args = parser.parse_args()

    sender = KeyboardSender(
        target_ip=args.target_ip,
        port=args.port,
        hz=args.hz,
        cartesian_step=args.cart_step,
        rotation_step=args.rot_step,
    )
    sender.run()


if __name__ == "__main__":
    main()
