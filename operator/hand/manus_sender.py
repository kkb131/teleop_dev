#!/usr/bin/env python3
"""UDP Manus glove data sender — run on the operator PC with Manus gloves.

Reads finger joint angles and wrist pose from Manus Quantum Metagloves
via the Manus SDK, then sends combined data over UDP to the robot PC.

Requirements: numpy, pyyaml, pynput
    pip install numpy pyyaml pynput

Usage:
    python3 -m manus.manus_sender --target-ip <ROBOT_PC_IP>
    python3 -m manus.manus_sender --config manus/config/default.yaml
    python3 -m manus.manus_sender --target-ip 10.0.0.5 --hand both
"""

import argparse
import json
import socket
import threading
import time

from teleop_dev.operator.hand.manus_config import ManusConfig
from teleop_dev.operator.hand.manus_reader import ManusReader

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

    def get_and_clear(self) -> dict:
        """Get current button state and clear edge-triggered flags."""
        with self._lock:
            state = {
                "estop": self._estop,
                "reset": self._reset,
                "quit": self._quit,
                "speed_up": self._speed_up,
                "speed_down": self._speed_down,
            }
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


def _build_packet(data, buttons: dict) -> dict:
    """Build a JSON-serializable UDP packet from HandData."""
    return {
        "type": "manus",
        "hand": data.hand_side,
        "joint_angles": data.joint_angles.tolist(),
        "finger_spread": data.finger_spread.tolist(),
        "wrist_pos": data.wrist_pos.tolist(),
        "wrist_quat": data.wrist_quat.tolist(),
        "tracking": True,
        "buttons": buttons,
        "timestamp": time.time(),
    }


def _build_null_packet(hand_side: str, buttons: dict) -> dict:
    """Build a null packet when no data is available."""
    return {
        "type": "manus",
        "hand": hand_side,
        "joint_angles": [0.0] * 20,
        "finger_spread": [0.0] * 5,
        "wrist_pos": [0.0, 0.0, 0.0],
        "wrist_quat": [1.0, 0.0, 0.0, 0.0],
        "tracking": False,
        "buttons": buttons,
        "timestamp": time.time(),
    }


def main():
    parser = argparse.ArgumentParser(description="UDP Manus glove data sender")
    parser.add_argument("--config", default=None,
                        help="YAML config file (default: manus/config/default.yaml)")
    parser.add_argument("--target-ip", default=None,
                        help="Robot PC IP (overrides config)")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP port (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Send rate in Hz (overrides config)")
    parser.add_argument("--hand", default=None,
                        choices=["left", "right", "both"],
                        help="Which hand(s) to stream (overrides config)")
    parser.add_argument("--sdk-path", default=None,
                        help="Path to SDKClient_Linux.out (overrides config)")
    args = parser.parse_args()

    # Load config
    cfg = ManusConfig.load(args.config)

    # CLI overrides
    target_ip = args.target_ip or cfg.network.target_ip
    port = args.port or cfg.network.port
    hz = args.hz or cfg.network.hz
    hand_side = args.hand or cfg.hand.side
    sdk_path = args.sdk_path or cfg.sdk.bin_path

    if target_ip is None:
        print("[ERROR] --target-ip required (or set network.target_ip in config)")
        return

    # Connect to gloves
    reader = ManusReader(sdk_bin_path=sdk_path, hand_side=hand_side)
    reader.connect()

    # Start keyboard listener
    kb = KeyboardState()
    kb.start()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_ip, port)
    dt = 1.0 / hz
    send_count = 0
    lost_count = 0

    print(f"\n[Sender] Config: hand={hand_side}, sdk={sdk_path}")
    print(f"[Sender] Sending to {target_ip}:{port} at {hz} Hz")
    print("[Sender] Press Ctrl+C or Q to stop.\n")

    try:
        while True:
            t_start = time.perf_counter()

            buttons = kb.get_and_clear()

            if buttons["quit"]:
                print("\n[Sender] Quit requested")
                break

            # Read glove data
            if hand_side == "both":
                hands = reader.get_both_hands()
                for side, data in hands.items():
                    if data is not None:
                        pkt = _build_packet(data, buttons)
                        if lost_count > 0:
                            print(f"\n[Sender] {side} tracking recovered")
                    else:
                        pkt = _build_null_packet(side, buttons)

                    raw = json.dumps(pkt).encode()
                    sock.sendto(raw, target)
            else:
                data = reader.get_hand_data()
                if data is not None:
                    pkt = _build_packet(data, buttons)
                    if lost_count > 0:
                        print(f"\n[Sender] Tracking recovered (was lost for {lost_count} frames)")
                        lost_count = 0
                else:
                    pkt = _build_null_packet(hand_side, buttons)
                    lost_count += 1
                    if lost_count == 1 or lost_count % (hz * 2) == 0:
                        print(f"\r[Sender] Glove LOST ({lost_count} frames)", end="", flush=True)

                raw = json.dumps(pkt).encode()
                sock.sendto(raw, target)

            send_count += 1
            if send_count % (hz * 5) == 0:
                print(f"[Sender] Sent {send_count} packets")

            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\n[Sender] Stopped. Total packets sent: {send_count}")
    finally:
        kb.stop()
        sock.close()
        reader.disconnect()


if __name__ == "__main__":
    main()
