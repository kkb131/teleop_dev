#!/usr/bin/env python3
"""UDP Manus glove data sender — run on the operator PC.

Reads finger joint angles from Manus Quantum Metagloves via ROS2 topic
(manus_ros2) or SDK subprocess, applies retargeting, and sends DG5F
joint angles over UDP to the robot PC.

Requirements: numpy, pyyaml, pynput
    pip install numpy pyyaml pynput

Usage:
    # Raw 전송 (retarget 없이, receiver에서 처리 안 함)
    python3 -m sender.hand.manus_sender --target-ip <ROBOT_IP> --hand right

    # [1A] Ergonomics Direct Mapping (권장)
    python3 -m sender.hand.manus_sender --target-ip <ROBOT_IP> --hand right \
        --retarget ergo-direct --sdk-mode ros2

    # [1A] + 2포즈 캘리브레이션 (open hand → fist)
    python3 -m sender.hand.manus_sender --target-ip <ROBOT_IP> --hand right \
        --retarget ergo-direct --sdk-mode ros2 --calibrate
"""

import argparse
import json
import socket
import threading
import time

from sender.hand.manus_config import ManusConfig
from sender.hand.manus_reader import ManusReader

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


def _build_packet(data, buttons: dict, retargeted: bool = False) -> dict:
    """Build a JSON-serializable UDP packet from HandData."""
    pkt = {
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
    if retargeted:
        pkt["retargeted"] = True
    return pkt


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
                        help="YAML config file (default: sender/hand/config/default.yaml)")
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
    parser.add_argument("--sdk-mode", default=None,
                        choices=["subprocess", "ros2"],
                        help="SDK mode: subprocess=SDKClient binary, ros2=manus_ros2 topics (default: from config)")
    parser.add_argument("--retarget", default="none",
                        choices=["none", "ergo-direct"],
                        help="Retarget mode: none=raw ergonomics, ergo-direct=[1A] DG5F direct mapping")
    parser.add_argument("--calibrate", action="store_true",
                        help="2-pose calibration at startup (open hand + fist)")
    args = parser.parse_args()

    # Load config
    cfg = ManusConfig.load(args.config)

    # CLI overrides
    target_ip = args.target_ip or cfg.network.target_ip
    port = args.port or cfg.network.port
    hz = args.hz or cfg.network.hz
    hand_side = args.hand or cfg.hand.side
    sdk_path = args.sdk_path or cfg.sdk.bin_path
    sdk_mode = args.sdk_mode or cfg.sdk.mode

    if target_ip is None:
        print("[ERROR] --target-ip required (or set network.target_ip in config)")
        return

    # Connect to gloves
    if sdk_mode == "ros2":
        from sender.hand.manus_reader_ros2 import ManusReaderROS2
        reader = ManusReaderROS2(hand_side=hand_side)
    else:
        reader = ManusReader(sdk_bin_path=sdk_path, hand_side=hand_side)
    reader.connect()

    # Retarget setup
    retarget = None
    # retarget setup

    if args.retarget == "ergo-direct":
        import numpy as np
        from sender.hand.gen1a_ergo_direct import ErgoDirectRetarget
        retarget = ErgoDirectRetarget(
            hand_side=hand_side if hand_side != "both" else "right",
        )
        print(f"[Sender] Retarget: 1A-ergo-direct ({hand_side})")

        if args.calibrate:
            def _collect_ergo_samples(duration=3.0):
                """Collect ergonomics samples for calibration."""
                samples = []
                t0 = time.time()
                while time.time() - t0 < duration:
                    data = reader.get_hand_data()
                    if data is not None:
                        samples.append(data.joint_angles.copy())
                    time.sleep(0.016)
                return samples

            # Step 1: Rest (open hand)
            print("\n[1A-Cal] === Step 1/2: OPEN HAND ===")
            print("[1A-Cal] Hold hand FULLY OPEN (fingers spread) for 3 seconds...")
            time.sleep(2.0)
            print("[1A-Cal] Recording...")
            rest_samples = _collect_ergo_samples(3.0)
            if rest_samples:
                rest_avg = np.mean(rest_samples, axis=0)
                retarget.calibrate_rest(rest_avg)
                print(f"[1A-Cal] Rest recorded ({len(rest_samples)} samples)")
            else:
                print("[1A-Cal] WARNING: No data received!")

            # Step 2: Fist
            print("\n[1A-Cal] === Step 2/2: FIST ===")
            print("[1A-Cal] Make a TIGHT FIST for 3 seconds...")
            time.sleep(2.0)
            print("[1A-Cal] Recording...")
            fist_samples = _collect_ergo_samples(3.0)
            if fist_samples:
                fist_avg = np.mean(fist_samples, axis=0)
                retarget.calibrate_fist(fist_avg)
                print(f"[1A-Cal] Fist recorded ({len(fist_samples)} samples)")
            else:
                print("[1A-Cal] WARNING: No data received!")

            print("\n[1A-Cal] Calibration complete!\n")

    # Start keyboard listener
    kb = KeyboardState()
    kb.start()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_ip, port)
    dt = 1.0 / hz
    send_count = 0
    lost_count = 0

    print(f"\n[Sender] Config: hand={hand_side}, sdk_mode={sdk_mode}")
    print(f"[Sender] Retarget: {args.retarget}")
    print(f"[Sender] Sending to {target_ip}:{port} at {hz} Hz")
    print("[Sender] Press Ctrl+C or Q to stop.\n")

    try:
        while True:
            t_start = time.perf_counter()

            buttons = kb.get_and_clear()

            if buttons["quit"]:
                print("\n[Sender] Quit requested")
                break

            # Read glove data + apply retarget
            def _apply_retarget(data):
                """Apply retarget to HandData, return retargeted flag."""
                if retarget is None:
                    return False
                dg5f_q = retarget.retarget(ergonomics=data.joint_angles)
                data.joint_angles = dg5f_q.astype(np.float32)
                return True

            if hand_side == "both":
                hands = reader.get_both_hands()
                for side, data in hands.items():
                    if data is not None:
                        is_rt = _apply_retarget(data)
                        pkt = _build_packet(data, buttons, retargeted=is_rt)
                        if lost_count > 0:
                            print(f"\n[Sender] {side} tracking recovered")
                    else:
                        pkt = _build_null_packet(side, buttons)

                    raw = json.dumps(pkt).encode()
                    sock.sendto(raw, target)
            else:
                data = reader.get_hand_data()
                if data is not None:
                    is_rt = _apply_retarget(data)
                    pkt = _build_packet(data, buttons, retargeted=is_rt)
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
            # Status display every 0.5s
            if send_count % max(1, int(hz * 0.5)) == 0:
                mode_str = "VECTOR" if (retarget is not None) else "RAW"
                flag_str = "retargeted=True" if pkt.get("retargeted") else "retargeted=False"
                angles = pkt.get("joint_angles", [])
                preview = " ".join(f"{a:+6.2f}" for a in angles[:5])
                print(f"\r[SEND] #{send_count} mode={mode_str} {flag_str} angles[0:5]=[{preview}]", end="", flush=True)

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
