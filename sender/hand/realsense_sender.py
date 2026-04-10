#!/usr/bin/env python3
"""UDP RealSense D405 hand sender — run on the operator PC with a D405 camera.

Captures color + depth frames from a RealSense D405, runs MediaPipe
HandLandmarker, deprojects 2D landmarks via real depth, transforms into
MANO 21-keypoint frame, retargets to DG5F joint angles via DexPilot/Vector
optimizer, and streams the resulting (20,) joint angles over UDP to the
robot PC. The robot-side receiver treats packets with ``"retargeted":True``
as final commands.

Pipeline:
    D405 → RealSenseReader → HandKeypoints (21,3 MANO)
        → DexRetargetWrapper(keypoints=...) → DG5F angles (20,)
        → UDP packet → robot/hand/receiver

Requirements: pyrealsense2, mediapipe==0.10.21, opencv-python, numpy<2,
              dex_retargeting, pynput, pyyaml

Usage:
    # Default (DexPilot optimizer, 30Hz, right hand)
    python3 -m sender.hand.realsense_sender --target-ip <ROBOT_IP>

    # Vector optimizer + OpenCV preview window
    python3 -m sender.hand.realsense_sender --target-ip <ROBOT_IP> \
        --dex-optimizer vector --viz

    # Specific D405 device + custom resolution
    python3 -m sender.hand.realsense_sender --target-ip <ROBOT_IP> \
        --rs-serial 1234ABCD --resolution 640x480
"""

import argparse
import json
import socket
import time

import numpy as np

from sender.hand.gen3a_dex_retarget import DexRetargetWrapper
from sender.hand.keyboard_state import KeyboardState
from sender.hand.realsense import RealsenseConfig, RealSenseReader
from sender.hand.realsense.visualizer import HandVisualizer


def _build_packet(dg5f_q, hand_side: str, buttons: dict) -> dict:
    """Build a JSON-serializable UDP packet with retargeted DG5F angles."""
    return {
        "type": "realsense",
        "hand": hand_side,
        "joint_angles": dg5f_q.tolist(),
        "finger_spread": [0.0] * 5,          # MediaPipe gives no ergonomics
        "wrist_pos": [0.0, 0.0, 0.0],
        "wrist_quat": [1.0, 0.0, 0.0, 0.0],
        "tracking": True,
        "buttons": buttons,
        "timestamp": time.time(),
        "retargeted": True,                   # always — sender retargets every frame
    }


def _build_null_packet(hand_side: str, buttons: dict) -> dict:
    """Build a packet for frames where no hand was detected."""
    return {
        "type": "realsense",
        "hand": hand_side,
        "joint_angles": [0.0] * 20,
        "finger_spread": [0.0] * 5,
        "wrist_pos": [0.0, 0.0, 0.0],
        "wrist_quat": [1.0, 0.0, 0.0, 0.0],
        "tracking": False,
        "buttons": buttons,
        "timestamp": time.time(),
    }


def _parse_resolution(s: str) -> tuple[int, int]:
    try:
        w_str, h_str = s.lower().split("x")
        return int(w_str), int(h_str)
    except (ValueError, AttributeError):
        raise argparse.ArgumentTypeError(f"--resolution must be WxH, got {s!r}")


def main():
    parser = argparse.ArgumentParser(
        description="UDP RealSense D405 hand sender ([3A] dex_retarget)",
    )
    parser.add_argument("--config", default=None,
                        help="YAML config file (default: realsense/default_config.yaml)")
    parser.add_argument("--target-ip", default=None,
                        help="Robot PC IP (overrides config)")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP port (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Send rate in Hz (overrides config; D405 is 30Hz native)")
    parser.add_argument("--hand", default=None,
                        choices=["left", "right"],
                        help="Which hand to track (overrides config)")
    parser.add_argument("--rs-serial", default=None,
                        help="D405 device serial (auto-detect if omitted)")
    parser.add_argument("--resolution", type=_parse_resolution, default=None,
                        help="Camera resolution as WxH (default 640x480)")
    parser.add_argument("--dex-optimizer", default=None,
                        choices=["dexpilot", "vector"],
                        help="dex_retargeting optimizer (default: dexpilot)")
    parser.add_argument("--viz", action="store_true",
                        help="Show OpenCV preview window with detected hand")
    args = parser.parse_args()

    # Load config + apply CLI overrides
    cfg = RealsenseConfig.load(args.config)
    target_ip = args.target_ip or cfg.network.target_ip
    port = args.port or cfg.network.port
    hz = args.hz or cfg.network.hz
    hand_side = args.hand or cfg.hand.side
    rs_serial = args.rs_serial or cfg.camera.serial
    if args.resolution is not None:
        cam_w, cam_h = args.resolution
    else:
        cam_w, cam_h = cfg.camera.width, cfg.camera.height
    optimizer = args.dex_optimizer or cfg.retarget.optimizer
    viz_enabled = args.viz or cfg.display.viz

    if target_ip is None:
        print("[ERROR] --target-ip required (or set network.target_ip in config)")
        return

    # 1. RealSense reader
    reader = RealSenseReader(
        hand_side=hand_side,
        serial=rs_serial,
        width=cam_w,
        height=cam_h,
        fps=cfg.camera.fps,
    )
    reader.connect()

    # 2. dex_retargeting wrapper (gen3a)
    retarget = DexRetargetWrapper(hand_side=hand_side, optimizer=optimizer)
    print(f"[Sender] Retarget: 3A-dex-retarget ({hand_side}, optimizer={optimizer})")

    # 3. Keyboard listener
    kb = KeyboardState()
    kb.start()

    # 4. UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_ip, port)
    dt = 1.0 / hz
    send_count = 0
    lost_count = 0

    # 5. Optional visualizer
    visualizer = HandVisualizer() if viz_enabled else None

    print(f"\n[Sender] Config: hand={hand_side}, source=realsense, "
          f"camera={cam_w}x{cam_h}@{cfg.camera.fps}fps")
    print(f"[Sender] Sending to {target_ip}:{port} at {hz} Hz")
    print("[Sender] Press Ctrl+C or Q to stop.\n")

    try:
        while True:
            t_start = time.perf_counter()

            buttons = kb.get_and_clear()
            if buttons["quit"]:
                print("\n[Sender] Quit requested")
                break

            data = reader.get_data()  # HandKeypoints or None
            if data is not None:
                dg5f_q = retarget.retarget(keypoints=data.keypoints_3d)
                pkt = _build_packet(dg5f_q, hand_side, buttons)
                if lost_count > 0:
                    print(f"\n[Sender] Tracking recovered (was lost for {lost_count} frames)")
                    lost_count = 0
            else:
                pkt = _build_null_packet(hand_side, buttons)
                lost_count += 1
                if lost_count == 1 or lost_count % (hz * 2) == 0:
                    print(f"\r[Sender] Hand LOST ({lost_count} frames)",
                          end="", flush=True)

            sock.sendto(json.dumps(pkt).encode(), target)
            send_count += 1

            # Optional viz
            if visualizer is not None:
                frame = reader.get_frame()
                if frame is not None:
                    annotated = visualizer.draw(
                        frame, data, fps=0.0,
                        detection_ms=reader.detection_time_ms,
                    )
                    if not visualizer.show(annotated):
                        print("\n[Sender] Visualizer window closed")
                        break

            # Status display every 0.5s
            if send_count % max(1, int(hz * 0.5)) == 0:
                flag_str = "retargeted=True" if pkt.get("retargeted") else "retargeted=False"
                angles = pkt.get("joint_angles", [])
                preview = " ".join(f"{a:+6.2f}" for a in angles[:5])
                print(f"\r[SEND] #{send_count} mode=DEX {flag_str} "
                      f"angles[0:5]=[{preview}]", end="", flush=True)

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
        if visualizer is not None:
            visualizer.close()


if __name__ == "__main__":
    main()
