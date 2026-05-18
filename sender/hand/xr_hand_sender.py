#!/usr/bin/env python3
"""XR (Galaxy XR / Quest 3) hand sender — WebXR 25-joint → DG-5F 20-vec → UDP 9872.

조종 PC 에서 BridgePoseStore 의 right_hand_positions 를 읽어 XRDexRetargeter
로 retarget 한 후 UDP 9872 packet 송신. receiver.py 측은 `retargeted: True`
플래그 보고 passthrough + EMA filter (변경 없음).

Pipeline:
    BridgePoseStore.right_hand_positions (25, 3 WebXR world frame)
        → webxr_to_wrist_local_mano (wrist-local + palm-aligned MANO)
        → XRDexRetargeter.retarget → q20 (20-vec DG-5F)
        → UDP packet (HandData 호환)
        → robot/hand/receiver.py (passthrough + EMA + Modbus)

xr_teleop entry (`run_teleop_ur10e_ws.py`) 와의 차이:
    - DDS rt/dg5f/cmd 발행 대신 UDP 9872 송신
    - robot PC 측은 teleop_dev 의 기존 receiver.py + dg5f_ros2_client.py 그대로 사용
    - sim 모드 / DDS 의존성 없음

사용 예:
    # smoke (receiver 없이 monitor 로 확인)
    python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1

    # 실 환경
    python3 -m sender.hand.xr_hand_sender --target-ip 192.168.0.10 --hz 60
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

import numpy as np

from sender.hand.keyboard_state import KeyboardState
from sender.hand.xr_dex_retargeter import XRDexRetargeter, DG5F_NUM_MOTORS
from sender.hand.xr_remap import is_kp25_valid
from sender.xr_common import BridgePoseStore


HAND_UDP_PORT = 9872   # protocol/hand_protocol.py


def _build_packet(q20: np.ndarray, wrist_pose: np.ndarray, hand_side: str,
                  buttons: dict, tracking: bool) -> dict:
    """retargeted=True 패킷 — receiver.py 가 passthrough."""
    if wrist_pose is not None and wrist_pose.shape == (4, 4) and tracking:
        pos = wrist_pose[:3, 3].tolist()
        # rotation matrix → quaternion (wxyz)
        quat = _rotmat_to_quat_wxyz(wrist_pose[:3, :3])
    else:
        pos = [0.0, 0.0, 0.0]
        quat = [1.0, 0.0, 0.0, 0.0]

    return {
        "type": "xr",
        "hand": hand_side,
        "joint_angles": q20.tolist(),
        "finger_spread": [0.0] * 5,        # XR has no separate spread channel
        "wrist_pos": pos,
        "wrist_quat": quat,
        "tracking": tracking,
        "buttons": buttons,
        "timestamp": time.time(),
        "retargeted": True,
    }


def _build_null_packet(hand_side: str, buttons: dict) -> dict:
    """tracking lost frame — joint_angles 0 (receiver EMA 가 마지막 값으로 hold)."""
    return {
        "type": "xr",
        "hand": hand_side,
        "joint_angles": [0.0] * DG5F_NUM_MOTORS,
        "finger_spread": [0.0] * 5,
        "wrist_pos": [0.0, 0.0, 0.0],
        "wrist_quat": [1.0, 0.0, 0.0, 0.0],
        "tracking": False,
        "buttons": buttons,
        "timestamp": time.time(),
    }


def _rotmat_to_quat_wxyz(R: np.ndarray) -> list:
    """3×3 rotation matrix → wxyz quaternion (sender_base 패턴 호환).

    Branch-free Shepperd's method. numpy only.
    """
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [float(w), float(x), float(y), float(z)]


def main() -> int:
    parser = argparse.ArgumentParser(description="XR hand sender (Galaxy XR / Quest 3 → DG-5F)")
    parser.add_argument("--target-ip", required=True, help="Robot PC IP (UDP target)")
    parser.add_argument("--port", type=int, default=HAND_UDP_PORT,
                        help="UDP target port (default 9872)")
    parser.add_argument("--hz", type=int, default=60, help="Send rate Hz (default 60)")
    parser.add_argument("--hand", default="right", choices=["right", "left"],
                        help="Hand side (default: right; left 미검증)")
    parser.add_argument("--convention", default="mediapipe", choices=["mediapipe", "manus"],
                        help="WebXR → MANO chirality. visual 검증 후 fist↔spread inversion 시 manus 로 toggle")
    parser.add_argument("--bridge-port", type=int, default=None,
                        help="BridgePoseStore ws port (default 8013 or env XR_BRIDGE_PORT)")
    parser.add_argument("--no-keyboard", action="store_true",
                        help="pynput KeyboardState 비활성 (headless / sshd)")
    args = parser.parse_args()

    print(f"[xr_hand_sender] target={args.target_ip}:{args.port} hz={args.hz} side={args.hand}")
    print(f"[xr_hand_sender] convention={args.convention}")

    # 1) BridgePoseStore (Singleton)
    store = BridgePoseStore(use_hand_tracking=True, port=args.bridge_port)
    print(f"[xr_hand_sender] BridgePoseStore ready: http://localhost:{store.port}/")

    # 2) retargeter
    retargeter = XRDexRetargeter(convention=args.convention, hand_side=args.hand)

    # 3) keyboard (optional)
    kb = None
    if not args.no_keyboard:
        try:
            kb = KeyboardState()
            kb.start()
        except Exception as e:
            print(f"[xr_hand_sender] WARN: keyboard 비활성 ({e})")
            kb = None

    # 4) UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target_ip, args.port)
    dt = 1.0 / args.hz
    send_count = 0
    lost_count = 0
    last_log = time.time()

    print(f"\n[xr_hand_sender] Quest 3 / Galaxy XR Chrome 에서 "
          f"http://localhost:{store.port}/ → Enter VR/AR → 손 들이밀기")
    print(f"[xr_hand_sender] Press Ctrl+C or 'q' to stop.\n")

    try:
        while True:
            t_start = time.perf_counter()
            buttons = kb.get_and_clear() if kb else {
                "estop": False, "reset": False, "quit": False,
                "speed_up": False, "speed_down": False,
            }

            if buttons.get("quit"):
                print("\n[xr_hand_sender] Quit requested")
                break

            # 5) read hand keypoints + wrist pose
            if args.hand == "right":
                kp_25 = store.right_hand_positions
                wrist_pose = store.right_arm_pose
            else:
                kp_25 = store.left_hand_positions
                wrist_pose = store.left_arm_pose

            valid = is_kp25_valid(kp_25)
            if valid:
                q20 = retargeter.retarget(kp_25)
                pkt = _build_packet(q20, wrist_pose, args.hand, buttons, tracking=True)
                if lost_count > 0:
                    print(f"\n[xr_hand_sender] tracking recovered (was lost {lost_count} frames)")
                    lost_count = 0
            else:
                pkt = _build_null_packet(args.hand, buttons)
                lost_count += 1

            sock.sendto(json.dumps(pkt).encode("utf-8"), target)
            send_count += 1

            # 1Hz 상태 로그
            now = time.time()
            if now - last_log >= 1.0:
                stats = store.get_stats()
                q_preview = " ".join(f"{a:+5.2f}" for a in pkt["joint_angles"][:6])
                state = "TRACK" if pkt["tracking"] else "LOST "
                print(
                    f"[xr_hand_sender] #{send_count:>6d} {state} "
                    f"ws_msg={stats['hand_msg_count']} q[0:6]=[{q_preview}]",
                    flush=True,
                )
                last_log = now

            # rate control
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\n[xr_hand_sender] stopped. Total: {send_count}")
    finally:
        if kb:
            kb.stop()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
