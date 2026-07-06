#!/usr/bin/env python3
"""Unified XR (Galaxy XR / Quest 3) teleop launcher — arm + hand 동시 실행.

한 process 에서:
    - BridgePoseStore (Singleton, 자체 ws server)
    - XR arm sender thread  (UDP 9871)
    - XR hand sender thread (UDP 9872)
을 모두 띄움. 두 sender 가 BridgePoseStore 같은 인스턴스 공유 → port 8013 중복 부팅 없음.

사용 예:
    # 둘 다 (default)
    python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10

    # 팔만
    python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-hand

    # 손만
    python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-arm

    # 안전 기본값 (저속, 상대 motion scale 0.3)
    python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3

키 입력:
    arm 측 키 입력 (r/p/c/+/-/x) 은 메인 thread (foreground) 의 termios 가 받음.
    hand 측 key 는 pynput (background) 로 받음.
    → arm 의 sshkeyboard-like 단일 키 모드 사용. 'x' 또는 'Esc' 또는 'q' 로 quit.

종료:
    - 'x' 또는 'q' (메인 thread arm sender) → graceful shutdown
    - Ctrl+C → SIGINT propagate, 두 thread 모두 KeyboardInterrupt 처리
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Optional

from sender.xr_common import BridgePoseStore
from sender.xr_common.watchdog import StoreWatchdog, WorkspaceLimits


def _arm_thread_fn(args, store: BridgePoseStore, stop_event: threading.Event) -> None:
    """팔 sender thread. main thread 에서 실행 (termios 키 입력 받기 위해)."""
    try:
        # local import: BridgePoseStore singleton 은 이미 main 에서 init 완료
        from sender.arm.xr_sender import XRArmSender

        sender = XRArmSender(
            target_ip=args.target_ip,
            port=args.arm_port,
            hz=args.arm_hz,
            scale=args.scale,
            convention=args.convention,
            bridge_port=args.bridge_port,
            hand_side=args.hand,
            no_keyboard=args.no_keyboard,
            watchdog_timeout_s=args.watchdog_timeout_s,
            workspace=WorkspaceLimits(),
            enforce_workspace=not args.no_workspace_clamp,
        )
        sender.run()   # termios + 메인 루프
    except KeyboardInterrupt:
        print("\n[run_xr_teleop] arm thread interrupted")
    except Exception as e:
        import traceback
        print(f"[run_xr_teleop] arm thread error: {e}")
        traceback.print_exc()
    finally:
        stop_event.set()


def _hand_thread_fn(args, store: BridgePoseStore, stop_event: threading.Event) -> None:
    """손 sender thread. background thread (daemon)."""
    try:
        # 손 sender 의 main() 을 직접 호출하지 않고, 내부 loop 만 재현 — kb 비활성, target/store 공유
        import json
        import socket

        from sender.hand.xr_dex_retargeter import XRDexRetargeter, DG5F_NUM_MOTORS
        from sender.hand.xr_remap import is_kp25_valid
        from sender.hand.xr_hand_sender import _build_packet, _build_null_packet

        retargeter = XRDexRetargeter(convention=args.convention, hand_side=args.hand)
        watchdog = StoreWatchdog(store, timeout_s=args.watchdog_timeout_s)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = (args.target_ip, args.hand_port)
        dt = 1.0 / args.hand_hz
        send_count = 0
        skip_count = 0
        last_log = time.time()
        buttons_empty = {
            "estop": False, "reset": False, "quit": False,
            "speed_up": False, "speed_down": False,
        }

        print(f"[run_xr_teleop:hand] target={args.target_ip}:{args.hand_port} hz={args.hand_hz}")

        while not stop_event.is_set():
            t_start = time.perf_counter()

            # watchdog: BridgePoseStore stale 이면 송신 skip — receiver.py 의 EMA filter 가
            # 마지막 valid q20 hold (receiver._recv_loop 가 마지막 valid HandData 만 유지).
            if not watchdog.fresh():
                skip_count += 1
                elapsed = time.perf_counter() - t_start
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)
                continue

            if args.hand == "right":
                kp_25 = store.right_hand_positions
                wrist_pose = store.right_arm_pose
            else:
                kp_25 = store.left_hand_positions
                wrist_pose = store.left_arm_pose

            if is_kp25_valid(kp_25):
                q20 = retargeter.retarget(kp_25)
                pkt = _build_packet(q20, wrist_pose, args.hand, buttons_empty, tracking=True)
            else:
                pkt = _build_null_packet(args.hand, buttons_empty)

            sock.sendto(json.dumps(pkt).encode("utf-8"), target)
            send_count += 1

            now = time.time()
            if now - last_log >= 2.0:
                stats = store.get_stats()
                state = "TRACK" if pkt["tracking"] else "LOST "
                q = pkt["joint_angles"]
                print(
                    f"[run_xr_teleop:hand] #{send_count:>6d} {state} skip={skip_count} "
                    f"ws_msg={stats['hand_msg_count']} "
                    f"q[5/9/13/17]=[{q[5]:+.2f}/{q[9]:+.2f}/{q[13]:+.2f}/{q[17]:+.2f}]",
                    flush=True,
                )
                last_log = now

            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

        sock.close()
        print("[run_xr_teleop:hand] thread stopped")
    except KeyboardInterrupt:
        print("\n[run_xr_teleop] hand thread interrupted")
    except Exception as e:
        import traceback
        print(f"[run_xr_teleop] hand thread error: {e}")
        traceback.print_exc()
    finally:
        # hand thread 사망을 main loop (--no-arm 대기 루프 포함) 에 전파.
        # 없으면 --no-arm 실행 중 hand thread crash 시 main 이 영원히 대기.
        stop_event.set()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Unified XR teleop — arm + hand 동시 실행 (Galaxy XR / Quest 3 → UR10e + DG-5F)"
    )
    parser.add_argument("--target-ip", required=True, help="Robot PC IP")

    # arm
    parser.add_argument("--arm-port", type=int, default=9871, help="UDP port for arm (default 9871)")
    parser.add_argument("--arm-hz", type=int, default=50, help="Arm send Hz")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Position scale for relative motion (1.0 = 1:1)")
    parser.add_argument("--no-arm", action="store_true", help="팔 sender 비활성")

    # hand
    parser.add_argument("--hand-port", type=int, default=9872, help="UDP port for hand (default 9872)")
    parser.add_argument("--hand-hz", type=int, default=60, help="Hand send Hz")
    parser.add_argument("--convention", default="mediapipe", choices=["mediapipe", "manus"])
    parser.add_argument("--no-hand", action="store_true", help="손 sender 비활성")

    # shared
    parser.add_argument("--hand", default="right", choices=["right", "left"],
                        help="Hand side (default: right; left 미검증)")
    parser.add_argument("--bridge-port", type=int, default=None,
                        help="BridgePoseStore ws port (default 8013 or env XR_BRIDGE_PORT)")
    parser.add_argument("--no-keyboard", action="store_true",
                        help="termios 비활성 — sshd / headless. 자동 calibrate 시도")

    # safety
    parser.add_argument("--watchdog-timeout-s", type=float, default=0.2,
                        help="BridgePoseStore stale 판정 timeout (default 0.2s). "
                             "msg age 가 이 값 초과 시 sender 송신 skip")
    parser.add_argument("--no-workspace-clamp", action="store_true",
                        help="(arm) workspace envelope clamp 비활성. 위험 — 검증 후에만 사용")

    args = parser.parse_args()

    if args.no_arm and args.no_hand:
        print("[run_xr_teleop] both --no-arm and --no-hand specified; nothing to do")
        return 0

    # 1) BridgePoseStore (Singleton)
    store = BridgePoseStore(use_hand_tracking=True, port=args.bridge_port)
    print(f"[run_xr_teleop] BridgePoseStore: http://localhost:{store.port}/")
    print(f"[run_xr_teleop] adb reverse tcp:{store.port} tcp:{store.port} 필요")
    print(f"[run_xr_teleop] target_ip={args.target_ip}  scale={args.scale}  convention={args.convention}")
    print()

    stop_event = threading.Event()

    # 2) hand thread (background daemon)
    hand_thread: Optional[threading.Thread] = None
    if not args.no_hand:
        hand_thread = threading.Thread(
            target=_hand_thread_fn,
            args=(args, store, stop_event),
            name="xr-hand-sender",
            daemon=True,
        )
        hand_thread.start()

    # 3) arm thread — foreground (termios 키 입력 받기 위해)
    if not args.no_arm:
        try:
            _arm_thread_fn(args, store, stop_event)
        except KeyboardInterrupt:
            print("\n[run_xr_teleop] interrupted")
    else:
        # arm 없으면 메인 thread 가 그냥 대기
        print("[run_xr_teleop] arm 비활성 — Ctrl+C 로 종료")
        try:
            while not stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[run_xr_teleop] interrupted")

    stop_event.set()

    if hand_thread is not None:
        hand_thread.join(timeout=2.0)

    print("[run_xr_teleop] exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
