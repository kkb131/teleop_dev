#!/usr/bin/env python3
"""Dual-arm + dual-hand XR teleop — UR10e×2 + DG-5F×2 동시 조종.

한 process 에서:
    - BridgePoseStore (singleton ws server, port 8013)
    - right arm sender thread (UDP 9871) / left arm sender thread (UDP 9875)
    - right hand sender thread (UDP 9872) / left hand sender thread (UDP 9874)
    - main thread: termios 키 dispatcher — 키를 양팔 sender queue 로 broadcast

단일팔 기존 플로우 (scripts/run_xr_teleop.py) 는 그대로 유지 — 이 스크립트는
별도 진입점. 팔별 설정 (IP/포트/scale/remap/workspace) 은 yaml 로 관리.

사용 예:
    # 양팔 + 양손 (기본)
    python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml

    # robot PC IP override
    python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \\
        --target-ip 192.168.0.10

    # 왼팔만 (좌표계 튜닝 시 — docs/xr_dual_arm_left_tuning_ko.md)
    python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \\
        --only-arm left --no-hands

키 입력 (main thread 가 받아 양팔에 동시 broadcast):
    r — 양팔 동시 sync 시작/recalibrate (한 사용자 자세에 양팔 origin 쌍 캡처)
    p — pause/resume (resume 시 자동 recalibrate)
    c — immediate recalibrate
    Space — E-Stop (양팔)
    +/- — speed (robot 측)
    x / q / Esc — 전체 종료
"""

from __future__ import annotations

import argparse
import queue
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from sender.arm.xr_frame_align import remap_from_rpy_deg, validate_remap
from sender.xr_common import BridgePoseStore
from sender.xr_common.gesture_commands import GestureCommander, GestureConfig
from sender.xr_common.watchdog import StoreWatchdog, WorkspaceLimits


# ── config ──────────────────────────────────────────────────────────────

@dataclass
class ArmSideCfg:
    side: str
    enabled: bool = True
    port: int = 9871
    hz: int = 50
    scale: float = 0.5
    r_remap: Optional[np.ndarray] = None      # None = 기본 R_x(+90°)
    workspace: WorkspaceLimits = field(default_factory=WorkspaceLimits)


@dataclass
class HandSideCfg:
    side: str
    enabled: bool = True
    port: int = 9872
    hz: int = 60
    convention: str = "mediapipe"


@dataclass
class DualTeleopConfig:
    robot_pc_ip: str = "192.168.0.10"
    bridge_port: int = 8013
    watchdog_timeout_s: float = 0.2
    enforce_workspace: bool = True
    arms: List[ArmSideCfg] = field(default_factory=list)
    hands: List[HandSideCfg] = field(default_factory=list)
    gestures: GestureConfig = field(default_factory=GestureConfig)


def _resolve_remap(side: str, d: dict) -> Optional[np.ndarray]:
    """remap_matrix > remap_rpy_deg > None(기본값) 우선순위로 remap 해석."""
    if "remap_matrix" in d and d["remap_matrix"] is not None:
        R = validate_remap(np.array(d["remap_matrix"], dtype=np.float64))
        print(f"[xr_dual] {side} arm remap: matrix 지정\n{R}")
        return R
    if "remap_rpy_deg" in d and d["remap_rpy_deg"] is not None:
        r, p, y = d["remap_rpy_deg"]
        R = remap_from_rpy_deg(r, p, y)
        print(f"[xr_dual] {side} arm remap: rpy_deg [{r}, {p}, {y}]")
        return R
    print(f"[xr_dual] {side} arm remap: 기본값 R_x(+90°)")
    return None


def _resolve_workspace(d: dict) -> WorkspaceLimits:
    ws = d.get("workspace")
    if not ws:
        return WorkspaceLimits()
    x, y, z = ws.get("x"), ws.get("y"), ws.get("z")
    kw = {}
    if x: kw.update(x_min=float(x[0]), x_max=float(x[1]))
    if y: kw.update(y_min=float(y[0]), y_max=float(y[1]))
    if z: kw.update(z_min=float(z[0]), z_max=float(z[1]))
    return WorkspaceLimits(**kw)


def load_dual_config(path: str) -> DualTeleopConfig:
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    cfg = DualTeleopConfig()
    net = data.get("network", {})
    cfg.robot_pc_ip = net.get("robot_pc_ip", cfg.robot_pc_ip)
    cfg.bridge_port = int(net.get("bridge_port", cfg.bridge_port))
    saf = data.get("safety", {})
    cfg.watchdog_timeout_s = float(saf.get("watchdog_timeout_s", cfg.watchdog_timeout_s))
    cfg.enforce_workspace = bool(saf.get("enforce_workspace", cfg.enforce_workspace))

    ges = data.get("gestures", {}) or {}
    cfg.gestures = GestureConfig(
        enabled=bool(ges.get("enabled", False)),
        hold_s=float(ges.get("hold_s", 1.5)),
        refractory_s=float(ges.get("refractory_s", 3.0)),
    )

    for side in ("right", "left"):
        a = data.get("arms", {}).get(side)
        if a:
            cfg.arms.append(ArmSideCfg(
                side=side,
                enabled=bool(a.get("enabled", True)),
                port=int(a.get("port", 9871 if side == "right" else 9875)),
                hz=int(a.get("hz", 50)),
                scale=float(a.get("scale", 0.5)),
                r_remap=_resolve_remap(side, a),
                workspace=_resolve_workspace(a),
            ))
        h = data.get("hands", {}).get(side)
        if h:
            cfg.hands.append(HandSideCfg(
                side=side,
                enabled=bool(h.get("enabled", True)),
                port=int(h.get("port", 9872 if side == "right" else 9874)),
                hz=int(h.get("hz", 60)),
                convention=h.get("convention", "mediapipe"),
            ))
    return cfg


# ── threads ─────────────────────────────────────────────────────────────

def _arm_thread(sender, stop_event: threading.Event) -> None:
    try:
        sender.run()
    except Exception as e:
        import traceback
        print(f"\n[xr_dual] arm({sender.hand_side}) thread error: {e}")
        traceback.print_exc()
    finally:
        # 한 팔이라도 죽으면 전체 종료 신호 — 반쪽 운전 방지
        stop_event.set()


def _hand_thread(store: BridgePoseStore, cfg: HandSideCfg, target_ip: str,
                 watchdog_timeout_s: float, stop_event: threading.Event) -> None:
    """run_xr_teleop._hand_thread_fn 의 side/port 파라미터화 버전."""
    try:
        import json
        import socket

        from sender.hand.xr_dex_retargeter import XRDexRetargeter
        from sender.hand.xr_remap import is_kp25_valid
        from sender.hand.xr_hand_sender import _build_packet, _build_null_packet

        retargeter = XRDexRetargeter(convention=cfg.convention, hand_side=cfg.side)
        watchdog = StoreWatchdog(store, timeout_s=watchdog_timeout_s)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = (target_ip, cfg.port)
        dt = 1.0 / cfg.hz
        send_count = 0
        skip_count = 0
        last_log = time.time()
        buttons_empty = {
            "estop": False, "reset": False, "quit": False,
            "speed_up": False, "speed_down": False,
        }

        print(f"[xr_dual:hand:{cfg.side}] target={target_ip}:{cfg.port} hz={cfg.hz}")

        while not stop_event.is_set():
            t_start = time.perf_counter()

            if not watchdog.fresh():
                skip_count += 1
                elapsed = time.perf_counter() - t_start
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)
                continue

            if cfg.side == "right":
                kp_25 = store.right_hand_positions
                wrist_pose = store.right_arm_pose
            else:
                kp_25 = store.left_hand_positions
                wrist_pose = store.left_arm_pose

            if is_kp25_valid(kp_25):
                q20 = retargeter.retarget(kp_25)
                pkt = _build_packet(q20, wrist_pose, cfg.side, buttons_empty, tracking=True)
            else:
                pkt = _build_null_packet(cfg.side, buttons_empty)

            sock.sendto(json.dumps(pkt).encode("utf-8"), target)
            send_count += 1

            now = time.time()
            if now - last_log >= 2.0:
                state = "TRACK" if pkt["tracking"] else "LOST "
                q = pkt["joint_angles"]
                print(
                    f"[xr_dual:hand:{cfg.side}] #{send_count:>6d} {state} "
                    f"skip={skip_count} "
                    f"q[5/9/13/17]=[{q[5]:+.2f}/{q[9]:+.2f}/{q[13]:+.2f}/{q[17]:+.2f}]",
                    flush=True,
                )
                last_log = now

            elapsed = time.perf_counter() - t_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        sock.close()
        print(f"[xr_dual:hand:{cfg.side}] thread stopped")
    except Exception as e:
        import traceback
        print(f"[xr_dual] hand({cfg.side}) thread error: {e}")
        traceback.print_exc()


# ── key dispatch (main thread — 유일한 termios 소유자) ──────────────────

def _key_dispatch_loop(key_queues: List[queue.Queue],
                       arm_threads: List[threading.Thread],
                       stop_event: threading.Event) -> None:
    """stdin 키를 모든 arm sender queue 로 broadcast.

    'r' 이 양팔에 동시에 전달되므로 한 사용자 자세에서 양팔 origin 이 같은
    순간에 캡처됨 (좌/우 손목-로봇 origin 쌍의 시간 정합).
    """
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while not stop_event.is_set():
            # 모든 arm thread 가 죽으면 종료
            if arm_threads and not any(t.is_alive() for t in arm_threads):
                print("\n[xr_dual] 모든 arm thread 종료됨")
                break

            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[" and select.select([sys.stdin], [], [], 0.01)[0]:
                        sys.stdin.read(1)
                ch = "ESC"

            for q_ in key_queues:
                q_.put(ch)

            if ch in ("x", "q", "ESC"):
                print("\n[xr_dual] quit — 전체 종료")
                stop_event.set()
                break
    except KeyboardInterrupt:
        print("\n[xr_dual] Ctrl+C — 전체 종료")
        for q_ in key_queues:
            q_.put("x")
        stop_event.set()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# ── main ────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Dual-arm + dual-hand XR teleop (Galaxy XR → UR10e×2 + DG-5F×2)"
    )
    default_cfg = Path(__file__).parent / "config" / "xr_dual.yaml"
    parser.add_argument("--config", default=str(default_cfg),
                        help=f"dual teleop yaml (default {default_cfg})")
    parser.add_argument("--target-ip", default=None,
                        help="robot PC IP (yaml network.robot_pc_ip override)")
    parser.add_argument("--only-arm", choices=["right", "left"], default=None,
                        help="한쪽 팔만 활성 (좌표계 튜닝/디버그용)")
    parser.add_argument("--no-hands", action="store_true", help="손 sender 전체 비활성")
    parser.add_argument("--no-arms", action="store_true", help="팔 sender 전체 비활성")
    parser.add_argument("--scale", type=float, default=None,
                        help="모든 팔 scale override (yaml 값 대신)")
    parser.add_argument("--no-keyboard", action="store_true",
                        help="termios 비활성 (headless). 각 팔이 자동 calibrate 재시도")
    ges_group = parser.add_mutually_exclusive_group()
    ges_group.add_argument("--gestures", dest="gestures", action="store_true",
                           default=None,
                           help="양손 제스처 명령 활성 (yaml gestures.enabled override). "
                                "양손 pinch hold='r', 양손 squeeze hold='p'")
    ges_group.add_argument("--no-gestures", dest="gestures", action="store_false",
                           default=None,
                           help="양손 제스처 명령 비활성 (yaml override)")
    args = parser.parse_args()

    cfg = load_dual_config(args.config)
    target_ip = args.target_ip or cfg.robot_pc_ip

    arms = [a for a in cfg.arms if a.enabled]
    hands = [h for h in cfg.hands if h.enabled]
    if args.only_arm:
        arms = [a for a in arms if a.side == args.only_arm]
    if args.no_arms:
        arms = []
    if args.no_hands:
        hands = []
    if args.scale is not None:
        for a in arms:
            a.scale = args.scale

    if not arms and not hands:
        print("[xr_dual] 활성화된 arm/hand 없음 — yaml enabled 또는 CLI 플래그 확인")
        return 1

    # 포트 중복 검사 (좌우 sender 가 같은 포트로 쏘면 robot 측에서 분리 불가)
    ports = [a.port for a in arms] + [h.port for h in hands]
    if len(ports) != len(set(ports)):
        print(f"[xr_dual] ERROR: UDP 포트 중복 — {ports}")
        return 1

    print("=" * 60)
    print("  Dual XR Teleop")
    print(f"  target: {target_ip}")
    for a in arms:
        print(f"  arm  {a.side:<5s} port={a.port} hz={a.hz} scale={a.scale}")
    for h in hands:
        print(f"  hand {h.side:<5s} port={h.port} hz={h.hz} conv={h.convention}")
    print("=" * 60)

    # 1) BridgePoseStore singleton
    store = BridgePoseStore(use_hand_tracking=True, port=cfg.bridge_port)
    print(f"[xr_dual] BridgePoseStore: http://localhost:{store.port}/")
    print(f"[xr_dual] adb reverse tcp:{store.port} tcp:{store.port} 필요")

    stop_event = threading.Event()

    # 2) arm senders (background threads, key_queue 로 키 수신)
    from sender.arm.xr_sender import XRArmSender

    key_queues: List[queue.Queue] = []
    arm_threads: List[threading.Thread] = []
    for a in arms:
        kq: Optional[queue.Queue] = None if args.no_keyboard else queue.Queue()
        sender = XRArmSender(
            target_ip=target_ip,
            port=a.port,
            hz=a.hz,
            scale=a.scale,
            bridge_port=cfg.bridge_port,
            hand_side=a.side,
            no_keyboard=args.no_keyboard,
            watchdog_timeout_s=cfg.watchdog_timeout_s,
            workspace=a.workspace,
            enforce_workspace=cfg.enforce_workspace,
            r_remap=a.r_remap,
            key_queue=kq,
            label=a.side,
        )
        if kq is not None:
            key_queues.append(kq)
        t = threading.Thread(target=_arm_thread, args=(sender, stop_event),
                             name=f"xr-arm-{a.side}", daemon=True)
        arm_threads.append(t)

    # 3) hand threads
    hand_threads: List[threading.Thread] = []
    for h in hands:
        t = threading.Thread(
            target=_hand_thread,
            args=(store, h, target_ip, cfg.watchdog_timeout_s, stop_event),
            name=f"xr-hand-{h.side}", daemon=True,
        )
        hand_threads.append(t)

    for t in arm_threads + hand_threads:
        t.start()

    # 3.5) 양손 제스처 명령 (opt-in) — 키를 양팔 queue 에 broadcast
    if args.gestures is not None:
        cfg.gestures.enabled = args.gestures
    if cfg.gestures.enabled:
        if key_queues:
            gesture = GestureCommander(
                store,
                emit=lambda ch: [q_.put(ch) for q_ in key_queues],
                cfg=cfg.gestures,
                stop_event=stop_event,
            )
            gesture.start()
        else:
            print("[gesture] key_queue 없음 (--no-keyboard 또는 팔 비활성) — "
                  "제스처 비활성", flush=True)

    # 4) main thread — 키 dispatcher (또는 headless 대기)
    try:
        if args.no_keyboard or not key_queues:
            print("[xr_dual] keyboard 비활성 — Ctrl+C 로 종료")
            while not stop_event.is_set():
                if arm_threads and not any(t.is_alive() for t in arm_threads):
                    break
                time.sleep(0.5)
        else:
            print("[xr_dual] keys: r=양팔 sync  p=pause  c=recal  Space=E-Stop"
                  "  +/-=speed  x/q/Esc=quit")
            _key_dispatch_loop(key_queues, arm_threads, stop_event)
    except KeyboardInterrupt:
        print("\n[xr_dual] interrupted")

    # 5) shutdown
    stop_event.set()
    for q_ in key_queues:
        q_.put("x")
    for t in arm_threads + hand_threads:
        t.join(timeout=3.0)

    print("[xr_dual] exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
