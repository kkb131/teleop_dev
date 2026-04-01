#!/usr/bin/env python3
"""Manus UDP receiver → DG5F PID controller (MultiDOFCommand).

Receives Manus glove data via UDP (from manus_sender on operator PC),
retargets joint angles to DG5F motor commands, and publishes via
MultiDOFCommand to pid_controller/PidController.

Two retarget modes (auto-detected via packet "retargeted" flag):
  - sender raw mode: receiver applies ManusToD5FRetarget + EMA filter
  - sender vector mode: sender already computed DG5F angles, receiver passes through

Supports hot-reload of calibration via {"type":"reload_config"} UDP trigger.

Requires dg5f_driver (PID mode) running:
    ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

Usage:
    # Dry-run (no hardware — just receive + retarget + print):
    python3 -m robot.hand.receiver --dry-run

    # ROS2 mode (dg5f_driver pid_all must be running):
    python3 -m robot.hand.receiver --hand right

    # With config file:
    python3 -m robot.hand.receiver --config config/default.yaml
"""

import argparse
import json
import signal
import socket
import sys
import threading
import time
from typing import Optional

import numpy as np

from protocol.hand_protocol import HandData, NUM_JOINTS, NUM_FINGERS
from robot.hand.retarget import ManusToD5FRetarget
from robot.hand.tesollo_config import TesolloConfig


# ─────────────────────────────────────────────────────────
# UDP Receiver (threaded)
# ─────────────────────────────────────────────────────────

class ManusReceiver:
    """Receives Manus glove UDP packets in a background thread.

    Thread-safe: get_latest() returns the most recent HandData.
    """

    def __init__(self, port: int = 9872, bind_ip: str = "0.0.0.0"):
        self._port = port
        self._bind_ip = bind_ip
        self._latest: Optional[HandData] = None
        self._lock = threading.Lock()
        self._running = False
        self._pkt_count = 0
        self._last_recv_time = 0.0
        self._reload_requested = False
        self._is_retargeted = False  # True if sender already applied retarget

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_ip, self._port))
        self._sock.settimeout(0.1)
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print(f"[Receiver] Listening on {self._bind_ip}:{self._port}")

    def stop(self):
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        if hasattr(self, "_sock"):
            self._sock.close()

    def get_latest(self) -> Optional[HandData]:
        with self._lock:
            return self._latest

    @property
    def packet_count(self) -> int:
        return self._pkt_count

    @property
    def last_recv_time(self) -> float:
        return self._last_recv_time

    @property
    def is_retargeted(self) -> bool:
        """True if the sender already applied retarget (vector mode)."""
        return self._is_retargeted

    @property
    def reload_requested(self) -> bool:
        if self._reload_requested:
            self._reload_requested = False
            return True
        return False

    def _recv_loop(self):
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(4096)
                pkt = json.loads(raw.decode())

                # Handle reload trigger from calibrate_retarget
                if pkt.get("type") == "reload_config":
                    self._reload_requested = True
                    continue

                data = HandData(
                    joint_angles=np.array(pkt["joint_angles"], dtype=np.float32),
                    finger_spread=np.array(pkt.get("finger_spread", [0] * NUM_FINGERS), dtype=np.float32),
                    wrist_pos=np.array(pkt.get("wrist_pos", [0, 0, 0])),
                    wrist_quat=np.array(pkt.get("wrist_quat", [1, 0, 0, 0])),
                    hand_side=pkt.get("hand", "right"),
                    timestamp=pkt.get("timestamp", time.time()),
                )

                # Track if sender already retargeted
                self._is_retargeted = pkt.get("retargeted", False)

                # Check e-stop
                buttons = pkt.get("buttons", {})
                if buttons.get("estop", False):
                    data = None  # Suppress data on e-stop

                with self._lock:
                    self._latest = data
                self._pkt_count += 1
                self._last_recv_time = time.time()

            except socket.timeout:
                continue
            except json.JSONDecodeError:
                continue
            except Exception:
                continue


# ─────────────────────────────────────────────────────────
# Control loop
# ─────────────────────────────────────────────────────────

def _print_status(data: HandData, dg5f_angles: np.ndarray, hz: float,
                  pkt_count: int, frame: int):
    """Print compact status to terminal."""
    if frame > 0:
        # Move cursor up to overwrite
        print(f"\033[8A", end="")

    print(f"  Frame: {frame:6d} | Pkts: {pkt_count:6d} | Rate: {hz:.1f} Hz")
    print(f"  Hand: {data.hand_side.upper()} | Tracking: True")
    print(f"  {'Finger':8s} {'Manus':>8s} {'DG5F':>8s} {'Manus':>8s} {'DG5F':>8s} {'Manus':>8s} {'DG5F':>8s} {'Manus':>8s} {'DG5F':>8s}")

    finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    for f in range(5):
        b = f * 4
        parts = []
        for j in range(4):
            m_val = data.joint_angles[b + j]
            d_val = dg5f_angles[b + j]
            parts.append(f"{m_val:+7.3f} {d_val:+7.3f}")
        print(f"  {finger_names[f]:8s} {' '.join(parts)}")


def main():
    parser = argparse.ArgumentParser(
        description="Manus → DG5F retarget receiver (ROS2)"
    )
    parser.add_argument("--config", default=None,
                        help="YAML config file path")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP listen port (overrides config)")
    parser.add_argument("--hand", default=None,
                        choices=["left", "right"],
                        help="Hand side (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Control loop Hz (overrides config)")
    parser.add_argument("--dry-run", action="store_true",
                        help="No ROS2 publishing — just receive + retarget + print")
    parser.add_argument("--motion-time", type=int, default=None,
                        help="Per-joint motion time in ms (overrides config)")
    args = parser.parse_args()

    # Load config
    cfg = TesolloConfig.load(args.config)

    # CLI overrides
    if args.port is not None:
        cfg.network.listen_port = args.port
    if args.hand is not None:
        cfg.hand.side = args.hand
    if args.hz is not None:
        cfg.control.hz = args.hz
    if args.motion_time is not None:
        cfg.control.motion_time_ms = args.motion_time

    print("=" * 70)
    print("  Manus → DG5F Retarget Receiver (ROS2)")
    print(f"  Mode: {'DRY RUN' if args.dry_run else 'LIVE (ROS2)'}")
    print(f"  UDP: {cfg.network.listen_ip}:{cfg.network.listen_port}")
    if not args.dry_run:
        print(f"  Hand: {cfg.hand.side}")
    print(f"  Rate: {cfg.control.hz} Hz | Motion time: {cfg.control.motion_time_ms} ms")
    print("  Controls: Ctrl+C = Quit")
    print("=" * 70)

    # Setup retarget
    retarget = ManusToD5FRetarget(
        hand_side=cfg.hand.side,
        calibration_factors=cfg.retarget.calibration_factors,
    )

    # Setup ROS2 client (unless dry-run)
    client = None
    if not args.dry_run:
        import rclpy
        rclpy.init()
        from robot.hand.dg5f_ros2_client import DG5FROS2Client
        client = DG5FROS2Client(
            hand_side=cfg.hand.side,
            motion_time_ms=cfg.control.motion_time_ms,
        )
        print(f"  [ROS2] DG5FROS2Client initialized ({cfg.hand.side} hand)")

    # Setup UDP receiver
    receiver = ManusReceiver(
        port=cfg.network.listen_port,
        bind_ip=cfg.network.listen_ip,
    )
    receiver.start()

    # Graceful shutdown
    shutdown = threading.Event()

    def _signal_handler(sig, frame):
        print("\n\n  Shutting down...")
        shutdown.set()

    signal.signal(signal.SIGINT, _signal_handler)

    # Control loop
    dt = 1.0 / cfg.control.hz
    frame = 0
    loop_hz = 0.0
    hz_timer = time.time()
    ema_alpha = 0.2  # EMA smoothing on retarget OUTPUT (0=frozen, 1=no smoothing)
    filtered_dg5f = None
    hz_count = 0
    last_data = None

    print("\n  Waiting for Manus data...\n")

    try:
        while not shutdown.is_set():
            t0 = time.perf_counter()

            # Check for config reload trigger
            if receiver.reload_requested:
                print("\n\n  [RELOAD] Reloading config...")
                cfg = TesolloConfig.load(args.config)
                retarget = ManusToD5FRetarget(
                    hand_side=cfg.hand.side,
                    calibration_factors=cfg.retarget.calibration_factors,
                )
                filtered_dg5f = None  # reset EMA
                print("  [RELOAD] Calibration updated!\n")

            # Process ROS2 callbacks (joint_states feedback)
            if client is not None:
                rclpy.spin_once(client, timeout_sec=0)

            data = receiver.get_latest()
            if data is not None and data is not last_data:
                last_data = data

                # Retarget (skip if sender already applied vector retarget)
                if receiver.is_retargeted:
                    dg5f_raw = data.joint_angles
                else:
                    dg5f_raw = retarget.retarget(data.joint_angles)

                # EMA filter on retarget OUTPUT to reduce jitter
                if filtered_dg5f is None:
                    filtered_dg5f = dg5f_raw.copy()
                else:
                    filtered_dg5f = ema_alpha * dg5f_raw + (1 - ema_alpha) * filtered_dg5f
                dg5f_angles = filtered_dg5f

                # Publish to ROS2
                if client is not None:
                    try:
                        client.set_positions(dg5f_angles)
                    except Exception as e:
                        print(f"\r  [WARN] ROS2 publish error: {e}", end="", flush=True)

                # Display
                hz_count += 1
                now = time.time()
                if now - hz_timer >= 1.0:
                    loop_hz = hz_count / (now - hz_timer)
                    hz_count = 0
                    hz_timer = now

                _print_status(data, dg5f_angles, loop_hz, receiver.packet_count, frame)
                frame += 1
            else:
                # No new data — check for timeout
                if receiver.last_recv_time > 0:
                    age = time.time() - receiver.last_recv_time
                    if age > 2.0 and frame > 0:
                        print(f"\r  [WARN] No data for {age:.1f}s", end="", flush=True)

            elapsed = time.perf_counter() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    finally:
        receiver.stop()
        if client is not None:
            client.destroy_node()
            rclpy.shutdown()

    print(f"\n  Total frames: {frame}")
    print(f"  Total packets: {receiver.packet_count}")
    print("  Done.")


if __name__ == "__main__":
    main()
