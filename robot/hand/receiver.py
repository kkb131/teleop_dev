#!/usr/bin/env python3
"""Manus UDP receiver → DG5F retarget → Modbus TCP control loop.

Receives Manus glove data via UDP (from manus_sender on operator PC),
retargets joint angles to DG5F motor commands, and sends via Modbus TCP.

Usage:
    # Dry-run (no DG5F hardware — just receive + retarget + print):
    python3 -m tesollo.receiver --dry-run

    # Real DG5F hand:
    python3 -m tesollo.receiver --hand-ip 169.254.186.72 --port 9872

    # With config file:
    python3 -m tesollo.receiver --config tesollo/config/default.yaml
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

from teleop_dev.protocol.hand_protocol import HandData, NUM_JOINTS, NUM_FINGERS
from teleop_dev.robot.hand.retarget import ManusToD5FRetarget
from teleop_dev.robot.hand.tesollo_config import TesolloConfig


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

    def _recv_loop(self):
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(4096)
                pkt = json.loads(raw.decode())

                data = HandData(
                    joint_angles=np.array(pkt["joint_angles"], dtype=np.float32),
                    finger_spread=np.array(pkt.get("finger_spread", [0] * NUM_FINGERS), dtype=np.float32),
                    wrist_pos=np.array(pkt.get("wrist_pos", [0, 0, 0])),
                    wrist_quat=np.array(pkt.get("wrist_quat", [1, 0, 0, 0])),
                    hand_side=pkt.get("hand", "right"),
                    timestamp=pkt.get("timestamp", time.time()),
                )

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
        description="Manus → DG5F retarget receiver"
    )
    parser.add_argument("--config", default=None,
                        help="YAML config file path")
    parser.add_argument("--hand-ip", default=None,
                        help="DG5F hand IP (overrides config)")
    parser.add_argument("--hand-port", type=int, default=None,
                        help="DG5F Modbus port (overrides config)")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP listen port (overrides config)")
    parser.add_argument("--hand", default=None,
                        choices=["left", "right"],
                        help="Hand side (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Control loop Hz (overrides config)")
    parser.add_argument("--dry-run", action="store_true",
                        help="No DG5F connection — just receive + retarget + print")
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
    if args.hand_ip is not None:
        cfg.hand.ip = args.hand_ip
    if args.hand_port is not None:
        cfg.hand.port = args.hand_port
    if args.hz is not None:
        cfg.control.hz = args.hz
    if args.motion_time is not None:
        cfg.control.motion_time_ms = args.motion_time

    print("=" * 70)
    print("  Manus → DG5F Retarget Receiver")
    print(f"  Mode: {'DRY RUN' if args.dry_run else 'LIVE'}")
    print(f"  UDP: {cfg.network.listen_ip}:{cfg.network.listen_port}")
    if not args.dry_run:
        print(f"  DG5F: {cfg.hand.ip}:{cfg.hand.port} ({cfg.hand.side} hand)")
    print(f"  Rate: {cfg.control.hz} Hz | Motion time: {cfg.control.motion_time_ms} ms")
    print("  Controls: Ctrl+C = Quit")
    print("=" * 70)

    # Setup retarget
    retarget = ManusToD5FRetarget(
        hand_side=cfg.hand.side,
        calibration_factors=cfg.retarget.calibration_factors,
    )

    # Setup DG5F client (unless dry-run)
    client = None
    if not args.dry_run:
        from teleop_dev.robot.hand.dg5f_client import DG5FClient
        client = DG5FClient(
            ip=cfg.hand.ip,
            port=cfg.hand.port,
            hand_side=cfg.hand.side,
        )
        try:
            client.connect()
            if cfg.control.enable_on_start:
                client.start()
            # Set motion times
            client.set_motion_times([cfg.control.motion_time_ms] * 20)
        except Exception as e:
            print(f"\n[ERROR] DG5F connection failed: {e}")
            print("        Use --dry-run to test without hardware")
            sys.exit(1)

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
    hz_count = 0
    last_data = None

    print("\n  Waiting for Manus data...\n")

    try:
        while not shutdown.is_set():
            t0 = time.perf_counter()

            data = receiver.get_latest()
            if data is not None and data is not last_data:
                last_data = data

                # Retarget
                dg5f_angles = retarget.retarget(data.joint_angles)

                # Send to DG5F
                if client is not None:
                    try:
                        client.set_positions(dg5f_angles)
                    except Exception as e:
                        print(f"\r  [WARN] DG5F write error: {e}", end="", flush=True)

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
            client.stop()
            client.disconnect()

    print(f"\n  Total frames: {frame}")
    print(f"  Total packets: {receiver.packet_count}")
    print("  Done.")


if __name__ == "__main__":
    main()
