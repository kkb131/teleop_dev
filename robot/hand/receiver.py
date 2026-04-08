#!/usr/bin/env python3
"""Manus UDP receiver → DG5F PID controller (MultiDOFCommand).

Receives DG5F joint angles via UDP from manus_sender on operator PC
and publishes to pid_controller/PidController. Sender handles all
retargeting — receiver is passthrough with EMA smoothing.

Requires dg5f_driver (PID mode) running:
    ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

Usage:
    # Dry-run (no hardware — just receive + print):
    python3 -m robot.hand.receiver --dry-run

    # ROS2 mode (dg5f_driver pid_all must be running):
    python3 -m robot.hand.receiver --hand right

    # Custom port / rate:
    python3 -m robot.hand.receiver --hand right --port 9872 --hz 60
"""

import argparse
import json
import signal
import socket
import threading
import time
from typing import Optional

import numpy as np

from protocol.hand_protocol import HandData, NUM_JOINTS, NUM_FINGERS


# ─────────────────────────────────────────────────────────
# UDP Receiver (threaded)
# ─────────────────────────────────────────────────────────

class ManusReceiver:
    """Receives Manus glove UDP packets in a background thread."""

    def __init__(self, port: int = 9872, bind_ip: str = "0.0.0.0"):
        self._port = port
        self._bind_ip = bind_ip
        self._latest: Optional[HandData] = None
        self._lock = threading.Lock()
        self._running = False
        self._pkt_count = 0
        self._last_recv_time = 0.0
        self._is_retargeted = False

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
        return self._is_retargeted

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

                self._is_retargeted = pkt.get("retargeted", False)

                buttons = pkt.get("buttons", {})
                if buttons.get("estop", False):
                    data = None

                with self._lock:
                    self._latest = data
                self._pkt_count += 1
                self._last_recv_time = time.time()

            except socket.timeout:
                continue
            except (json.JSONDecodeError, KeyError, Exception):
                continue


# ─────────────────────────────────────────────────────────
# Display
# ─────────────────────────────────────────────────────────

def _print_status(data: HandData, dg5f_angles: np.ndarray, hz: float,
                  pkt_count: int, frame: int, is_retargeted: bool = False):
    if frame > 0:
        print(f"\033[8A", end="")

    mode_str = "RETARGETED (passthrough)" if is_retargeted else "RAW (no retarget)"
    print(f"  Frame: {frame:6d} | Pkts: {pkt_count:6d} | Rate: {hz:.1f} Hz")
    print(f"  Hand: {data.hand_side.upper()} | Mode: {mode_str}")

    ab = ("Recv", "Send") if is_retargeted else ("Raw", "Send")
    joint_labels = ["Spread", "MCP", "PIP", "DIP"]
    header_parts = [f"{jl:>4s}({ab[0][0]}/{ab[1][0]})" for jl in joint_labels]
    print(f"  {'Finger':8s} {'  '.join(header_parts)}")

    finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    for f in range(5):
        b = f * 4
        parts = []
        for j in range(4):
            r_val = data.joint_angles[b + j]
            s_val = dg5f_angles[b + j]
            parts.append(f"{r_val:+6.2f}/{s_val:+6.2f}")
        print(f"  {finger_names[f]:8s} {'  '.join(parts)}")


# ─────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Manus → DG5F receiver (passthrough + EMA)"
    )
    parser.add_argument("--port", type=int, default=9872,
                        help="UDP listen port")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right"],
                        help="Hand side")
    parser.add_argument("--hz", type=int, default=60,
                        help="Control loop Hz")
    parser.add_argument("--dry-run", action="store_true",
                        help="No ROS2 publishing — just receive + print")
    parser.add_argument("--motion-time", type=int, default=50,
                        help="Per-joint motion time in ms")
    parser.add_argument("--ema-alpha", type=float, default=0.3,
                        help="EMA filter alpha (0=frozen, 1=no filter)")
    args = parser.parse_args()

    print("=" * 70)
    print("  Manus → DG5F Receiver (passthrough + EMA)")
    print(f"  Mode: {'DRY RUN' if args.dry_run else 'LIVE (ROS2)'}")
    print(f"  UDP: 0.0.0.0:{args.port}")
    print(f"  Hand: {args.hand}")
    print(f"  Rate: {args.hz} Hz | Motion time: {args.motion_time} ms | EMA: {args.ema_alpha}")
    print("  Controls: Ctrl+C = Quit")
    print("=" * 70)

    # Setup ROS2 client (unless dry-run)
    client = None
    if not args.dry_run:
        import rclpy
        rclpy.init()
        from robot.hand.dg5f_ros2_client import DG5FROS2Client
        client = DG5FROS2Client(
            hand_side=args.hand,
            motion_time_ms=args.motion_time,
        )
        print(f"  [ROS2] DG5FROS2Client initialized ({args.hand} hand)")

    # Setup UDP receiver
    receiver = ManusReceiver(port=args.port)
    receiver.start()

    # Graceful shutdown
    shutdown = threading.Event()

    def _signal_handler(sig, frame):
        print("\n\n  Shutting down...")
        shutdown.set()

    signal.signal(signal.SIGINT, _signal_handler)

    # Control loop
    dt = 1.0 / args.hz
    frame = 0
    loop_hz = 0.0
    hz_timer = time.time()
    ema_alpha = args.ema_alpha
    filtered_dg5f = None
    hz_count = 0
    last_data = None

    print("\n  Waiting for data...\n")

    try:
        while not shutdown.is_set():
            t0 = time.perf_counter()

            if client is not None:
                import rclpy
                rclpy.spin_once(client, timeout_sec=0)

            data = receiver.get_latest()
            if data is not None and data is not last_data:
                last_data = data

                # Passthrough: use joint_angles directly (sender already retargeted)
                dg5f_raw = data.joint_angles

                # EMA filter to reduce jitter
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

                _print_status(data, dg5f_angles, loop_hz, receiver.packet_count, frame,
                              is_retargeted=receiver.is_retargeted)
                frame += 1
            else:
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
            import rclpy
            rclpy.shutdown()

    print(f"\n  Total frames: {frame}")
    print(f"  Total packets: {receiver.packet_count}")
    print("  Done.")


if __name__ == "__main__":
    main()
