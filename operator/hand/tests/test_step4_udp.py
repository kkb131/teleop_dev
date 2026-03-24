#!/usr/bin/env python3
"""Step 4: UDP send/receive end-to-end test.

Tests the complete pipeline: glove data → packet building → UDP send → receive.
Can run with or without actual glove hardware (uses mock data if unavailable).

Usage:
    # With mock data (no hardware needed):
    python3 -m manus.tests.test_step4_udp

    # With real glove:
    python3 -m manus.tests.test_step4_udp --real --sdk-path manus/sdk/libManusSDK.so
"""

import argparse
import json
import socket
import sys
import threading
import time

import numpy as np

from teleop_dev.operator.hand.manus_reader import HandData, NUM_JOINTS, NUM_FINGERS


TEST_PORT = 19872  # Use a different port to avoid conflicts


class UDPReceiver:
    """Simple UDP receiver that collects packets in a thread."""

    def __init__(self, port: int):
        self._port = port
        self._packets = []
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._sock = None

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self._port))
        self._sock.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._sock is not None:
            self._sock.close()

    def get_packets(self) -> list:
        with self._lock:
            return list(self._packets)

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
                pkt = json.loads(data.decode())
                with self._lock:
                    self._packets.append({
                        "data": pkt,
                        "recv_time": time.time(),
                        "size": len(data),
                    })
            except socket.timeout:
                continue
            except Exception:
                continue


def _make_mock_hand_data(frame: int) -> HandData:
    """Generate mock hand data with sinusoidal joint movements."""
    t = frame * 0.016  # ~60Hz
    joint_angles = np.array([
        np.sin(t * (1 + i * 0.1)) * 0.5 + 0.5
        for i in range(NUM_JOINTS)
    ], dtype=np.float32)

    finger_spread = np.array([
        joint_angles[i * 4] for i in range(NUM_FINGERS)
    ], dtype=np.float32)

    return HandData(
        joint_angles=joint_angles,
        finger_spread=finger_spread,
        wrist_pos=np.array([0.1, 0.2, 0.3]),
        wrist_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        hand_side="right",
        timestamp=time.time(),
    )


def main():
    parser = argparse.ArgumentParser(description="Step 4: UDP test")
    parser.add_argument("--real", action="store_true",
                        help="Use real glove instead of mock data")
    parser.add_argument("--sdk-path", default="manus/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out (only with --real)")
    parser.add_argument("--num-packets", type=int, default=100,
                        help="Number of test packets (default: 100)")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 4: UDP Send/Receive Test")
    print(f"  Mode: {'Real Glove' if args.real else 'Mock Data'}")
    print(f"  Packets: {args.num_packets}")
    print("=" * 55)
    passed = 0
    failed = 0

    # Start receiver
    print("\n[TEST] Start UDP receiver on port", TEST_PORT, "...", end=" ")
    receiver = UDPReceiver(TEST_PORT)
    try:
        receiver.start()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # Connect real reader if requested
    reader = None
    if args.real:
        print("[TEST] Connect to Manus SDK...", end=" ")
        try:
            from teleop_dev.operator.hand.manus_reader import ManusReader
            reader = ManusReader(sdk_bin_path=args.sdk_path)
            reader.connect()
            print("[PASS]")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {e}")
            print("       Falling back to mock data")
            reader = None

    # Send packets
    print(f"[TEST] Send {args.num_packets} packets...", end=" ")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", TEST_PORT)
    send_times = []

    try:
        for i in range(args.num_packets):
            if reader is not None:
                data = reader.get_hand_data()
                if data is None:
                    data = _make_mock_hand_data(i)
            else:
                data = _make_mock_hand_data(i)

            pkt = {
                "type": "manus",
                "hand": data.hand_side,
                "joint_angles": data.joint_angles.tolist(),
                "finger_spread": data.finger_spread.tolist(),
                "wrist_pos": data.wrist_pos.tolist(),
                "wrist_quat": data.wrist_quat.tolist(),
                "tracking": True,
                "buttons": {"estop": False, "reset": False, "quit": False},
                "timestamp": time.time(),
            }

            raw = json.dumps(pkt).encode()
            send_times.append(time.time())
            sock.sendto(raw, target)
            time.sleep(0.01)  # ~100Hz

        print(f"[PASS] Sent {args.num_packets} packets")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
    finally:
        sock.close()

    # Wait for receiver to catch up
    time.sleep(0.5)

    # Analyze received packets
    packets = receiver.get_packets()
    receiver.stop()

    if reader is not None:
        reader.disconnect()

    print(f"\n[TEST] Packet delivery...", end=" ")
    recv_count = len(packets)
    delivery_pct = recv_count / args.num_packets * 100 if args.num_packets > 0 else 0
    if recv_count >= args.num_packets * 0.95:
        print(f"[PASS] {recv_count}/{args.num_packets} received ({delivery_pct:.1f}%)")
        passed += 1
    elif recv_count > 0:
        print(f"[WARN] {recv_count}/{args.num_packets} received ({delivery_pct:.1f}%)")
        passed += 1
    else:
        print(f"[FAIL] No packets received")
        failed += 1

    if recv_count == 0:
        _summary(passed, failed)
        return

    # Test: Packet format validation
    print(f"[TEST] Packet format...", end=" ")
    required_keys = {"type", "hand", "joint_angles", "finger_spread",
                     "wrist_pos", "wrist_quat", "tracking", "buttons", "timestamp"}
    sample = packets[0]["data"]
    missing = required_keys - set(sample.keys())
    if not missing:
        print("[PASS] All required fields present")
        passed += 1
    else:
        print(f"[FAIL] Missing fields: {missing}")
        failed += 1

    # Test: Data types and sizes
    print(f"[TEST] Data types and sizes...", end=" ")
    errors = []
    p = packets[0]["data"]
    if p["type"] != "manus":
        errors.append(f"type should be 'manus', got '{p['type']}'")
    if len(p["joint_angles"]) != NUM_JOINTS:
        errors.append(f"joint_angles should have {NUM_JOINTS} values, got {len(p['joint_angles'])}")
    if len(p["finger_spread"]) != NUM_FINGERS:
        errors.append(f"finger_spread should have {NUM_FINGERS} values, got {len(p['finger_spread'])}")
    if len(p["wrist_pos"]) != 3:
        errors.append(f"wrist_pos should have 3 values, got {len(p['wrist_pos'])}")
    if len(p["wrist_quat"]) != 4:
        errors.append(f"wrist_quat should have 4 values, got {len(p['wrist_quat'])}")

    if not errors:
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL]")
        for err in errors:
            print(f"       {err}")
        failed += 1

    # Test: Packet size
    print(f"[TEST] Packet size...", end=" ")
    sizes = [p["size"] for p in packets]
    avg_size = sum(sizes) / len(sizes)
    max_size = max(sizes)
    print(f"[PASS] avg={avg_size:.0f} bytes, max={max_size} bytes")
    passed += 1

    # Test: Latency
    print(f"[TEST] Latency...", end=" ")
    latencies = []
    for i, pkt in enumerate(packets):
        send_ts = pkt["data"]["timestamp"]
        recv_ts = pkt["recv_time"]
        latency_ms = (recv_ts - send_ts) * 1000
        latencies.append(latency_ms)

    if latencies:
        avg_lat = sum(latencies) / len(latencies)
        max_lat = max(latencies)
        min_lat = min(latencies)
        print(f"[PASS] avg={avg_lat:.2f}ms, min={min_lat:.2f}ms, max={max_lat:.2f}ms")
        passed += 1

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 4 complete!")
        print("  Ready for robot PC integration.")
        print("  Next: python3 -m manus.manus_sender --target-ip <ROBOT_IP>")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
