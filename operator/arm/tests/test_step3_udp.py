#!/usr/bin/env python3
"""Step 3: UDP send/receive test.

Tests UDP communication with mock Vive packets (NO SteamVR required).
Can be run on any machine including the robot PC.

Usage:
    # Terminal 1 — receiver
    python3 -m operator.arm.tests.test_step3_udp --mode recv --port 9871

    # Terminal 2 — sender (mock data)
    python3 -m operator.arm.tests.test_step3_udp --mode send --target-ip 127.0.0.1 --port 9871

    # Auto mode — runs both in same process (loopback test)
    python3 -m operator.arm.tests.test_step3_udp --mode auto --port 9871
"""

import argparse
import json
import math
import socket
import sys
import threading
import time


def _make_mock_packet(seq: int, hz: float = 50.0) -> dict:
    """Create a mock Vive packet with slowly changing pose."""
    t = seq / hz
    return {
        "type": "vive",
        "pos": [
            0.1 * math.sin(t),
            0.5 + 0.05 * math.cos(t),
            0.3,
        ],
        "quat": [1.0, 0.0, 0.0, 0.0],  # identity
        "tracking": True,
        "buttons": {
            "estop": False,
            "reset": False,
            "quit": False,
            "speed_up": False,
            "speed_down": False,
        },
        "timestamp": time.time(),
    }


def run_sender(target_ip: str, port: int, hz: int = 50, duration: float = 2.0):
    """Send mock packets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_ip, port)
    dt = 1.0 / hz
    count = int(duration * hz)

    print(f"[Sender] Sending {count} mock packets to {target_ip}:{port}")
    for i in range(count):
        pkt = _make_mock_packet(i, hz)
        data = json.dumps(pkt).encode()
        sock.sendto(data, target)
        time.sleep(dt)

    print(f"[Sender] Done. Sent {count} packets.")
    sock.close()


def run_receiver(port: int, duration: float = 3.0):
    """Receive and validate packets."""
    print(f"[Receiver] Listening on UDP port {port} for {duration}s...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(duration + 1.0)

    packets = []
    t_start = time.time()
    passed = 0
    failed = 0

    try:
        while time.time() - t_start < duration:
            try:
                data, addr = sock.recvfrom(4096)
                pkt = json.loads(data)
                packets.append(pkt)
            except socket.timeout:
                break
            except json.JSONDecodeError:
                print(f"[WARN] Invalid JSON from {addr}")
    finally:
        sock.close()

    elapsed = time.time() - t_start

    if not packets:
        print("[FAIL] No packets received!")
        print("       Check: sender running? firewall? correct port?")
        return

    # Test 1: Packet count
    actual_hz = len(packets) / elapsed if elapsed > 0 else 0
    print(f"\n[TEST] Received {len(packets)} packets in {elapsed:.1f}s ({actual_hz:.1f} Hz)", end=" ")
    if len(packets) >= 5:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL]")
        failed += 1

    # Test 2: Packet format
    print("[TEST] Packet format validation...", end=" ")
    format_ok = True
    for i, pkt in enumerate(packets):
        if pkt.get("type") != "vive":
            print(f"\n  [FAIL] Packet {i}: missing type='vive'")
            format_ok = False
            break
        if not isinstance(pkt.get("pos"), list) or len(pkt["pos"]) != 3:
            print(f"\n  [FAIL] Packet {i}: pos must be [x,y,z]")
            format_ok = False
            break
        if not isinstance(pkt.get("quat"), list) or len(pkt["quat"]) != 4:
            print(f"\n  [FAIL] Packet {i}: quat must be [w,x,y,z]")
            format_ok = False
            break
        if "tracking" not in pkt:
            print(f"\n  [FAIL] Packet {i}: missing 'tracking' field")
            format_ok = False
            break
        if "buttons" not in pkt:
            print(f"\n  [FAIL] Packet {i}: missing 'buttons' field")
            format_ok = False
            break
        if "timestamp" not in pkt:
            print(f"\n  [FAIL] Packet {i}: missing 'timestamp' field")
            format_ok = False
            break

    if format_ok:
        print("[PASS]")
        passed += 1
    else:
        failed += 1

    # Test 3: Timestamps monotonically increasing
    print("[TEST] Timestamp monotonicity...", end=" ")
    timestamps = [p["timestamp"] for p in packets]
    monotonic = all(timestamps[i] <= timestamps[i+1] for i in range(len(timestamps)-1))
    if monotonic:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Timestamps not monotonically increasing")
        failed += 1

    # Summary
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 3 complete — proceed to Step 4")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 50}")


def run_auto(port: int):
    """Loopback test — sender + receiver in same process."""
    print("=" * 50)
    print("  Step 3: UDP Loopback Test (auto mode)")
    print("=" * 50)

    # Start receiver in background
    recv_thread = threading.Thread(
        target=run_receiver, args=(port, 3.0), daemon=True
    )
    recv_thread.start()
    time.sleep(0.2)  # Let receiver bind

    # Send mock packets
    run_sender("127.0.0.1", port, hz=50, duration=2.0)

    # Wait for receiver
    recv_thread.join(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description="Step 3: UDP test")
    parser.add_argument("--mode", choices=["send", "recv", "auto"], default="auto",
                        help="Mode: send, recv, or auto (loopback)")
    parser.add_argument("--target-ip", default="127.0.0.1",
                        help="Target IP for send mode")
    parser.add_argument("--port", type=int, default=9871, help="UDP port")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Duration in seconds")
    args = parser.parse_args()

    if args.mode == "send":
        run_sender(args.target_ip, args.port, duration=args.duration)
    elif args.mode == "recv":
        run_receiver(args.port, duration=args.duration + 1.0)
    else:
        run_auto(args.port)


if __name__ == "__main__":
    main()
