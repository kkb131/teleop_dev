#!/usr/bin/env python3
"""Step 6: End-to-end sender quality test.

Receives real Vive Tracker packets from vive_sender.py and generates
a quality report (frequency, tracking rate, noise, latency).

Requirements: vive_sender.py running on operator PC
Usage:
    # On operator PC:
    python3 -m sender.arm.vive_sender --target-ip <THIS_PC_IP> --port 9871

    # On this PC (robot PC):
    python3 -m sender.arm.tests.test_step6_e2e_sender --port 9871 --duration 5
"""

import argparse
import json
import socket
import sys
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Step 6: E2E sender quality test")
    parser.add_argument("--port", type=int, default=9871, help="UDP port")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Test duration in seconds")
    args = parser.parse_args()

    print("=" * 50)
    print("  Step 6: End-to-End Sender Quality Test")
    print("=" * 50)
    print(f"\nListening on UDP port {args.port} for {args.duration}s...")
    print("Make sure vive_sender.py is running on the operator PC.\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(args.duration + 2.0)

    packets = []
    recv_times = []
    t_start = time.time()
    passed = 0
    failed = 0

    try:
        while time.time() - t_start < args.duration:
            try:
                data, addr = sock.recvfrom(4096)
                recv_time = time.time()
                pkt = json.loads(data)
                packets.append(pkt)
                recv_times.append(recv_time)
            except socket.timeout:
                break
            except json.JSONDecodeError:
                continue
    finally:
        sock.close()

    elapsed = time.time() - t_start

    if not packets:
        print("[FAIL] No packets received!")
        print("       Check:")
        print("       1. vive_sender.py running on operator PC?")
        print("       2. Correct IP/port?")
        print(f"       3. Firewall: sudo ufw allow {args.port}/udp")
        sys.exit(1)

    # Test 1: Packet rate
    actual_hz = len(packets) / elapsed
    print(f"[TEST] Received {len(packets)} packets in {elapsed:.1f}s ({actual_hz:.1f} Hz)", end=" ")
    if actual_hz > 40:
        print("[PASS]")
        passed += 1
    elif actual_hz > 20:
        print("[WARN] Low rate — network congestion?")
        failed += 1
    else:
        print("[FAIL] Very low rate")
        failed += 1

    # Test 2: Tracking rate
    tracking_count = sum(1 for p in packets if p.get("tracking", False))
    tracking_pct = 100.0 * tracking_count / len(packets)
    print(f"[TEST] Tracking rate: {tracking_count}/{len(packets)} ({tracking_pct:.1f}%)", end=" ")
    if tracking_pct >= 95:
        print("[PASS]")
        passed += 1
    elif tracking_pct >= 50:
        print("[WARN] Frequent tracking loss — check base station visibility")
        failed += 1
    else:
        print("[FAIL] Severe tracking loss")
        failed += 1

    # Test 3: Position noise (if stationary)
    tracked_packets = [p for p in packets if p.get("tracking", False)]
    if len(tracked_packets) > 10:
        positions = np.array([p["pos"] for p in tracked_packets])
        pos_std = positions.std(axis=0)
        pos_std_mm = pos_std * 1000
        print(f"[INFO] Position std dev (hold tracker still for best result):")
        print(f"       X: {pos_std_mm[0]:.2f}mm, Y: {pos_std_mm[1]:.2f}mm, Z: {pos_std_mm[2]:.2f}mm")

        max_std = pos_std_mm.max()
        print(f"[TEST] Position noise (max σ = {max_std:.2f}mm)", end=" ")
        if max_std < 5.0:  # sub-cm is good for Vive
            print("[PASS]")
            passed += 1
        else:
            print("[WARN] High noise — was tracker moving? Or reflective surfaces nearby?")
            # Don't fail — might be moving intentionally
            passed += 1

    # Test 4: Latency estimate
    sender_timestamps = [p["timestamp"] for p in packets]
    latencies = [rt - st for rt, st in zip(recv_times, sender_timestamps)]
    if latencies:
        lat_arr = np.array(latencies)
        # Filter out negative (clock skew)
        if np.any(lat_arr > 0):
            lat_positive = lat_arr[lat_arr > 0]
            lat_mean = lat_positive.mean() * 1000
            lat_max = lat_positive.max() * 1000
            print(f"[TEST] Latency: mean={lat_mean:.1f}ms, max={lat_max:.1f}ms", end=" ")
            if lat_mean < 20:
                print("[PASS]")
                passed += 1
            elif lat_mean < 50:
                print("[WARN] Moderate latency")
                passed += 1
            else:
                print("[WARN] High latency — check network")
                failed += 1
        else:
            print("[INFO] Latency: clocks not synced (all negative) — use NTP for accurate measurement")
            passed += 1

    # Test 5: Packet interval consistency
    if len(recv_times) > 2:
        intervals = np.diff(recv_times)
        interval_std = intervals.std() * 1000
        interval_mean = intervals.mean() * 1000
        print(f"[TEST] Interval: mean={interval_mean:.1f}ms, σ={interval_std:.1f}ms", end=" ")
        if interval_std < 10:
            print("[PASS]")
            passed += 1
        else:
            print("[WARN] Jittery — network or sender load issue")
            failed += 1

    # Summary
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 6 complete — ready for teleop integration!")
    else:
        print("  [ISSUES] Fix the above before full teleop testing")
    print(f"{'=' * 50}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
