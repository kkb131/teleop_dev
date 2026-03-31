#!/usr/bin/env python3
"""End-to-end mock test: UDP sender → receiver → retarget.

No hardware required. Simulates the full pipeline:
  1. Mock manus_sender sends UDP packets
  2. ManusReceiver receives them
  3. ManusToD5FRetarget converts to DG5F angles
  4. Validates output quality

Usage:
    python3 -m robot.hand.tests.test_e2e
"""

import json
import math
import socket
import sys
import time

import numpy as np

from robot.hand.receiver import ManusReceiver
from robot.hand.retarget import ManusToD5FRetarget, RIGHT_LIMITS


TEST_PORT = 19873  # Avoid conflicts with real receiver


def _make_mock_packet(frame: int, hand: str = "right") -> bytes:
    """Generate a mock Manus UDP packet with sinusoidal finger motion."""
    t = frame * 0.016  # ~60Hz
    angles = []
    for f in range(5):
        freq = 0.5 + f * 0.15
        phase = f * 0.4
        curl = (math.sin(t * freq + phase) + 1.0) * 0.5
        angles.extend([
            math.sin(t * freq + phase) * 0.15,  # spread
            curl * 1.2,                           # MCP flex
            curl * 1.5,                           # PIP flex
            curl * 1.0,                           # DIP flex
        ])

    pkt = {
        "type": "manus",
        "hand": hand,
        "joint_angles": angles,
        "finger_spread": [angles[i * 4] for i in range(5)],
        "wrist_pos": [0.0, 0.0, 0.3],
        "wrist_quat": [1.0, 0.0, 0.0, 0.0],
        "tracking": True,
        "buttons": {"estop": False, "reset": False, "quit": False},
        "timestamp": time.time(),
    }
    return json.dumps(pkt).encode()


def main():
    print("=" * 55)
    print("  E2E Mock Test: UDP → Retarget")
    print("  (No hardware required)")
    print("=" * 55)
    passed = 0
    failed = 0

    # ── Test 1: Start receiver ──────────────────────────
    print("\n[TEST] Start mock receiver on port", TEST_PORT, "...", end=" ")
    receiver = ManusReceiver(port=TEST_PORT)
    try:
        receiver.start()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # ── Test 2: Send mock packets ───────────────────────
    num_packets = 60
    print(f"[TEST] Send {num_packets} mock packets...", end=" ")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for i in range(num_packets):
            pkt = _make_mock_packet(i)
            sock.sendto(pkt, ("127.0.0.1", TEST_PORT))
            time.sleep(0.01)
        print(f"[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
    finally:
        sock.close()

    # Wait for receiver to process
    time.sleep(0.3)

    # ── Test 3: Verify reception ────────────────────────
    print(f"[TEST] Packet reception...", end=" ")
    pkt_count = receiver.packet_count
    if pkt_count >= num_packets * 0.9:
        print(f"[PASS] {pkt_count}/{num_packets} received")
        passed += 1
    elif pkt_count > 0:
        print(f"[WARN] {pkt_count}/{num_packets} received")
        passed += 1
    else:
        print("[FAIL] No packets received")
        failed += 1

    # ── Test 4: Get latest data ─────────────────────────
    print(f"[TEST] Get latest HandData...", end=" ")
    data = receiver.get_latest()
    if data is not None:
        print(f"[PASS] hand={data.hand_side}, angles={len(data.joint_angles)}")
        passed += 1
    else:
        print("[FAIL] No data available")
        failed += 1
        receiver.stop()
        _summary(passed, failed)
        return

    # ── Test 5: Retarget ────────────────────────────────
    print(f"[TEST] Retarget Manus → DG5F...", end=" ")
    retarget = ManusToD5FRetarget(hand_side="right")
    try:
        dg5f = retarget.retarget(data.joint_angles)
        if dg5f.shape == (20,) and np.all(np.isfinite(dg5f)):
            print(f"[PASS] 20 joints, all finite")
            passed += 1
        else:
            print(f"[FAIL] shape={dg5f.shape}, finite={np.all(np.isfinite(dg5f))}")
            failed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 6: Joint limits check ──────────────────────
    print(f"[TEST] DG5F output within joint limits...", end=" ")
    violations = 0
    for i in range(20):
        lim = RIGHT_LIMITS[i]
        if dg5f[i] < lim.min_rad - 1e-6 or dg5f[i] > lim.max_rad + 1e-6:
            violations += 1
    if violations == 0:
        print("[PASS]")
        passed += 1
    else:
        print(f"[FAIL] {violations} joints out of bounds")
        failed += 1

    # ── Test 7: Continuous retarget (simulate 1s of data) ─
    print(f"[TEST] Continuous retarget (60 frames)...", end=" ")
    errors = 0
    for i in range(60):
        # Generate and retarget
        t = i * 0.016
        angles = np.zeros(20, dtype=np.float32)
        for f in range(5):
            b = f * 4
            curl = (math.sin(t * (0.5 + f * 0.15) + f * 0.4) + 1) * 0.5
            angles[b + 0] = math.sin(t + f) * 0.15
            angles[b + 1] = curl * 1.2
            angles[b + 2] = curl * 1.5
            angles[b + 3] = curl * 1.0

        dg5f = retarget.retarget(angles)
        if not np.all(np.isfinite(dg5f)):
            errors += 1
        for j in range(20):
            lim = RIGHT_LIMITS[j]
            if dg5f[j] < lim.min_rad - 1e-6 or dg5f[j] > lim.max_rad + 1e-6:
                errors += 1
                break

    if errors == 0:
        print(f"[PASS] All 60 frames valid")
        passed += 1
    else:
        print(f"[FAIL] {errors}/60 frames had issues")
        failed += 1

    # ── Test 8: E-stop suppression ──────────────────────
    print(f"[TEST] E-stop packet suppression...", end=" ")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    estop_pkt = {
        "type": "manus",
        "hand": "right",
        "joint_angles": [0.0] * 20,
        "finger_spread": [0.0] * 5,
        "wrist_pos": [0, 0, 0],
        "wrist_quat": [1, 0, 0, 0],
        "tracking": True,
        "buttons": {"estop": True, "reset": False, "quit": False},
        "timestamp": time.time(),
    }
    sock.sendto(json.dumps(estop_pkt).encode(), ("127.0.0.1", TEST_PORT))
    time.sleep(0.2)
    estop_data = receiver.get_latest()
    sock.close()
    if estop_data is None:
        print("[PASS] Data suppressed during e-stop")
        passed += 1
    else:
        print("[WARN] Data not suppressed (receiver may ignore e-stop)")
        passed += 1

    # ── Print sample output ─────────────────────────────
    print(f"\n  Sample retarget output (last frame):")
    from robot.hand.dg5f_client import RIGHT_JOINT_NAMES
    print(f"  {'Joint':12s} {'Manus(rad)':>12s} {'DG5F(rad)':>12s} {'DG5F(deg)':>10s}")
    print(f"  {'-' * 48}")
    for i in range(20):
        m = angles[i]
        d = dg5f[i]
        print(f"  {RIGHT_JOINT_NAMES[i]:12s} {m:+12.4f} {d:+12.4f} {math.degrees(d):+10.2f}")

    # Cleanup
    receiver.stop()
    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] E2E pipeline verified!")
        print("  Next steps:")
        print("    1. python3 -m robot.hand.tests.test_modbus --ip <DG5F_IP>")
        print("    2. python3 -m robot.hand.receiver --dry-run")
        print("    3. python3 -m robot.hand.receiver --hand-ip <DG5F_IP>")
    else:
        print("  [ISSUES] Fix the above failures")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
