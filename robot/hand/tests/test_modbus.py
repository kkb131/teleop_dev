#!/usr/bin/env python3
"""Modbus TCP connection test for Tesollo DG 5F M hand.

Tests:
  - TCP connection to DG5F
  - Read current positions (input registers)
  - Read motor currents
  - System start/stop
  - Write test position (optional, with --write flag)

Usage:
    python3 -m tesollo.tests.test_modbus --ip 169.254.186.72
    python3 -m tesollo.tests.test_modbus --ip 169.254.186.72 --write
"""

import argparse
import sys

import numpy as np

from teleop_dev.robot.hand.dg5f_client import DG5FClient, NUM_MOTORS


def main():
    parser = argparse.ArgumentParser(description="DG5F Modbus TCP test")
    parser.add_argument("--ip", default="169.254.186.72",
                        help="DG5F hand IP address")
    parser.add_argument("--port", type=int, default=502,
                        help="Modbus TCP port")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right"])
    parser.add_argument("--slave-id", type=int, default=1,
                        help="Modbus slave ID (default: 1)")
    parser.add_argument("--write", action="store_true",
                        help="Test writing positions (send zeros = open hand)")
    args = parser.parse_args()

    print("=" * 55)
    print("  DG5F Modbus TCP Connection Test")
    print(f"  Target: {args.ip}:{args.port} ({args.hand} hand)")
    print(f"  Slave ID: {args.slave_id}")
    print(f"  Write test: {'YES' if args.write else 'NO'}")
    print("=" * 55)
    passed = 0
    failed = 0

    client = DG5FClient(ip=args.ip, port=args.port, hand_side=args.hand,
                        slave_id=args.slave_id)

    # ── Test 1: Connection ──────────────────────────────
    print("\n[TEST] Modbus TCP connection...", end=" ")
    try:
        client.connect()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # ── Test 2: Read positions ──────────────────────────
    print("[TEST] Read current positions...", end=" ")
    try:
        positions = client.get_positions()
        if len(positions) == NUM_MOTORS:
            print(f"[PASS] {NUM_MOTORS} joints read")
            for name, pos in zip(client.joint_names, positions):
                print(f"       {name}: {pos:+.4f} rad ({np.degrees(pos):+.1f} deg)")
            passed += 1
        else:
            print(f"[FAIL] Expected {NUM_MOTORS}, got {len(positions)}")
            failed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 3: Read currents ───────────────────────────
    print("[TEST] Read motor currents...", end=" ")
    try:
        currents = client.get_currents()
        if len(currents) == NUM_MOTORS:
            max_cur = np.max(np.abs(currents))
            print(f"[PASS] max |current| = {max_cur:.4f} A")
            passed += 1
        else:
            print(f"[FAIL] Expected {NUM_MOTORS}, got {len(currents)}")
            failed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 4: Read velocities ─────────────────────────
    print("[TEST] Read motor velocities...", end=" ")
    try:
        velocities = client.get_velocities()
        if len(velocities) == NUM_MOTORS:
            print(f"[PASS] max |vel| = {np.max(np.abs(velocities)):.4f} rad/s")
            passed += 1
        else:
            print(f"[FAIL] Expected {NUM_MOTORS}, got {len(velocities)}")
            failed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 5: Is moving ───────────────────────────────
    print("[TEST] Is moving flag...", end=" ")
    try:
        moving = client.is_moving()
        print(f"[PASS] moving={moving}")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 6: System start/stop ───────────────────────
    print("[TEST] System start...", end=" ")
    try:
        client.start()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # ── Test 7: Write positions (optional) ──────────────
    if args.write:
        print("[TEST] Write zero positions (open hand)...", end=" ")
        try:
            client.set_positions([0.0] * NUM_MOTORS)
            print("[PASS]")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {e}")
            failed += 1

    # ── Cleanup ─────────────────────────────────────────
    print("[TEST] System stop...", end=" ")
    try:
        client.stop()
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    client.disconnect()
    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] DG5F Modbus communication OK!")
        print("  Next: python3 -m tesollo.receiver --hand-ip <IP> --dry-run")
    else:
        print("  [ISSUES] Fix the above failures")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
