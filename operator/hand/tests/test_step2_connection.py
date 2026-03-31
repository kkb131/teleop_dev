#!/usr/bin/env python3
"""Step 2: Manus glove connection test (subprocess mode).

Verifies that gloves can be connected and ergonomics data
is received via the SDKClient_Linux subprocess.

Requirements:
    - SDKClient_Linux.out built (bash operator/hand/sdk/build.sh)
    - Manus gloves powered on and connected (USB)

Usage: python3 -m operator.hand.tests.test_step2_connection [--sdk-path operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out]
"""

import argparse
import sys
import time

from operator.hand.manus_reader import ManusReader


def main():
    parser = argparse.ArgumentParser(description="Step 2: Glove connection test")
    parser.add_argument("--sdk-path", default="operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"],
                        help="Which hand to test (default: right)")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 2: Manus Glove Connection Test (SDK v3.1.0)")
    print("  Make sure gloves are powered on and connected!")
    print("=" * 55)
    passed = 0
    failed = 0

    # Test 1: Create ManusReader and connect
    print("\n[TEST] Connect to Manus SDK (Integrated Mode)...", end=" ")
    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
        print("[PASS]")
        passed += 1
    except FileNotFoundError as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Check status
    print("[TEST] Reader status...", end=" ")
    status = reader.get_status()
    if status["connected"]:
        print("[PASS]")
        print(f"       SDK loaded: {status['sdk_loaded']}")
        print(f"       Connected: {status['connected']}")
        print(f"       Hand side: {status['hand_side']}")
        print(f"       Left glove ID: {status['left_glove_id']}")
        print(f"       Right glove ID: {status['right_glove_id']}")
        passed += 1
    else:
        print("[FAIL] Not connected")
        failed += 1

    # Test 3: Wait for ergonomics callback data
    print("[TEST] Wait for ergonomics stream data...", end=" ")
    got_data = reader.wait_for_data(timeout=10.0)
    if got_data:
        print("[PASS] Ergonomics callback received data")
        passed += 1
    else:
        print("[FAIL] No ergonomics data after 10s")
        print("       Check: glove powered on? Connected? Within range?")
        failed += 1

    # Test 4: Read hand data from callback cache
    print("[TEST] Read hand data from cache...", end=" ")
    time.sleep(0.5)  # let a few callbacks accumulate

    data = reader.get_hand_data()
    if data is not None:
        print("[PASS]")
        print(f"       Hand side: {data.hand_side}")
        print(f"       Joint angles shape: {data.joint_angles.shape}")
        print(f"       Timestamp: {data.timestamp:.3f}")

        nonzero = (data.joint_angles != 0).sum()
        print(f"       Non-zero joints: {nonzero}/{len(data.joint_angles)}")
        passed += 1
    else:
        print("[WARN] No data in cache")
        print("       Trying 5 more times with 1s delay...")

        got_data = False
        for attempt in range(5):
            time.sleep(1.0)
            data = reader.get_hand_data()
            if data is not None:
                print(f"       Got data on attempt {attempt + 2}!")
                got_data = True
                passed += 1
                break
            print(f"       Attempt {attempt + 2}: no data")

        if not got_data:
            print("[FAIL] No data received after retries")
            failed += 1

    # Test 5: Read from both hands (if applicable)
    if args.hand == "both":
        print("[TEST] Read both hands...", end=" ")
        hands = reader.get_both_hands()
        for side, hdata in hands.items():
            if hdata is not None:
                nonzero = (hdata.joint_angles != 0).sum()
                print(f"\n       {side}: {nonzero} non-zero joints")
            else:
                print(f"\n       {side}: no data")
        passed += 1

    # Test 6: Quick burst read (check for errors)
    print("[TEST] Burst read (10 frames)...", end=" ")
    success_count = 0
    error_count = 0
    for i in range(10):
        try:
            data = reader.get_hand_data()
            if data is not None:
                success_count += 1
        except Exception:
            error_count += 1
        time.sleep(0.05)

    if error_count == 0:
        print(f"[PASS] {success_count}/10 successful reads, 0 errors")
        passed += 1
    else:
        print(f"[FAIL] {error_count}/10 read errors")
        failed += 1

    # Cleanup
    reader.disconnect()
    print("\n[INFO] Disconnected from SDK")

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 2 complete — proceed to Step 3")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
