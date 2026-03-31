#!/usr/bin/env python3
"""Step 1: Manus SDK binary and dongle detection test.

Verifies that the SDKClient_Linux.out binary exists and USB
dongle is detected.

Requirements:
    - SDKClient_Linux.out built (bash sender/hand/sdk/build.sh)
    - Manus USB dongle plugged in (optional)

Usage: python3 -m sender.hand.tests.test_step1_sdk [--sdk-path sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out]
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Step 1: SDK binary check")
    parser.add_argument("--sdk-path",
                        default="sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 1: Manus SDK Binary & Dongle Check")
    print("=" * 55)
    passed = 0
    failed = 0

    # Test 1: Binary exists
    print("\n[TEST] SDK binary exists...", end=" ")
    sdk_path = Path(args.sdk_path)
    if sdk_path.exists():
        size_mb = sdk_path.stat().st_size / (1024 * 1024)
        print(f"[PASS] {sdk_path} ({size_mb:.1f} MB)")
        passed += 1
    else:
        print(f"[FAIL] Not found: {sdk_path.resolve()}")
        print(f"       Build: cd manus/sdk && bash build.sh")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Binary is executable
    print("[TEST] Binary is executable...", end=" ")
    if os.access(sdk_path, os.X_OK):
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Not executable. Run: chmod +x " + str(sdk_path))
        failed += 1

    # Test 3: ManusSDK .so libraries exist
    print("[TEST] ManusSDK shared libraries...", end=" ")
    lib_dir = sdk_path.parent / "ManusSDK" / "lib"
    so_files = list(lib_dir.glob("*.so")) if lib_dir.exists() else []
    if so_files:
        names = ", ".join(f.name for f in so_files)
        print(f"[PASS] {names}")
        passed += 1
    else:
        print(f"[FAIL] No .so files in {lib_dir}")
        print(f"       Download SDK and place .so files in: {lib_dir}")
        failed += 1

    # Test 4: Quick launch test (--stream-json, timeout 3s)
    print("[TEST] Binary launches successfully...", end=" ")
    try:
        proc = subprocess.Popen(
            [str(sdk_path.resolve()), "--stream-json"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(sdk_path.parent),
            env={**os.environ, "LD_LIBRARY_PATH":
                 str(lib_dir) + ":" + os.environ.get("LD_LIBRARY_PATH", "")},
        )
        # Wait briefly for initial output or error
        import time
        time.sleep(3)
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace")
            print(f"[FAIL] Exited with code {proc.returncode}")
            if stderr:
                print(f"       stderr: {stderr[:200]}")
            failed += 1
        else:
            print("[PASS] Process running")
            proc.terminate()
            proc.wait(timeout=2)
            passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # Test 5: USB device scan
    print("\n[TEST] USB device scan (lsusb)...", end=" ")
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True)
        manus_lines = [l for l in result.stdout.split("\n")
                       if "manus" in l.lower() or "3325" in l]
        if manus_lines:
            print("[PASS] Manus device detected:")
            for line in manus_lines:
                print(f"       {line.strip()}")
            passed += 1
        else:
            print("[INFO] No Manus device in lsusb")
            passed += 1
    except FileNotFoundError:
        print("[SKIP] lsusb not available")
        passed += 1

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 1 complete — proceed to Step 2")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
