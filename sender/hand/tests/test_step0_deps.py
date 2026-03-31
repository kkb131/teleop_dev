#!/usr/bin/env python3
"""Step 0: System dependency verification.

Checks that all required system packages and Python modules are installed.
No hardware (dongle/gloves) required for this step.

Usage: python3 -m sender.hand.tests.test_step0_deps
"""

import shutil
import subprocess
import sys


def main():
    print("=" * 55)
    print("  Step 0: System Dependency Verification")
    print("  (No hardware required)")
    print("=" * 55)
    passed = 0
    failed = 0

    # ── System packages ──────────────────────────────────

    # Test 1: gcc (build-essential)
    print("\n[TEST] gcc (build-essential)...", end=" ")
    if shutil.which("gcc"):
        ver = subprocess.check_output(["gcc", "--version"],
                                      text=True).split("\n")[0]
        print(f"[PASS] {ver}")
        passed += 1
    else:
        print("[FAIL] gcc not found. Run: sudo apt install build-essential")
        failed += 1

    # Test 2: pkg-config
    print("[TEST] pkg-config...", end=" ")
    if shutil.which("pkg-config"):
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Run: sudo apt install pkg-config")
        failed += 1

    # Test 3: libusb-1.0-0-dev
    print("[TEST] libusb-1.0-0-dev...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "libusb-1.0"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Run: sudo apt install libusb-1.0-0-dev")
        failed += 1

    # Test 4: libudev-dev
    print("[TEST] libudev-dev...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "libudev"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Run: sudo apt install libudev-dev")
        failed += 1

    # Test 5: libncurses5-dev (check for ncurses header)
    print("[TEST] libncurses-dev...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "ncurses"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS]")
        passed += 1
    else:
        # Fallback: check header directly
        import os
        if os.path.exists("/usr/include/ncurses.h"):
            print("[PASS] (header found)")
            passed += 1
        else:
            print("[FAIL] Run: sudo apt install libncurses5-dev")
            failed += 1

    # Test 6: zlib1g-dev
    print("[TEST] zlib1g-dev...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "zlib"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] Run: sudo apt install zlib1g-dev")
        failed += 1

    # ── gRPC / Protobuf (Remote Mode only, informational) ──

    print("\n[TEST] gRPC (Remote Mode only)...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "grpc++"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS] gRPC installed")
        passed += 1
    else:
        # Check ldconfig as fallback
        ret2 = subprocess.run(
            ["ldconfig", "-p"],
            capture_output=True, text=True,
        )
        if ret2.returncode == 0 and "libgrpc" in ret2.stdout:
            print("[PASS] gRPC found via ldconfig")
            passed += 1
        else:
            print("[INFO] gRPC not found (only needed for Remote Mode)")
            print("       Integrated Mode (direct USB dongle) does NOT require gRPC.")
            print("       If needed: see https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/")
            passed += 1  # informational

    print("[TEST] Protobuf (Remote Mode only)...", end=" ")
    ret = subprocess.run(
        ["pkg-config", "--exists", "protobuf"],
        capture_output=True,
    )
    if ret.returncode == 0:
        print("[PASS] Protobuf installed")
        passed += 1
    else:
        ret2 = subprocess.run(
            ["ldconfig", "-p"],
            capture_output=True, text=True,
        )
        if ret2.returncode == 0 and "libprotobuf" in ret2.stdout:
            print("[PASS] Protobuf found via ldconfig")
            passed += 1
        else:
            print("[INFO] Protobuf not found (only needed for Remote Mode)")
            print("       Integrated Mode (direct USB dongle) does NOT require Protobuf.")
            passed += 1  # informational

    # ── Python version ───────────────────────────────────

    print(f"\n[TEST] Python >= 3.10...", end=" ")
    ver = sys.version_info
    if ver >= (3, 10):
        print(f"[PASS] Python {ver.major}.{ver.minor}.{ver.micro}")
        passed += 1
    else:
        print(f"[FAIL] Python {ver.major}.{ver.minor} — need 3.10+")
        failed += 1

    # ── Python packages ──────────────────────────────────

    # numpy
    print("[TEST] numpy...", end=" ")
    try:
        import numpy as np
        print(f"[PASS] numpy {np.__version__}")
        passed += 1
    except ImportError:
        print("[FAIL] Run: pip install numpy")
        failed += 1

    # pyyaml
    print("[TEST] pyyaml...", end=" ")
    try:
        import yaml
        print(f"[PASS] PyYAML {yaml.__version__}")
        passed += 1
    except ImportError:
        print("[FAIL] Run: pip install pyyaml")
        failed += 1

    # pynput
    print("[TEST] pynput...", end=" ")
    try:
        import pynput
        print(f"[PASS] pynput {pynput.__version__}")
        passed += 1
    except ImportError:
        print("[FAIL] Run: pip install pynput")
        failed += 1

    # ── Conda environment (informational) ─────────────────

    import os

    print("\n[TEST] Conda environment (tamp_sender)...", end=" ")
    conda_env = os.environ.get("CONDA_DEFAULT_ENV", "")
    if conda_env == "tamp_sender":
        print(f"[PASS] active: {conda_env}")
        passed += 1
    elif conda_env:
        print(f"[INFO] active: '{conda_env}' (expected 'tamp_sender')")
        print("       Run: conda activate tamp_sender")
        passed += 1  # informational, not a hard failure
    else:
        print("[INFO] No conda env active")
        print("       Recommended: conda activate tamp_sender")
        print("       See docs/host_setup_guide.md for setup")
        passed += 1  # informational

    # ── LD_LIBRARY_PATH (informational) ─────────────────

    print("[TEST] LD_LIBRARY_PATH (manus SDK)...", end=" ")
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    if "manus" in ld_path and "sdk" in ld_path:
        print(f"[PASS] SDK path found in LD_LIBRARY_PATH")
        passed += 1
    else:
        print("[INFO] manus/sdk not in LD_LIBRARY_PATH")
        print("       If using conda, run:")
        print("         mkdir -p $CONDA_PREFIX/etc/conda/activate.d")
        print("         cat > $CONDA_PREFIX/etc/conda/activate.d/manus_env.sh << 'EOF'")
        print("         #!/bin/bash")
        print("         export MANUS_SDK_PATH=~/tamp_ws/src/tamp_dev/manus/sdk")
        print("         export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MANUS_SDK_PATH")
        print("         EOF")
        print("         conda deactivate && conda activate tamp_sender")
        passed += 1  # informational

    # ── udev rules ───────────────────────────────────────
    print("\n[TEST] udev rules (70-manus-hid.rules)...", end=" ")
    udev_path = "/etc/udev/rules.d/70-manus-hid.rules"
    if os.path.exists(udev_path):
        print(f"[PASS] {udev_path}")
        passed += 1
    else:
        local_rules = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "udev", "70-manus-hid.rules",
        )
        print("[WARN] Not installed yet")
        print(f"       Run: sudo cp {local_rules} /etc/udev/rules.d/")
        print("            sudo udevadm control --reload-rules")
        print("            sudo udevadm trigger")
        # Not a hard failure — can install later
        passed += 1

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 0 complete — proceed to Step 1")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
