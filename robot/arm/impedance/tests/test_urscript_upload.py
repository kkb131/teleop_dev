#!/usr/bin/env python3
"""Diagnostic tool: upload and test URScripts independently.

Tests:
  freedrive  — Upload freedrive script (robot becomes compliant for 10s)
  torque     — Upload minimal direct_torque script (zero torque, gravity comp only)

Usage:
  python3 -m robot.arm.impedance.test_urscript_upload --robot-ip 192.168.0.2 --test freedrive
  python3 -m robot.arm.impedance.test_urscript_upload --robot-ip 192.168.0.2 --test torque
"""

import argparse
import socket
import time
from pathlib import Path


_UR_SECONDARY_PORT = 30002
_SCRIPT_DIR = Path(__file__).parent / "scripts"

# Minimal direct_torque test: zero torque for 5 seconds (gravity comp only)
_TORQUE_TEST_SCRIPT = """\
def test_direct_torque():
  textmsg("TestTorque: Starting")
  local zero_tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  local i = 0
  while i < 2500:
    direct_torque(zero_tau)
    i = i + 1
  end
  textmsg("TestTorque: Done (5s elapsed)")
  stopj(4.0)
end

test_direct_torque()
"""


def upload_script(robot_ip: str, script_text: str, label: str):
    """Upload URScript via TCP socket to UR Secondary Interface."""
    print(f"[Upload] Connecting to {robot_ip}:{_UR_SECONDARY_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((robot_ip, _UR_SECONDARY_PORT))
        sock.sendall(script_text.encode("utf-8") + b"\n")
        print(f"[Upload] {label} uploaded successfully")
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description="URScript upload diagnostic")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--test", required=True, choices=["freedrive", "torque"],
                        help="freedrive: 10s freedrive mode, torque: 5s zero-torque (gravity comp)")
    args = parser.parse_args()

    if args.test == "freedrive":
        script_path = _SCRIPT_DIR / "test_freedrive.script"
        if not script_path.exists():
            print(f"[Error] Script not found: {script_path}")
            return
        script_text = script_path.read_text()
        label = "test_freedrive.script"
        duration = 10
        expected = "Robot should become compliant (push by hand)"
    else:
        script_text = _TORQUE_TEST_SCRIPT
        label = "inline direct_torque test"
        duration = 5
        expected = "Robot should hold position with gravity comp (slightly compliant)"

    print(f"\n{'='*60}")
    print(f"Test: {args.test}")
    print(f"Expected: {expected}")
    print(f"Duration: {duration}s (automatic)")
    print(f"{'='*60}\n")

    upload_script(args.robot_ip, script_text, label)

    print(f"\n[Wait] Script running for {duration}s...")
    print("[Check] Look at UR teach pendant log tab for textmsg output")
    print("[Check] Try pushing the robot by hand\n")

    for i in range(duration):
        time.sleep(1.0)
        print(f"  {i+1}/{duration}s...")

    print(f"\n[Done] Script should have finished. Check UR log for results.")
    print("  - If 'TestFreedrive: Done' or 'TestTorque: Done' appears → script executed OK")
    print("  - If no message or error → check UR firmware version and Remote Control mode")


if __name__ == "__main__":
    main()
