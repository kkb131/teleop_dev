#!/usr/bin/env python3
"""Diagnostic: upload impedance_pd.script file directly (no RTDE connections).

Isolates whether the issue is the script content or URScriptManager's RTDE flow.

Usage:
  python3 -m standalone.teleop_impedance.test_script_upload --robot-ip 192.168.0.2
"""

import argparse
import socket
import time
from pathlib import Path

_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"
_UR_SECONDARY_PORT = 30002


def main():
    parser = argparse.ArgumentParser(description="Upload impedance_pd.script directly")
    parser.add_argument("--robot-ip", required=True)
    args = parser.parse_args()

    if not _SCRIPT_PATH.exists():
        print(f"[ERROR] Script not found: {_SCRIPT_PATH}")
        return

    script = _SCRIPT_PATH.read_text()

    print(f"Script path: {_SCRIPT_PATH}")
    print(f"Resolved:    {_SCRIPT_PATH.resolve()}")
    print(f"Length:       {len(script)} bytes")
    print(f"Lines:        {len(script.splitlines())}")
    print(f"Non-ASCII:   {any(ord(c) > 127 for c in script)}")
    print()
    print("--- First 20 lines ---")
    for i, line in enumerate(script.splitlines()[:20], 1):
        print(f"  {i:3d}: {line}")
    print("---")
    print()

    print(f"Uploading to {args.robot_ip}:{_UR_SECONDARY_PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((args.robot_ip, _UR_SECONDARY_PORT))
        sock.sendall(script.encode("utf-8") + b"\n")
        print("Upload OK")
    finally:
        sock.close()

    print()
    print("Waiting 5s... Check UR panel for 'TorqueRelay' messages.")
    print("  - If messages appear -> script content is OK (RTDE connection order issue)")
    print("  - If NO messages     -> script content has parse error")
    for i in range(5):
        time.sleep(1.0)
        print(f"  {i+1}/5s...")

    print("\nDone.")


if __name__ == "__main__":
    main()
