#!/usr/bin/env python3
"""Basic stream test: verify SDKClient_Linux.out --stream-json output
and Python subprocess reading.

Usage:
    python3 -m operator.hand.tests.test_stream_basic
    python3 -m operator.hand.tests.test_stream_basic --sdk-path operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out
    python3 -m operator.hand.tests.test_stream_basic --duration 10
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Basic stream JSON test")
    parser.add_argument("--sdk-path",
                        default="operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Test duration in seconds (default: 5)")
    args = parser.parse_args()

    bin_path = Path(args.sdk_path)
    if not bin_path.exists():
        print(f"[FAIL] Binary not found: {bin_path.resolve()}")
        sys.exit(1)

    bin_resolved = bin_path.resolve()
    cwd = bin_resolved.parent
    sdk_lib_dir = cwd / "ManusSDK" / "lib"

    env = dict(os.environ)
    ld_path = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{sdk_lib_dir}:{ld_path}" if ld_path else str(sdk_lib_dir)

    print("=" * 60)
    print("  Basic Stream JSON Test")
    print(f"  Binary: {bin_resolved}")
    print(f"  Duration: {args.duration}s")
    print("=" * 60)

    # ── Test 1: Launch and read with readline() ──────────────
    print(f"\n[TEST 1] Launch subprocess, read {args.duration}s with readline()...")

    proc = subprocess.Popen(
        [str(bin_resolved), "--stream-json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(cwd),
        env=env,
    )

    # Check process started
    time.sleep(1.0)
    if proc.poll() is not None:
        stderr = proc.stderr.read().decode(errors="replace")
        print(f"  [FAIL] Process exited with code {proc.returncode}")
        print(f"  stderr: {stderr[:300]}")
        sys.exit(1)
    print("  Process started OK")

    # Dump ALL raw lines to file for inspection
    dump_path = Path("stream_dump.txt")
    dump_file = open(dump_path, "w")
    print(f"  Dumping raw lines to: {dump_path.resolve()}")

    # Read lines
    left_count = 0
    right_count = 0
    debug_count = 0
    other_count = 0
    other_samples = []
    left_in_other = []
    parse_errors = 0
    total_lines = 0
    first_left = None
    first_right = None

    start = time.time()
    while time.time() - start < args.duration:
        raw = proc.stdout.readline()
        if not raw:
            break
        total_lines += 1
        raw_str = raw.decode(errors="replace")
        dump_file.write(f"LINE {total_lines}: {repr(raw_str)}\n")
        line = raw_str.strip()

        if not line or not line.startswith("{"):
            other_count += 1
            if len(other_samples) < 10:
                preview = line[:150] if len(line) > 150 else line
                other_samples.append(f"  #{other_count}: [{len(line)}ch] {preview}")
            if "left" in line.lower():
                left_in_other.append(f"  #{other_count}: [{len(line)}ch] {line[:200]}")
            continue

        try:
            pkt = json.loads(line)
        except json.JSONDecodeError:
            parse_errors += 1
            continue

        pkt_type = pkt.get("type", "")
        if pkt_type == "debug":
            debug_count += 1
            continue
        if pkt_type != "manus":
            other_count += 1
            continue

        hand = pkt.get("hand", "?")
        if hand == "left":
            left_count += 1
            if first_left is None:
                first_left = pkt
        elif hand == "right":
            right_count += 1
            if first_right is None:
                first_right = pkt
        else:
            other_count += 1

    dump_file.close()

    # Kill process
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()

    elapsed = time.time() - start

    # ── Results ──────────────────────────────────────────────
    print(f"\n  Results ({elapsed:.1f}s):")
    print(f"    Total lines:  {total_lines}")
    print(f"    Left packets:  {left_count}")
    print(f"    Right packets: {right_count}")
    print(f"    Debug packets: {debug_count}")
    print(f"    Other/non-JSON: {other_count}")
    print(f"    Parse errors:  {parse_errors}")

    if other_samples:
        print(f"\n  Non-JSON line samples (first {len(other_samples)}):")
        for s in other_samples:
            print(s)

    if left_in_other:
        print(f"\n  *** Lines containing 'left' in non-JSON output:")
        for s in left_in_other:
            print(s)

    total_manus = left_count + right_count
    if total_manus > 0:
        left_pct = left_count / total_manus * 100
        right_pct = right_count / total_manus * 100
        print(f"    Left ratio:  {left_pct:.1f}%")
        print(f"    Right ratio: {right_pct:.1f}%")
        hz = total_manus / elapsed
        print(f"    Rate: {hz:.1f} packets/s")

    # ── Test 2: Validate JSON format ────────────────────────
    print(f"\n[TEST 2] JSON format validation...")
    required_keys = {"type", "hand", "joint_angles", "finger_spread",
                     "wrist_pos", "wrist_quat", "timestamp"}

    for label, pkt in [("left", first_left), ("right", first_right)]:
        if pkt is None:
            print(f"  {label}: [SKIP] No packet received")
            continue

        missing = required_keys - set(pkt.keys())
        if missing:
            print(f"  {label}: [FAIL] Missing keys: {missing}")
        else:
            print(f"  {label}: [PASS] All required keys present")

        angles = pkt.get("joint_angles", [])
        spread = pkt.get("finger_spread", [])
        print(f"    joint_angles: {len(angles)} values (expect 20)")
        print(f"    finger_spread: {len(spread)} values (expect 5)")
        print(f"    sample angles[0:4]: {angles[:4]}")

    # ── Summary ──────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    issues = []
    if left_count == 0:
        issues.append("No left hand packets received")
    if right_count == 0:
        issues.append("No right hand packets received")
    if total_manus > 0 and left_count / total_manus < 0.3:
        issues.append(f"Left hand underrepresented ({left_pct:.1f}%)")
    if parse_errors > 0:
        issues.append(f"{parse_errors} JSON parse errors")

    if not issues:
        print("  [ALL PASS] Stream data looks good!")
    else:
        print("  [ISSUES]")
        for issue in issues:
            print(f"    - {issue}")
    print(f"{'=' * 60}")

    sys.exit(1 if issues else 0)


if __name__ == "__main__":
    main()
