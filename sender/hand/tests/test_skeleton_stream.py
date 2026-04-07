#!/usr/bin/env python3
"""Diagnose skeleton data from SDKClient_Linux.out --stream-json.

Runs the binary, collects 100 frames, and reports:
- has_skeleton true/false ratio per hand
- skeleton node count when present
- saves first 10 raw lines to skeleton_dump.txt

Usage:
    python3 -m sender.hand.tests.test_skeleton_stream
"""

import json
import os
import re
import subprocess
import sys
import time

ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
SDK_BIN = "sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out"
SDK_DIR = "sender/hand/sdk/SDKClient_Linux"
LIB_DIR = os.path.join(SDK_DIR, "ManusSDK", "lib")
MAX_FRAMES = 200
TIMEOUT_S = 30


def main():
    bin_path = os.path.abspath(SDK_BIN)
    if not os.path.isfile(bin_path):
        print(f"[ERROR] Binary not found: {bin_path}")
        sys.exit(1)

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = os.path.abspath(LIB_DIR) + ":" + env.get("LD_LIBRARY_PATH", "")

    print(f"[TEST] Launching {bin_path} --stream-json")
    print(f"[TEST] Collecting up to {MAX_FRAMES} frames (timeout {TIMEOUT_S}s)")
    print()

    proc = subprocess.Popen(
        [bin_path, "--stream-json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=os.path.abspath(SDK_DIR),
        env=env,
    )

    stats = {
        "total": 0,
        "left_skel_true": 0, "left_skel_false": 0,
        "right_skel_true": 0, "right_skel_false": 0,
        "left_nodes": [], "right_nodes": [],
        "parse_errors": 0,
        "non_json": 0,
    }
    raw_lines = []
    t_start = time.time()

    try:
        while stats["total"] < MAX_FRAMES and (time.time() - t_start) < TIMEOUT_S:
            raw = proc.stdout.readline()
            if not raw:
                break
            line = ANSI_RE.sub('', raw.decode('utf-8', errors='replace')).strip()

            # Save first 20 raw lines
            if len(raw_lines) < 20:
                raw_lines.append(repr(raw.decode('utf-8', errors='replace').rstrip()))

            if not line or not line.startswith("{"):
                stats["non_json"] += 1
                continue

            try:
                pkt = json.loads(line)
            except json.JSONDecodeError:
                stats["parse_errors"] += 1
                continue

            if pkt.get("type") != "manus":
                continue

            stats["total"] += 1
            hand = pkt.get("hand", "?")
            has_skel = pkt.get("has_skeleton", False)
            skel = pkt.get("skeleton")
            node_count = len(skel) if isinstance(skel, list) else 0

            if hand == "left":
                if has_skel:
                    stats["left_skel_true"] += 1
                    stats["left_nodes"].append(node_count)
                else:
                    stats["left_skel_false"] += 1
            elif hand == "right":
                if has_skel:
                    stats["right_skel_true"] += 1
                    stats["right_nodes"].append(node_count)
                else:
                    stats["right_skel_false"] += 1

            # Progress
            if stats["total"] % 50 == 0:
                print(f"  ... {stats['total']} frames collected")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

    # Save raw dump
    dump_file = "skeleton_dump.txt"
    with open(dump_file, "w") as f:
        for i, line in enumerate(raw_lines):
            f.write(f"[{i:3d}] {line}\n")
    print(f"\n[DUMP] First {len(raw_lines)} raw lines saved to {dump_file}")

    # Print stderr
    stderr = proc.stderr.read().decode('utf-8', errors='replace')
    if stderr.strip():
        print(f"\n[STDERR] (first 500 chars):")
        print(stderr[:500])

    # Results
    print("\n" + "=" * 60)
    print("  SKELETON STREAM DIAGNOSTIC")
    print("=" * 60)
    print(f"  Total manus frames: {stats['total']}")
    print(f"  Non-JSON lines:     {stats['non_json']}")
    print(f"  Parse errors:       {stats['parse_errors']}")
    print()

    lt = stats["left_skel_true"]
    lf = stats["left_skel_false"]
    rt = stats["right_skel_true"]
    rf = stats["right_skel_false"]

    print(f"  LEFT  hand: has_skeleton=true {lt:4d}, false {lf:4d}  "
          f"({100*lt/(lt+lf):.1f}% true)" if (lt + lf) > 0 else "  LEFT  hand: no data")
    print(f"  RIGHT hand: has_skeleton=true {rt:4d}, false {rf:4d}  "
          f"({100*rt/(rt+rf):.1f}% true)" if (rt + rf) > 0 else "  RIGHT hand: no data")

    if stats["left_nodes"]:
        print(f"  LEFT  skeleton nodes: min={min(stats['left_nodes'])}, max={max(stats['left_nodes'])}, avg={sum(stats['left_nodes'])/len(stats['left_nodes']):.0f}")
    if stats["right_nodes"]:
        print(f"  RIGHT skeleton nodes: min={min(stats['right_nodes'])}, max={max(stats['right_nodes'])}, avg={sum(stats['right_nodes'])/len(stats['right_nodes']):.0f}")

    print()
    if lt + rt == 0:
        print("  [FAIL] No skeleton data at all!")
        print("  Possible causes:")
        print("    1. C++ binary not rebuilt after adding LoadTestSkeleton()")
        print("    2. LoadTestSkeleton() failed (check stderr)")
        print("    3. m_RawSkeleton callback never fires")
    elif (lt + rt) < (lf + rf):
        print(f"  [WARN] Skeleton present only {100*(lt+rt)/(lt+rt+lf+rf):.0f}% of frames")
        print("  Skeleton callback may fire less frequently than ergonomics")
    else:
        print("  [OK] Skeleton data present in majority of frames")
    print("=" * 60)


if __name__ == "__main__":
    main()
