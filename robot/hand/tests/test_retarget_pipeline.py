#!/usr/bin/env python3
"""Retarget pipeline diagnostic: analyze each stage of Manus → DG5F.

Connects to Manus glove, records data, runs retarget, and reports:
- Spread values at each stage (Manus raw → retarget → clamp)
- Noise/jitter level per joint
- Value change rate (for vibration detection)
- Posture constraint clipping frequency

Usage:
    cd /workspaces/tamp_ws/src/teleop_dev
    python3 -m robot.hand.tests.test_retarget_pipeline --hand right --duration 5
    python3 -m robot.hand.tests.test_retarget_pipeline --hand right --duration 5 --pose fist
"""

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from sender.hand.manus_reader import (
    ManusReader, FINGER_NAMES, JOINT_NAMES_PER_FINGER,
    JOINTS_PER_FINGER, DEFAULT_SDK_BIN,
)
from robot.hand.retarget import ManusToD5FRetarget


FINGER_LABELS = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_LABELS = ["Spread", "MCP/Flex", "PIP/Flex", "DIP/Flex"]


def collect(reader, retarget, duration, hz=60):
    """Collect Manus raw + retarget output for `duration` seconds."""
    dt = 1.0 / hz
    raw_samples = []
    retarget_samples = []
    pre_clamp_samples = []

    start = time.time()
    while time.time() - start < duration:
        data = reader.get_hand_data()
        if data is not None:
            raw = data.joint_angles.copy()
            raw_samples.append(raw)

            # Run retarget (includes clamp)
            dg5f = retarget.retarget(raw)
            retarget_samples.append(dg5f.copy())

            # Also compute pre-clamp for comparison
            q_deg = np.degrees(raw.astype(np.float64))
            qd = np.zeros(20, dtype=np.float64)
            # Reproduce retarget math without clamp
            qd[0] = (58.5 - q_deg[1]) * math.pi / 180
            qd[1] = (q_deg[0] + 20.0) * math.pi / 180
            qd[2] = q_deg[2] * math.pi / 180
            qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * math.pi / 180
            qd[4] = q_deg[4] * math.pi / 180
            qd[5] = q_deg[5] * math.pi / 180
            qd[6] = (q_deg[6] - 40.0) * math.pi / 180
            qd[7] = 0.5 * (q_deg[6] + q_deg[7]) * math.pi / 180
            qd[8] = q_deg[8] * math.pi / 180
            qd[9] = q_deg[9] * math.pi / 180
            qd[10] = (q_deg[10] - 30.0) * math.pi / 180
            qd[11] = 0.5 * (q_deg[10] + q_deg[11]) * math.pi / 180
            for i in range(12, 16):
                qd[i] = q_deg[i] * math.pi / 180
            if q_deg[17] > 55 and q_deg[18] > 25 and q_deg[19] > 20:
                sm = 2.0
            else:
                sm = 1.0 / 1.5
            qd[16] = q_deg[16] * sm * math.pi / 180
            for i in range(17, 20):
                qd[i] = q_deg[i] * math.pi / 180
            pre_clamp_samples.append(qd.copy())

        time.sleep(dt)

    return (np.array(raw_samples) if raw_samples else None,
            np.array(retarget_samples) if retarget_samples else None,
            np.array(pre_clamp_samples) if pre_clamp_samples else None)


def print_stage_comparison(raw, retarget, pre_clamp, limits):
    """Print mean values at each pipeline stage for all 20 joints."""
    raw_mean = raw.mean(axis=0)
    ret_mean = retarget.mean(axis=0)
    pre_mean = pre_clamp.mean(axis=0)

    print(f"\n  {'Finger':7s} {'Joint':10s} "
          f"{'Manus(deg)':>11s} {'PreClamp':>10s} {'DG5F(deg)':>10s} "
          f"{'Limit Lo':>9s} {'Limit Hi':>9s} {'Clipped?':>9s}")
    print(f"  {'-' * 78}")

    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            m_deg = math.degrees(raw_mean[idx])
            p_deg = math.degrees(pre_mean[idx])
            d_deg = math.degrees(ret_mean[idx])
            lo = math.degrees(limits[idx].min_rad)
            hi = math.degrees(limits[idx].max_rad)
            clipped = "YES" if abs(d_deg - p_deg) > 0.5 else ""
            print(f"  {FINGER_LABELS[f]:7s} {JOINT_LABELS[j]:10s} "
                  f"{m_deg:+11.2f} {p_deg:+10.2f} {d_deg:+10.2f} "
                  f"{lo:+9.1f} {hi:+9.1f} {clipped:>9s}")


def print_spread_analysis(raw, retarget):
    """Detailed analysis of spread (abduction) joints only."""
    spread_indices = [0, 4, 8, 12, 16]
    raw_mean = raw.mean(axis=0)
    raw_std = raw.std(axis=0)
    raw_min = raw.min(axis=0)
    raw_max = raw.max(axis=0)
    ret_mean = retarget.mean(axis=0)

    print(f"\n  SPREAD ANALYSIS (abduction joints)")
    print(f"  {'Finger':7s} {'Manus(deg)':>11s} {'Range':>8s} {'StdDev':>8s} "
          f"{'DG5F(deg)':>10s} {'Moving?':>8s}")
    print(f"  {'-' * 60}")

    for i, idx in enumerate(spread_indices):
        m_mean = math.degrees(raw_mean[idx])
        m_range = math.degrees(raw_max[idx] - raw_min[idx])
        m_std = math.degrees(raw_std[idx])
        d_mean = math.degrees(ret_mean[idx])
        moving = "YES" if m_range > 2.0 else "NO"
        print(f"  {FINGER_LABELS[i]:7s} {m_mean:+11.2f} {m_range:8.2f} {m_std:8.3f} "
              f"{d_mean:+10.2f} {moving:>8s}")


def print_noise_analysis(raw, retarget):
    """Measure noise/jitter per joint."""
    # Compute frame-to-frame delta
    raw_diff = np.diff(raw, axis=0)
    ret_diff = np.diff(retarget, axis=0)

    raw_jitter = np.abs(raw_diff).mean(axis=0)
    ret_jitter = np.abs(ret_diff).mean(axis=0)

    # Find joints with highest jitter
    print(f"\n  NOISE/JITTER ANALYSIS (frame-to-frame |delta|)")
    print(f"  {'Finger':7s} {'Joint':10s} {'Manus(deg)':>11s} {'DG5F(deg)':>10s} "
          f"{'Amplified':>10s}")
    print(f"  {'-' * 55}")

    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            m_j = math.degrees(raw_jitter[idx])
            d_j = math.degrees(ret_jitter[idx])
            amp = d_j / m_j if m_j > 0.001 else 0
            flag = f"x{amp:.1f}" if amp > 2.0 else ""
            print(f"  {FINGER_LABELS[f]:7s} {JOINT_LABELS[j]:10s} "
                  f"{m_j:11.4f} {d_j:10.4f} {flag:>10s}")


def print_posture_constraint_stats(raw, retarget, hand_side):
    """Count how often posture constraints clip values."""
    is_right = (hand_side == "right")
    clip_counts = np.zeros(20, dtype=int)

    rt = ManusToD5FRetarget(hand_side=hand_side)
    for i in range(len(raw)):
        dg5f_noclip = rt.retarget(raw[i])
        # Compare with posture-constrained output
        # (retarget already includes constraints, so we check for zeros)
        for idx in ([0, 2, 3] + [4, 8, 12, 16]):
            if retarget[i, idx] == 0.0 and dg5f_noclip[idx] == 0.0:
                # Both zero from constraint — count it
                pass  # Can't distinguish easily, skip

    # Simpler: count how many frames have spread=0 (likely constrained)
    spread_indices = [4, 8, 12, 16]  # Index/Middle/Ring/Pinky
    print(f"\n  POSTURE CONSTRAINT STATS")
    print(f"  {'Finger':7s} {'Frames at 0':>12s} {'% at 0':>8s}")
    print(f"  {'-' * 30}")
    for i, idx in enumerate(spread_indices):
        at_zero = (np.abs(retarget[:, idx]) < 0.001).sum()
        pct = at_zero / len(retarget) * 100
        print(f"  {FINGER_LABELS[i+1]:7s} {at_zero:12d} {pct:7.1f}%")


def print_timing_recommendation(raw):
    """Suggest motion_time_ms based on data rate."""
    n = len(raw)
    print(f"\n  TIMING")
    print(f"    Samples collected: {n}")
    print(f"    Current motion_time_ms: 50 (config default)")
    print(f"    At 60Hz control, command interval = 16.7ms")
    print(f"    → motion_time_ms=50 means each command takes 3x the interval")
    print(f"    → This causes sluggish response (commands overlap)")
    print(f"    Recommendation: Try motion_time_ms=16 or 20")


def main():
    parser = argparse.ArgumentParser(description="Retarget pipeline diagnostic")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_BIN)
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--pose", default="mixed",
                        choices=["mixed", "open", "fist", "spread"],
                        help="What pose to hold during recording")
    args = parser.parse_args()

    print("=" * 70)
    print("  Retarget Pipeline Diagnostic")
    print(f"  Hand: {args.hand}  |  Duration: {args.duration}s  |  Pose: {args.pose}")
    print("=" * 70)

    reader = ManusReader(sdk_bin_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
    except Exception as e:
        print(f"  [ERROR] {e}")
        sys.exit(1)

    time.sleep(0.5)
    retarget = ManusToD5FRetarget(hand_side=args.hand)

    pose_instructions = {
        "mixed": "Move fingers freely (open/close/spread)",
        "open": "Keep hand fully open",
        "fist": "Keep a tight fist",
        "spread": "Spread all fingers wide apart",
    }
    print(f"\n  Instruction: {pose_instructions[args.pose]}")
    print(f"  Recording starts in 2 seconds...")
    time.sleep(2.0)
    print(f"  Recording {args.duration}s...")

    raw, ret, pre_clamp = collect(reader, retarget, args.duration)
    reader.disconnect()

    if raw is None or len(raw) == 0:
        print("  [ERROR] No data received")
        sys.exit(1)

    print(f"\n  Collected {len(raw)} frames")

    # ── Stage comparison ─────────────────────────────────
    limits = retarget.get_limits()
    print_stage_comparison(raw, ret, pre_clamp, limits)

    # ── Spread analysis ──────────────────────────────────
    print_spread_analysis(raw, ret)

    # ── Noise analysis ───────────────────────────────────
    if len(raw) > 2:
        print_noise_analysis(raw, ret)

    # ── Posture constraint stats ─────────────────────────
    print_posture_constraint_stats(raw, ret, args.hand)

    # ── Timing recommendation ────────────────────────────
    print_timing_recommendation(raw)

    # ── Summary ──────────────────────────────────────────
    print(f"\n{'=' * 70}")
    print(f"  DIAGNOSIS SUMMARY")
    print(f"{'=' * 70}")

    # Check spread
    spread_indices = [0, 4, 8, 12, 16]
    raw_spread_range = []
    ret_spread_range = []
    for idx in spread_indices:
        raw_spread_range.append(raw[:, idx].max() - raw[:, idx].min())
        ret_spread_range.append(ret[:, idx].max() - ret[:, idx].min())

    print(f"\n  [1] SPREAD:")
    for i, idx in enumerate(spread_indices):
        r_deg = math.degrees(raw_spread_range[i])
        d_deg = math.degrees(ret_spread_range[i])
        status = "OK" if d_deg > 2.0 else "NO MOTION"
        print(f"    {FINGER_LABELS[i]}: Manus range={r_deg:.1f}° → DG5F range={d_deg:.1f}° [{status}]")

    # Check jitter
    if len(raw) > 2:
        ret_diff = np.abs(np.diff(ret, axis=0))
        max_jitter = math.degrees(ret_diff.mean(axis=0).max())
        jitter_status = "HIGH" if max_jitter > 1.0 else "OK"
        print(f"\n  [2] JITTER: max avg frame-to-frame change = {max_jitter:.2f}° [{jitter_status}]")
        if jitter_status == "HIGH":
            print(f"    → Add EMA filter (alpha=0.3) before retarget")

    # Check clipping
    print(f"\n  [3] CLIPPING:")
    for f in range(5):
        for j in range(4):
            idx = f * 4 + j
            pre_mean = pre_clamp[:, idx].mean()
            ret_mean_v = ret[:, idx].mean()
            if abs(math.degrees(pre_mean - ret_mean_v)) > 3.0:
                print(f"    {FINGER_LABELS[f]} {JOINT_LABELS[j]}: "
                      f"pre-clamp={math.degrees(pre_mean):.1f}° → "
                      f"clamped={math.degrees(ret_mean_v):.1f}° (SIGNIFICANT)")

    print(f"\n  [4] TIMING: motion_time_ms=50 → likely causes sluggish response")
    print(f"    → Try reducing to 16-20ms")

    print(f"\n{'=' * 70}")


if __name__ == "__main__":
    main()
