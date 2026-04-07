#!/usr/bin/env python3
"""Pose debug: analyze Manus data at each pipeline stage.

Records a pose for 3 seconds, then prints:
1. Raw ergonomics (type + degrees)
2. Skeleton fingertip vectors
3. Vector retarget output (DG5F angles)
4. DG5F FK fingertip vectors
5. Direction error (human vs robot)

Requires: manus_data_publisher running, manus_ros2_msgs sourced.

Usage:
    python3 -m sender.hand.tests.test_pose_debug --hand right --pose spread
    python3 -m sender.hand.tests.test_pose_debug --hand right --pose fist
"""

import argparse
import time
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Manus pose pipeline debug")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--pose", default="spread",
                        choices=["spread", "fist", "relax"],
                        help="Which pose to hold during recording")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    print("=" * 70)
    print(f"  POSE PIPELINE DEBUG — {args.pose.upper()} — {args.hand.upper()} hand")
    print("=" * 70)

    # Import ROS2 reader
    from sender.hand.manus_reader_ros2 import ManusReaderROS2

    reader = ManusReaderROS2(hand_side=args.hand, verbose=True)
    reader.connect()

    print(f"\n  Hold {args.pose.upper()} pose for {args.duration}s...")
    print(f"  Recording starts in 2 seconds...\n")
    time.sleep(2.0)

    # Collect frames
    frames = []
    t_start = time.time()
    while time.time() - t_start < args.duration:
        import rclpy
        rclpy.spin_once(reader._node, timeout_sec=0.01)
        data = reader.get_hand_data()
        if data is not None:
            frames.append(data)
        time.sleep(0.02)

    reader.disconnect()

    if not frames:
        print("  [ERROR] No data received!")
        return

    # Use last frame for analysis
    data = frames[-1]
    print(f"\n  Collected {len(frames)} frames. Analyzing last frame...\n")

    # ─── Stage 1: Raw Ergonomics ───
    print("─" * 70)
    print("  [1] RAW ERGONOMICS (from Manus ROS2)")
    print("─" * 70)
    joint_names = [
        "Thumb Spread", "Thumb MCP", "Thumb PIP", "Thumb DIP",
        "Index Spread", "Index MCP", "Index PIP", "Index DIP",
        "Middle Spread", "Middle MCP", "Middle PIP", "Middle DIP",
        "Ring Spread", "Ring MCP", "Ring PIP", "Ring DIP",
        "Pinky Spread", "Pinky MCP", "Pinky PIP", "Pinky DIP",
    ]
    print(f"  {'Joint':<16s} {'rad':>8s} {'deg':>8s}")
    for i, name in enumerate(joint_names):
        rad = data.joint_angles[i]
        deg = np.degrees(rad)
        marker = " <<<" if abs(deg) > 15 else ""
        print(f"  {name:<16s} {rad:+8.3f} {deg:+8.1f}°{marker}")

    # ─── Stage 2: Skeleton ───
    print(f"\n{'─' * 70}")
    print("  [2] SKELETON FINGERTIP VECTORS")
    print("─" * 70)

    if data.skeleton is not None and data.has_skeleton:
        from sender.hand.vector_retarget import skeleton_to_fingertip_vectors, FINGER_NAMES
        human_vecs = skeleton_to_fingertip_vectors(data.skeleton)
        print(f"  Skeleton nodes: {data.skeleton.shape[0]}")
        print(f"  Wrist pos: [{data.skeleton[0,0]:.4f}, {data.skeleton[0,1]:.4f}, {data.skeleton[0,2]:.4f}]")
        print(f"\n  {'Finger':<8s} {'X':>8s} {'Y':>8s} {'Z':>8s}  {'|v|':>6s}")
        for i, name in enumerate(FINGER_NAMES):
            v = human_vecs[i]
            norm = np.linalg.norm(v)
            print(f"  {name:<8s} {v[0]:+8.4f} {v[1]:+8.4f} {v[2]:+8.4f}  {norm:6.4f}")
    else:
        print("  [WARN] No skeleton data!")
        human_vecs = None

    # ─── Stage 3: Vector Retarget ───
    print(f"\n{'─' * 70}")
    print("  [3] VECTOR RETARGET OUTPUT (DG5F angles)")
    print("─" * 70)

    if data.skeleton is not None and data.has_skeleton:
        try:
            from sender.hand.vector_retarget import VectorRetarget
            rt = VectorRetarget(hand_side=args.hand)
            dg5f_q = rt.retarget(data.skeleton)

            print(f"  {'Joint':<16s} {'rad':>8s} {'deg':>8s}")
            for i, name in enumerate(joint_names):
                rad = dg5f_q[i]
                deg = np.degrees(rad)
                marker = " <<<" if abs(deg) > 15 else ""
                print(f"  {name:<16s} {rad:+8.3f} {deg:+8.1f}°{marker}")

            # ─── Stage 4: DG5F FK ───
            print(f"\n{'─' * 70}")
            print("  [4] DG5F FK FINGERTIP VECTORS (after retarget)")
            print("─" * 70)

            robot_vecs = rt._robot_fk.fingertip_vectors(dg5f_q)
            print(f"  {'Finger':<8s} {'X':>8s} {'Y':>8s} {'Z':>8s}  {'|v|':>6s}")
            for i, name in enumerate(FINGER_NAMES):
                v = robot_vecs[i]
                norm = np.linalg.norm(v)
                print(f"  {name:<8s} {v[0]:+8.4f} {v[1]:+8.4f} {v[2]:+8.4f}  {norm:6.4f}")

            # ─── Stage 5: Error ───
            print(f"\n{'─' * 70}")
            print("  [5] DIRECTION ERROR (human vs robot)")
            print("─" * 70)

            debug = rt.get_debug_info(data.skeleton, dg5f_q)
            print(f"  {'Finger':<8s} {'Error(°)':>10s}")
            for i, name in enumerate(FINGER_NAMES):
                err = debug["angle_errors_deg"][i]
                status = "OK" if err < 10 else "WARN" if err < 30 else "BAD"
                print(f"  {name:<8s} {err:+10.1f}°  [{status}]")
            print(f"\n  Mean error: {debug['mean_error_deg']:.1f}°")

        except ImportError as e:
            print(f"  [SKIP] Vector retarget not available: {e}")
    else:
        print("  [SKIP] No skeleton — cannot run vector retarget")

    # ─── Summary ───
    print(f"\n{'=' * 70}")
    print(f"  SUMMARY — {args.pose.upper()} pose")
    print("=" * 70)

    # Check ergonomics match expectations
    spread_indices = [0, 4, 8, 12, 16]
    stretch_indices = [i for i in range(20) if i not in spread_indices]

    avg_spread = np.mean(np.abs(np.degrees(data.joint_angles[spread_indices])))
    avg_stretch = np.mean(np.abs(np.degrees(data.joint_angles[stretch_indices])))

    print(f"  Avg |Spread| = {avg_spread:.1f}°")
    print(f"  Avg |Stretch(MCP/PIP/DIP)| = {avg_stretch:.1f}°")

    if args.pose == "spread":
        if avg_spread < 5:
            print("  [WARN] Spread values very small — glove may not detect abduction well")
        if avg_stretch > 20:
            print("  [WARN] Stretch values large during spread — unexpected")
    elif args.pose == "fist":
        if avg_stretch < 20:
            print("  [WARN] Stretch values small during fist — expected 40-90°")
        if avg_spread > 15:
            print("  [WARN] Spread values large during fist — unexpected")

    print("=" * 70)


if __name__ == "__main__":
    main()
