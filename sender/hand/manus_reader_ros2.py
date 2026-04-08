#!/usr/bin/env python3
"""Manus Quantum Metagloves reader — ROS2 subscriber mode.

Subscribes to manus_ros2 topics (manus_glove_0, manus_glove_1) published by
the official manus_data_publisher node. Provides the same HandData interface
as the subprocess-based ManusReader.

Requires:
    - manus_ros2 + manus_ros2_msgs packages built and sourced
    - manus_data_publisher running: ros2 run manus_ros2 manus_data_publisher

Usage:
    # As library (from manus_sender.py):
    reader = ManusReaderROS2(hand_side="right")
    reader.connect()
    data = reader.get_hand_data()

    # Standalone test:
    python3 -m sender.hand.manus_reader_ros2 --hand right
"""

import threading
import time
from typing import Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from manus_ros2_msgs.msg import ManusGlove
except ImportError as e:
    raise ImportError(
        f"ROS2 packages not found: {e}\n"
        "Source your manus_ws: source ~/manus_ws/install/setup.bash"
    )

from sender.hand.manus_reader import (
    HandData, NUM_JOINTS, NUM_FINGERS, JOINTS_PER_FINGER,
    FINGER_NAMES, JOINT_NAMES_PER_FINGER,
)

# Ergonomics type string → joint index mapping
# SDK publishes ergonomics with type strings (e.g., "ThumbCMCSpread", "IndexMCPFlexion")
# We map these to our 20-joint array: [Thumb(spread,MCP,PIP,DIP), Index(...), ...]
# Actual type strings from manus_ros2 (confirmed by user):
# ThumbMCPSpread, ThumbMCPStretch, ThumbPIPStretch, ThumbDIPStretch,
# IndexSpread, IndexMCPStretch, IndexPIPStretch, IndexDIPStretch, ...
_ERGO_TYPE_MAP = {
    # Thumb (0-3)
    "ThumbMCPSpread": 0, "ThumbMCPStretch": 1,
    "ThumbPIPStretch": 2, "ThumbDIPStretch": 3,
    # Index (4-7)
    "IndexSpread": 4, "IndexMCPStretch": 5,
    "IndexPIPStretch": 6, "IndexDIPStretch": 7,
    # Middle (8-11)
    "MiddleSpread": 8, "MiddleMCPStretch": 9,
    "MiddlePIPStretch": 10, "MiddleDIPStretch": 11,
    # Ring (12-15)
    "RingSpread": 12, "RingMCPStretch": 13,
    "RingPIPStretch": 14, "RingDIPStretch": 15,
    # Pinky (16-19)
    "PinkySpread": 16, "PinkyMCPStretch": 17,
    "PinkyPIPStretch": 18, "PinkyDIPStretch": 19,
}


class ManusReaderROS2:
    """Reads hand data from Manus gloves via ROS2 topics.

    Same interface as ManusReader (subprocess mode) — drop-in replacement.
    """

    def __init__(self, hand_side: str = "right", verbose: bool = False):
        self._hand_side = hand_side
        self._verbose = verbose
        self._connected = False

        # Data cache (thread-safe)
        self._lock = threading.Lock()
        self._left_data: Optional[HandData] = None
        self._right_data: Optional[HandData] = None
        self._data_received = threading.Event()
        self._line_count = 0
        self._left_count = 0
        self._right_count = 0
        self._error_count = 0

        self._node: Optional[Node] = None
        self._spin_thread: Optional[threading.Thread] = None

    def connect(self):
        """Initialize ROS2 node and subscribe to manus_glove topics."""
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('manus_reader_ros2')

        # Subscribe to multiple glove topics (typically manus_glove_0, manus_glove_1)
        for i in range(4):  # up to 4 gloves
            topic = f'/manus_glove_{i}'
            self._node.create_subscription(
                ManusGlove, topic, self._glove_callback, 10
            )
            if self._verbose:
                print(f"[ManusROS2] Subscribed to {topic}")

        self._connected = True

        # Spin in background thread
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        print(f"[ManusROS2] Waiting for data on /manus_glove_* topics...")
        if self._data_received.wait(timeout=10.0):
            print(f"[ManusROS2] OK — receiving data")
        else:
            print(f"[ManusROS2] TIMEOUT — no data received. Is manus_data_publisher running?")

    def disconnect(self):
        """Shutdown ROS2 node."""
        self._connected = False
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        print("[ManusROS2] Disconnected")

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        if not self._connected:
            return None
        target_side = side or self._hand_side
        with self._lock:
            return self._left_data if target_side == "left" else self._right_data

    def get_both_hands(self) -> dict[str, Optional[HandData]]:
        return {"left": self.get_hand_data("left"),
                "right": self.get_hand_data("right")}

    def get_status(self) -> dict:
        return {
            "sdk_loaded": True,
            "connected": self._connected,
            "hand_side": self._hand_side,
            "bin_path": "ros2",
            "lines_received": self._line_count,
            "left_count": self._left_count,
            "right_count": self._right_count,
            "errors": self._error_count,
            "proc_alive": self._connected,
        }

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        return self._data_received.wait(timeout=timeout)

    def get_debug_info(self) -> Optional[dict]:
        return None  # No C++ debug in ROS2 mode

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── Private ──────────────────────────────────────

    def _spin_loop(self):
        """Spin ROS2 node in background thread."""
        try:
            while self._connected and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception:
            pass

    def _glove_callback(self, msg: ManusGlove):
        """Convert ManusGlove message to HandData."""
        try:
            hand_side = msg.side.lower() if msg.side else "right"

            # Skip if we only want one hand
            if self._hand_side != "both" and hand_side != self._hand_side:
                return

            # Ergonomics → joint_angles (degrees → radians) using type string mapping
            joint_angles = np.zeros(NUM_JOINTS, dtype=np.float32)
            unmapped_types = []
            for ergo in msg.ergonomics:
                idx = _ERGO_TYPE_MAP.get(ergo.type)
                if idx is not None:
                    joint_angles[idx] = np.deg2rad(ergo.value)
                else:
                    unmapped_types.append(ergo.type)

            # Log unmapped types once
            if unmapped_types and self._line_count == 0:
                print(f"[ManusROS2] Unmapped ergo types: {unmapped_types}")
                print(f"[ManusROS2] All ergo types received: "
                      f"{[e.type for e in msg.ergonomics]}")

            # Finger spread (indices 0, 4, 8, 12, 16)
            finger_spread = np.array([
                joint_angles[0], joint_angles[4], joint_angles[8],
                joint_angles[12], joint_angles[16]
            ], dtype=np.float32)

            # Raw skeleton → ndarray[N, 7] (x, y, z, qw, qx, qy, qz)
            # IMPORTANT: Use node_id as index, NOT insertion order
            # Manus may send nodes in arbitrary order
            skeleton = None
            has_skeleton = False
            if msg.raw_nodes and len(msg.raw_nodes) > 0:
                max_id = max(node.node_id for node in msg.raw_nodes)
                skel_arr = np.zeros((max_id + 1, 7), dtype=np.float32)
                for node in msg.raw_nodes:
                    p = node.pose.position
                    q = node.pose.orientation
                    skel_arr[node.node_id] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
                skeleton = skel_arr
                has_skeleton = True

                # Log skeleton info once
                if self._line_count == 0:
                    print(f"[ManusROS2] Skeleton: {len(msg.raw_nodes)} nodes, "
                          f"max_id={max_id}, array shape={skeleton.shape}")
                    # Print node_id → joint_type mapping for first 5 nodes
                    for node in sorted(msg.raw_nodes, key=lambda n: n.node_id)[:10]:
                        print(f"  node[{node.node_id:2d}]: joint={node.joint_type}, "
                              f"chain={node.chain_type}")

            # Wrist from first skeleton node (if available)
            wrist_pos = np.zeros(3, dtype=np.float32)
            wrist_quat = np.array([1.0, 0, 0, 0], dtype=np.float32)
            if skeleton is not None and len(skeleton) > 0:
                wrist_pos = skeleton[0, :3].copy()
                wrist_quat = skeleton[0, 3:].copy()

            hd = HandData(
                joint_angles=joint_angles,
                finger_spread=finger_spread,
                wrist_pos=wrist_pos,
                wrist_quat=wrist_quat,
                hand_side=hand_side,
                timestamp=time.time(),
                skeleton=skeleton,
                has_skeleton=has_skeleton,
            )

            with self._lock:
                if hand_side == "left":
                    self._left_data = hd
                    self._left_count += 1
                else:
                    self._right_data = hd
                    self._right_count += 1

            self._line_count += 1
            self._data_received.set()

        except Exception as e:
            self._error_count += 1
            if self._verbose:
                print(f"[ManusROS2] Callback error: {e}")


# ─────────────────────────────────────────────────────────
# Standalone test
# ─────────────────────────────────────────────────────────

def main():
    import argparse
    from sender.hand.manus_reader import _print_hand_data

    parser = argparse.ArgumentParser(
        description="Manus Quantum Metagloves reader (ROS2 mode)")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"])
    parser.add_argument("--hz", type=int, default=30)
    parser.add_argument("--duration", type=float, default=0)
    args = parser.parse_args()

    print("=" * 65)
    print("  Manus Quantum Metagloves — Reader (ROS2 mode)")
    print("=" * 65)

    with ManusReaderROS2(hand_side=args.hand, verbose=True) as reader:
        status = reader.get_status()
        print(f"  Connected: {status['connected']}")
        print("-" * 65)
        print("  Press Ctrl+C to stop.\n")

        dt = 1.0 / args.hz
        count = 0
        start_time = time.time()
        num_lines = 6  # 1 header + 5 fingers

        try:
            while True:
                t_loop = time.perf_counter()

                data = reader.get_hand_data()
                if data is not None:
                    if count > 0:
                        print(f"\033[{num_lines}A", end="")
                    print(_print_hand_data(data))
                    skel_str = f"skeleton={data.skeleton.shape}" if data.has_skeleton else "no skeleton"
                    print(f"  [{skel_str}]")
                    count += 1

                if args.duration > 0 and time.time() - start_time >= args.duration:
                    break

                remaining = dt - (time.perf_counter() - t_loop)
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            pass

        elapsed = time.time() - start_time
        s = reader.get_status()
        print(f"\n{'=' * 65}")
        print(f"  Frames: {count}, Duration: {elapsed:.1f}s")
        print(f"  L={s['left_count']} R={s['right_count']} err={s['errors']}")
        print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
