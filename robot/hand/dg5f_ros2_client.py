#!/usr/bin/env python3
"""ROS2 topic-based client for Tesollo DG5F hand.

Replaces the Modbus-based DG5FClient with ROS2 JointTrajectory publishing.
Requires dg5f_driver running via ros2 launch.

Usage:
    from teleop_dev.robot.hand.dg5f_ros2_client import DG5FROS2Client

    rclpy.init()
    client = DG5FROS2Client(hand_side="right")
    client.set_positions([0.0] * 20)
    client.destroy_node()
    rclpy.shutdown()
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState

NUM_MOTORS = 20

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]


class DG5FROS2Client(Node):
    """ROS2-based DG5F hand controller.

    Compatible interface with DG5FClient (Modbus version).

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    motion_time_ms : int
        Default trajectory duration in milliseconds.
    """

    def __init__(self, hand_side: str = "right", motion_time_ms: int = 50):
        super().__init__("dg5f_ros2_client")
        self._hand_side = hand_side
        self._motion_time_s = motion_time_ms / 1000.0

        prefix = f"dg5f_{hand_side}"
        side_prefix = "rj" if hand_side == "right" else "lj"
        self._joint_names = RIGHT_JOINT_NAMES if hand_side == "right" else LEFT_JOINT_NAMES

        # Publisher for PidController (MultiDOFCommand)
        pid_topic = f"/{prefix}/{side_prefix}_dg_pospid/reference"
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(MultiDOFCommand, pid_topic, qos)
        self.get_logger().info(f"[DG5F-ROS2] Publishing to: {pid_topic}")

        # Subscriber for feedback
        js_topic = f"/{prefix}/joint_states"
        self._lock = threading.Lock()
        self._latest_positions = np.zeros(NUM_MOTORS)
        self._latest_velocities = np.zeros(NUM_MOTORS)
        self._has_feedback = False
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        self.get_logger().info(f"[DG5F-ROS2] {hand_side} hand feedback: {js_topic}")

    def _js_cb(self, msg: JointState):
        with self._lock:
            if len(msg.position) >= NUM_MOTORS:
                self._latest_positions[:] = msg.position[:NUM_MOTORS]
            if len(msg.velocity) >= NUM_MOTORS:
                self._latest_velocities[:] = msg.velocity[:NUM_MOTORS]
            self._has_feedback = True

    @property
    def connected(self) -> bool:
        return True  # Always "connected" if node is alive

    @property
    def started(self) -> bool:
        return True  # Driver manages motor enable

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def start(self):
        """No-op — driver manages motor enable."""
        self.get_logger().info("[DG5F-ROS2] start() — managed by dg5f_driver")

    def stop(self):
        """No-op — driver manages motor disable."""
        self.get_logger().info("[DG5F-ROS2] stop() — managed by dg5f_driver")

    def set_positions(self, angles_rad: list | np.ndarray):
        """Publish target positions as MultiDOFCommand to PidController."""
        if len(angles_rad) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} angles, got {len(angles_rad)}")

        msg = MultiDOFCommand()
        msg.dof_names = list(self._joint_names)
        msg.values = [float(a) for a in angles_rad]

        self._pub.publish(msg)

    def set_motion_times(self, times_ms: list | np.ndarray):
        """Update default motion time (uses first value)."""
        if len(times_ms) > 0:
            self._motion_time_s = max(1, int(times_ms[0])) / 1000.0

    def get_positions(self) -> np.ndarray:
        """Return latest joint positions from /joint_states."""
        with self._lock:
            return self._latest_positions.copy()

    def get_velocities(self) -> np.ndarray:
        """Return latest joint velocities from /joint_states."""
        with self._lock:
            return self._latest_velocities.copy()

    def is_moving(self) -> bool:
        """Check if any joint has non-zero velocity."""
        with self._lock:
            return bool(np.any(np.abs(self._latest_velocities) > 0.01))

    @property
    def has_feedback(self) -> bool:
        """True if at least one joint_states message was received."""
        with self._lock:
            return self._has_feedback
