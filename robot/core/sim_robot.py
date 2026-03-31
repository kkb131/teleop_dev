"""Isaac Sim ROS2 topic backend — SimBackend."""

import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot.config import JOINT_NAMES
from robot.core.robot_backend import RobotBackend


class _SimNode(Node):
    """Internal ROS2 node for Isaac Sim communication."""

    def __init__(self, joint_states_topic: str, joint_command_topic: str):
        super().__init__("standalone_sim_node")
        self._latest_msg: Optional[JointState] = None
        self._event = threading.Event()

        self._sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_cb, 10
        )
        self._pub = self.create_publisher(JointState, joint_command_topic, 10)

    def _joint_state_cb(self, msg: JointState):
        self._latest_msg = msg
        self._event.set()

    @property
    def latest(self) -> Optional[JointState]:
        return self._latest_msg

    def wait_for_first_msg(self, timeout: float = 10.0) -> bool:
        return self._event.wait(timeout)

    def publish_command(self, positions: List[float], joint_names: List[str]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [float(p) for p in positions]
        self._pub.publish(msg)


class SimBackend(RobotBackend):
    """Isaac Sim robot backend using ROS2 topics."""

    def __init__(
        self,
        joint_states_topic: str = "/joint_states",
        joint_command_topic: str = "/joint_command",
        joint_names: Optional[List[str]] = None,
    ):
        self._js_topic = joint_states_topic
        self._cmd_topic = joint_command_topic
        self._joint_names = joint_names or JOINT_NAMES
        self._node: Optional[_SimNode] = None
        self._spin_thread: Optional[threading.Thread] = None

    def connect(self):
        if not rclpy.ok():
            rclpy.init()
        self._node = _SimNode(self._js_topic, self._cmd_topic)

        # Spin in background thread
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()

        print(f"[SimBackend] Waiting for {self._js_topic}...")
        if not self._node.wait_for_first_msg(timeout=10.0):
            raise TimeoutError(
                f"No message received on {self._js_topic} within 10s. "
                "Is Isaac Sim running and publishing?"
            )
        print(f"[SimBackend] Connected (topics: {self._js_topic}, {self._cmd_topic})")

    def disconnect(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.try_shutdown()
        print("[SimBackend] Disconnected")

    def get_joint_positions(self) -> List[float]:
        msg = self._node.latest
        if msg is None:
            return [0.0] * len(self._joint_names)
        # Reorder to match our expected joint order
        return self._reorder(msg, "position")

    def get_joint_velocities(self) -> List[float]:
        msg = self._node.latest
        if msg is None or len(msg.velocity) == 0:
            return [0.0] * len(self._joint_names)
        return self._reorder(msg, "velocity")

    def send_joint_command(self, positions: List[float]):
        self._node.publish_command(positions, self._joint_names)

    def _reorder(self, msg: JointState, field: str) -> List[float]:
        """Reorder message fields to match JOINT_NAMES order."""
        values = getattr(msg, field)
        if len(msg.name) == 0 or len(values) == 0:
            return list(values) + [0.0] * (len(self._joint_names) - len(values))

        # Build name→value map
        name_map = {n: v for n, v in zip(msg.name, values)}
        return [name_map.get(n, 0.0) for n in self._joint_names]
