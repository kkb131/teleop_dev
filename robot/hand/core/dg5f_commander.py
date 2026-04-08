"""DG5F 명령 래퍼.

dg5f_ros2_client.py를 감싸서 safety clamp + logging 추가.
모든 세대의 robot/hand 코드에서 공통 사용.
"""

import numpy as np
from rclpy.node import Node

from robot.hand.dg5f_ros2_client import DG5FROS2Client
from sender.hand.core.dg5f_config import get_limits_arrays, NUM_JOINTS


class DG5FCommander:
    """DG5F 명령 전송 래퍼 with safety clamp.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    motion_time_ms : int
        모션 시간 (ms)
    """

    def __init__(self, hand_side: str = "right", motion_time_ms: int = 50):
        self._hand_side = hand_side
        self._lower, self._upper = get_limits_arrays(hand_side)

        self._client = DG5FROS2Client(hand_side=hand_side,
                                       motion_time_ms=motion_time_ms)

    @property
    def node(self) -> Node:
        """rclpy spin에 사용할 ROS2 노드."""
        return self._client

    def send(self, angles_rad: np.ndarray):
        """Safety clamp 후 DG5F에 명령 전송."""
        assert angles_rad.shape == (NUM_JOINTS,)
        safe = np.clip(angles_rad, self._lower, self._upper)
        self._client.set_positions(safe)

    def get_feedback(self) -> np.ndarray:
        """현재 관절 위치 피드백."""
        return self._client.get_positions()

    @property
    def has_feedback(self) -> bool:
        return self._client.has_feedback
