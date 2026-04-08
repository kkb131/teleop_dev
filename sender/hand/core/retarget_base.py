"""세대 공통 리타게팅 베이스 클래스.

1A, 1B, 2A 등 모든 세대의 retarget 클래스가 이 ABC를 상속.
기존 RetargetBase(skeleton 전용)와 달리, 입력 타입이 자유로움.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np


class HandRetargetBase(ABC):
    """인간 손 데이터 → DG5F 20-joint 각도 변환 ABC.

    Attributes
    ----------
    hand_side : str
        "left" or "right"
    """

    def __init__(self, hand_side: str = "right"):
        self._hand_side = hand_side

    @property
    def hand_side(self) -> str:
        return self._hand_side

    @abstractmethod
    def retarget(self, **kwargs) -> np.ndarray:
        """입력 데이터 → DG5F joint angles (20,) radians.

        서브클래스마다 kwargs가 다름:
        - 1A: retarget(ergonomics=ndarray[20])
        - 1B: retarget(skeleton=ndarray[25,7])
        - 2A: retarget(skeleton=ndarray[25,7], ergonomics=ndarray[20])
        - 1C: retarget(landmarks=ndarray[21,3])
        """
        ...

    @abstractmethod
    def get_method_name(self) -> str:
        """리타게팅 방법 이름 (로그용). e.g., '1A-ergo-direct'"""
        ...

    def get_debug_info(self) -> Optional[dict]:
        """선택적 디버그 정보."""
        return None
