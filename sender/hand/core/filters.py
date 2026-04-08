"""시간축 필터. 모든 세대에서 공통 사용."""

import numpy as np


class EMAFilter:
    """Exponential Moving Average filter for joint angles.

    Parameters
    ----------
    alpha : float
        0~1. 1에 가까울수록 새 값 반영 (반응 빠름, 떨림 많음).
    size : int
        필터 대상 벡터 크기 (DG5F: 20).
    """

    def __init__(self, alpha: float = 0.4, size: int = 20):
        self._alpha = alpha
        self._prev = None
        self._size = size

    def filter(self, value: np.ndarray) -> np.ndarray:
        if self._prev is None:
            self._prev = value.copy()
            return value.copy()
        filtered = self._alpha * value + (1.0 - self._alpha) * self._prev
        self._prev = filtered.copy()
        return filtered

    def reset(self):
        self._prev = None
