"""Exponential Moving Average filter with quaternion slerp for orientation."""

from typing import Optional, Tuple

import numpy as np
import pinocchio as pin


class ExpFilter:
    """Smooths target poses using EMA (position) and slerp (orientation).

    Operates on target poses, not raw deltas, to prevent drift.
    """

    def __init__(self, alpha_pos: float = 0.7, alpha_ori: float = 0.7):
        self._alpha_pos = alpha_pos
        self._alpha_ori = alpha_ori
        self._prev_pos: Optional[np.ndarray] = None
        self._prev_quat: Optional[pin.Quaternion] = None

    def reset(self, position: np.ndarray, quaternion: np.ndarray):
        """Initialize filter state from current robot pose.

        Args:
            position: [x, y, z]
            quaternion: [x, y, z, w] (pinocchio convention)
        """
        self._prev_pos = position.copy()
        self._prev_quat = pin.Quaternion(quaternion[3], quaternion[0],
                                         quaternion[1], quaternion[2])

    def update(
        self, position: np.ndarray, quaternion: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Apply EMA to position and slerp to orientation.

        Args:
            position: target [x, y, z]
            quaternion: target [x, y, z, w]

        Returns:
            (filtered_pos[3], filtered_quat[4] as xyzw)
        """
        if self._prev_pos is None:
            self.reset(position, quaternion)
            return position.copy(), quaternion.copy()

        # Position: linear EMA
        filtered_pos = (self._alpha_pos * position +
                        (1.0 - self._alpha_pos) * self._prev_pos)

        # Orientation: slerp
        new_quat = pin.Quaternion(quaternion[3], quaternion[0],
                                  quaternion[1], quaternion[2])
        filtered_quat = self._prev_quat.slerp(self._alpha_ori, new_quat)

        self._prev_pos = filtered_pos.copy()
        self._prev_quat = filtered_quat

        # Convert back to xyzw array
        quat_out = np.array([filtered_quat.x, filtered_quat.y,
                             filtered_quat.z, filtered_quat.w])

        return filtered_pos, quat_out
