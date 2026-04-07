"""Abstract base class for hand retargeting models.

All retarget modes implement this interface:
    skeleton (N, 7) → retarget() → DG5F joint angles (20,)
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


class RetargetBase(ABC):
    """Abstract base for hand retargeting: skeleton → robot joint angles."""

    @abstractmethod
    def retarget(self, skeleton: np.ndarray) -> np.ndarray:
        """Convert skeleton data to robot joint angles.

        Parameters
        ----------
        skeleton : ndarray
            (N, 7) per node: [x, y, z, qw, qx, qy, qz]

        Returns
        -------
        ndarray (20,) DG5F joint angles in radians.
        """

    def calibrate_baseline(self, skeleton: np.ndarray) -> None:
        """Optional: record open-hand baseline from skeleton."""
        pass

    def calibrate_fist(self, skeleton: np.ndarray) -> None:
        """Optional: record fist pose for scale computation."""
        pass

    def get_debug_info(self, skeleton: np.ndarray, q: np.ndarray) -> Optional[dict]:
        """Optional: return debug diagnostics."""
        return None
