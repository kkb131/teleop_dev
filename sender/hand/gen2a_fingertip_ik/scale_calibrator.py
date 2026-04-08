"""Human → Robot bone length ratio calibration.

Computes per-finger scale factors from Manus skeleton bone lengths
vs DG5F URDF link lengths.
"""

import numpy as np

from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics

# Manus Raw Skeleton tip node indices (confirmed from ROS2 topic)
# [0]=wrist, [1-4]=thumb(CMC,MCP,IP,TIP), [5-8]=index, [9-12]=middle,
# [13-16]=ring, [17-20]=pinky
# Tip indices: last node of each finger chain
MANUS_TIP_INDICES = [4, 8, 12, 16, 20]


class ScaleCalibrator:
    """Compute per-finger scale: robot_length / human_length.

    Parameters
    ----------
    fk : DG5FKinematics
        Robot FK model.
    """

    def __init__(self, fk: DG5FKinematics):
        self._fk = fk
        self._robot_lengths = self._compute_robot_lengths()

    def calibrate(self, skeleton: np.ndarray) -> np.ndarray:
        """Compute scale factors from open-hand skeleton.

        Parameters
        ----------
        skeleton : ndarray[N, 7]
            Manus raw skeleton nodes [x,y,z,qw,qx,qy,qz].

        Returns
        -------
        ndarray[5] — per-finger scale factors.
        """
        human_lengths = self._compute_human_lengths(skeleton)
        scales = self._robot_lengths / np.maximum(human_lengths, 1e-6)
        return scales

    def _compute_robot_lengths(self) -> np.ndarray:
        """DG5F wrist→tip distances at zero configuration."""
        q_zero = np.zeros(20)
        tips = self._fk.fingertip_positions(q_zero)
        # Use world origin (≈ wrist mount) as reference
        wrist = np.zeros(3)
        return np.array([np.linalg.norm(tips[i] - wrist) for i in range(5)])

    def _compute_human_lengths(self, skeleton: np.ndarray) -> np.ndarray:
        """Manus skeleton wrist→tip distances."""
        wrist = skeleton[0, :3]
        lengths = np.zeros(5)
        for i, tip_idx in enumerate(MANUS_TIP_INDICES):
            if tip_idx < len(skeleton):
                lengths[i] = np.linalg.norm(skeleton[tip_idx, :3] - wrist)
        return lengths
