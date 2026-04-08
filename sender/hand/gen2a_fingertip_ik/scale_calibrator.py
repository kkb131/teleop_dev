"""Human → Robot bone length ratio calibration.

Computes per-finger scale factors from Manus skeleton bone lengths
vs DG5F URDF link lengths.
"""

import numpy as np

from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics

# Manus Raw Skeleton tip node indices (25 nodes total)
# [0]=wrist, [1-4]=thumb(5 nodes), [5-9]=index(5 nodes), [10-14]=middle,
# [15-19]=ring, [20-24]=pinky
# Tip = last node of each 5-node chain
MANUS_TIP_INDICES = [4, 9, 14, 19, 24]


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
        """DG5F palm→tip distances at zero configuration."""
        q_zero = np.zeros(20)
        tips = self._fk.fingertip_positions(q_zero)
        palm = self._fk.palm_position(q_zero)
        return np.array([np.linalg.norm(tips[i] - palm) for i in range(5)])

    def _compute_human_lengths(self, skeleton: np.ndarray) -> np.ndarray:
        """Manus skeleton wrist→tip distances."""
        wrist = skeleton[0, :3]
        lengths = np.zeros(5)
        for i, tip_idx in enumerate(MANUS_TIP_INDICES):
            if tip_idx < len(skeleton):
                lengths[i] = np.linalg.norm(skeleton[tip_idx, :3] - wrist)
        return lengths
