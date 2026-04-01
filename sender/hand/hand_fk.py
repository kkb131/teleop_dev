#!/usr/bin/env python3
"""Human hand forward kinematics from Manus joint angles.

Computes 5 fingertip positions from 20 Manus ergonomics joint angles
using a simplified kinematic model with configurable bone lengths.

Joint layout per finger: [Spread, MCP_Flex, PIP_Flex, DIP_Flex]

Usage:
    from sender.hand.hand_fk import HumanHandFK
    fk = HumanHandFK()
    tips = fk.fingertip_positions(manus_angles)  # (5, 3)
"""

import math
import numpy as np


# Default human hand bone lengths (meters)
# [metacarpal, proximal, intermediate, distal] per finger
DEFAULT_BONE_LENGTHS = {
    "Thumb":  [0.040, 0.035, 0.028, 0.023],
    "Index":  [0.065, 0.040, 0.025, 0.020],
    "Middle": [0.065, 0.043, 0.028, 0.020],
    "Ring":   [0.060, 0.040, 0.026, 0.019],
    "Pinky":  [0.055, 0.032, 0.020, 0.017],
}

# Palm attachment positions (meters, relative to palm center)
# x=lateral, y=forward(fingers), z=up(dorsal)
DEFAULT_PALM_OFFSETS = {
    "Thumb":  np.array([-0.040, 0.010, 0.005]),
    "Index":  np.array([-0.025, 0.040, 0.0]),
    "Middle": np.array([-0.005, 0.042, 0.0]),
    "Ring":   np.array([0.015, 0.040, 0.0]),
    "Pinky":  np.array([0.032, 0.035, 0.0]),
}

# Base direction angles (radians, in palm plane)
DEFAULT_BASE_ANGLES = {
    "Thumb":  math.radians(-70),
    "Index":  math.radians(-15),
    "Middle": math.radians(0),
    "Ring":   math.radians(10),
    "Pinky":  math.radians(20),
}

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]


def _rot_x(angle):
    """Rotation matrix around X axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(angle):
    """Rotation matrix around Y axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(angle):
    """Rotation matrix around Z axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class HumanHandFK:
    """Simplified human hand forward kinematics.

    Converts Manus 20-DOF joint angles to fingertip 3D positions.
    Uses a planar kinematic chain per finger with spread (abduction).

    Parameters
    ----------
    bone_lengths : dict or None
        Per-finger bone lengths. None uses defaults.
    palm_offsets : dict or None
        Per-finger palm attachment positions. None uses defaults.
    baseline_angles : ndarray[20] or None
        Manus angles when hand is flat (open, fingers together).
        Subtracted before FK to zero-reference. None = no subtraction.
    """

    def __init__(self, bone_lengths=None, palm_offsets=None,
                 baseline_angles=None):
        self._bones = bone_lengths or DEFAULT_BONE_LENGTHS
        self._offsets = palm_offsets or DEFAULT_PALM_OFFSETS
        self._base_angles = DEFAULT_BASE_ANGLES
        self._baseline = baseline_angles

    def set_baseline(self, angles: np.ndarray):
        """Set open-hand baseline (subtracted from all future FK calls)."""
        self._baseline = angles.copy()

    def fingertip_positions(self, manus_angles: np.ndarray) -> np.ndarray:
        """Compute 5 fingertip positions from Manus joint angles.

        Parameters
        ----------
        manus_angles : ndarray[20]
            Joint angles in radians. Layout: [Spread, MCP, PIP, DIP] × 5.

        Returns
        -------
        ndarray[5, 3]
            Fingertip (x, y, z) positions relative to palm center.
        """
        angles = manus_angles.copy()
        if self._baseline is not None:
            angles = angles - self._baseline

        tips = np.zeros((5, 3))
        for f_idx, fname in enumerate(FINGER_NAMES):
            base = f_idx * 4
            spread = angles[base + 0]
            flexions = [angles[base + 1], angles[base + 2], angles[base + 3]]
            bones = self._bones[fname]

            # Start from palm attachment
            pos = self._offsets[fname].copy()

            # Base direction in palm plane (spread rotates around Z)
            base_angle = self._base_angles[fname]

            # Build rotation: spread around Z, then chain flex around X
            # Finger points in Y direction initially, spread rotates in XY
            direction = _rot_z(base_angle + spread) @ np.array([0, 1, 0])

            # Current orientation: tracks cumulative flexion
            # Flexion bends the finger in the sagittal plane (around local X)
            R = _rot_z(base_angle + spread)

            # Skip metacarpal (bone[0]) — it's inside the palm
            for j in range(3):  # proximal, intermediate, distal
                bone_len = bones[j + 1]
                # Apply flexion (rotation around local X axis)
                R = R @ _rot_x(flexions[j])
                # Extend along local Y
                pos = pos + R @ np.array([0, bone_len, 0])

            tips[f_idx] = pos

        return tips

    def fingertip_vectors(self, manus_angles: np.ndarray) -> np.ndarray:
        """Compute palm→fingertip unit direction vectors.

        Returns
        -------
        ndarray[5, 3]
            Normalized direction vectors.
        """
        tips = self.fingertip_positions(manus_angles)
        palm_center = np.zeros(3)
        vectors = np.zeros((5, 3))
        for i in range(5):
            v = tips[i] - palm_center
            norm = np.linalg.norm(v)
            if norm > 1e-6:
                vectors[i] = v / norm
            else:
                vectors[i] = v
        return vectors

    def all_keypoints(self, manus_angles: np.ndarray) -> np.ndarray:
        """Compute all 21 keypoints (palm + 4 per finger).

        Returns
        -------
        ndarray[21, 3]
            [0]=palm, [1-4]=thumb joints, [5-8]=index, ...
        """
        angles = manus_angles.copy()
        if self._baseline is not None:
            angles = angles - self._baseline

        kp = np.zeros((21, 3))
        kp[0] = np.zeros(3)  # palm center

        for f_idx, fname in enumerate(FINGER_NAMES):
            base = f_idx * 4
            spread = angles[base + 0]
            flexions = [angles[base + 1], angles[base + 2], angles[base + 3]]
            bones = self._bones[fname]

            pos = self._offsets[fname].copy()
            base_angle = self._base_angles[fname]
            R = _rot_z(base_angle + spread)

            for j in range(3):
                bone_len = bones[j + 1]
                R = R @ _rot_x(flexions[j])
                pos = pos + R @ np.array([0, bone_len, 0])
                kp[1 + f_idx * 4 + j] = pos.copy()

            # Tip = last position
            kp[1 + f_idx * 4 + 3] = pos.copy()

        return kp
