"""Shared MANO coordinate frame transform for hand keypoints.

This module implements the same transform as dex-retargeting's
``single_hand_detector.py`` so that any sensing source (phone camera,
RealSense D405, etc.) can produce keypoints in the **MANO convention**
expected by dex-retargeting's optimizers.

Reference:
    dex-retargeting/example/vector_retargeting/single_hand_detector.py
    - estimate_frame_from_hand_points()
    - OPERATOR2MANO_RIGHT / OPERATOR2MANO_LEFT

Pipeline:
    1. Shift origin to wrist (index 0)
    2. Estimate wrist frame via SVD on palm plane (wrist, index_MCP, middle_MCP)
    3. Rotate into MANO convention (`kp @ wrist_rot @ operator2mano`)
"""

import numpy as np

# Fixed rotations from the operator (wrist-aligned) frame to MANO convention.
# Copied verbatim from dex-retargeting's single_hand_detector.py.
OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)


def estimate_wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """Estimate the wrist coordinate frame from 21-point hand keypoints.

    Uses wrist (0), index MCP (5), middle MCP (9) to define the palm plane
    via SVD, then orthonormalizes the basis.

    This mirrors ``estimate_frame_from_hand_points`` in dex-retargeting's
    single_hand_detector.py.

    Args:
        keypoints: (21, 3) hand keypoints (already wrist-centered).

    Returns:
        (3, 3) rotation matrix: columns are x/y/z basis vectors of the wrist frame.
    """
    assert keypoints.shape == (21, 3)
    points = keypoints[[0, 5, 9], :]

    # x direction: palm → middle finger base
    x_vector = points[0] - points[2]

    # SVD to find palm plane normal
    centered = points - np.mean(points, axis=0, keepdims=True)
    _u, _s, v = np.linalg.svd(centered)
    normal = v[2, :]

    # Gram-Schmidt: make x orthogonal to normal
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / (np.linalg.norm(x) + 1e-10)
    z = np.cross(x, normal)

    # Orientation disambiguation: z should roughly point from pinky → index
    if np.sum(z * (points[1] - points[2])) < 0:
        normal = -normal
        z = -z

    frame = np.stack([x, normal, z], axis=1)
    return frame.astype(np.float32)


def apply_mano_transform(
    keypoints: np.ndarray,
    hand_type: str = "right",
) -> np.ndarray:
    """Apply full MANO transform to wrist-centered keypoints.

    Args:
        keypoints: (21, 3) wrist-centered keypoints.
        hand_type: "right" or "left".

    Returns:
        (21, 3) keypoints in MANO frame.
    """
    wrist_rot = estimate_wrist_frame(keypoints)
    if hand_type.lower() == "left":
        return keypoints @ wrist_rot @ OPERATOR2MANO_LEFT
    return keypoints @ wrist_rot @ OPERATOR2MANO_RIGHT
