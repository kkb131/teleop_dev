"""Extract joint angles from 21 MediaPipe keypoints for DG5F mapping.

Takes (21, 3) keypoint positions and computes 20 joint angles:
  5 fingers x 4 joints (spread, MCP flex, PIP flex, DIP flex).

Convention (matches DG5F):
  - Flexion: 0 rad = fully extended (hand spread open)
             positive = flexion (bending/closing)
             ~pi rad = fully flexed (fist)
  - Spread:  signed angle from middle finger reference on palm plane
"""

import numpy as np

# MediaPipe keypoint chains per finger (wrist → tip)
FINGER_CHAINS = [
    [0, 1, 2, 3, 4],       # Thumb:  WRIST → CMC → MCP → IP → TIP
    [0, 5, 6, 7, 8],       # Index:  WRIST → MCP → PIP → DIP → TIP
    [0, 9, 10, 11, 12],    # Middle: WRIST → MCP → PIP → DIP → TIP
    [0, 13, 14, 15, 16],   # Ring:   WRIST → MCP → PIP → DIP → TIP
    [0, 17, 18, 19, 20],   # Pinky:  WRIST → MCP → PIP → DIP → TIP
]

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Base keypoint indices for each finger (used for spread computation)
_FINGER_BASE_INDICES = [1, 5, 9, 13, 17]

# Reference finger for spread: Middle (index 9)
_MIDDLE_BASE_IDX = 9
_INDEX_BASE_IDX = 5
_PINKY_BASE_IDX = 17


def _safe_normalize(v: np.ndarray) -> np.ndarray:
    """Normalize vector, returning zeros if near-zero length."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-8 else np.zeros_like(v)


def compute_flexion(pos: np.ndarray, parent: int, joint: int, child: int) -> float:
    """Compute flexion angle at a joint.

    Angle between incoming bone (parent→joint) and outgoing bone (joint→child).

    Convention (matches DG5F: 0=spread, +=flexion):
        0 rad  = bones in straight line (fully extended / spread)
        pi rad = bones folded back on each other (fully flexed)

    Returns angle in radians.
    """
    v_in = pos[joint] - pos[parent]
    v_out = pos[child] - pos[joint]
    n_in = np.linalg.norm(v_in)
    n_out = np.linalg.norm(v_out)
    if n_in < 1e-8 or n_out < 1e-8:
        return 0.0
    cos_a = np.dot(v_in, v_out) / (n_in * n_out)
    # arccos gives 0 when parallel (extended), pi when anti-parallel (flexed)
    # pi - arccos flips: 0 = extended, pi = flexed (matches DG5F convention)
    return float(np.pi - np.arccos(np.clip(cos_a, -1.0, 1.0)))


def compute_spread(pos: np.ndarray, finger_base_idx: int) -> float:
    """Compute spread (abduction) angle for a finger.

    Projects finger base direction onto the palm plane and measures
    the signed angle from the middle finger reference direction.

    Returns angle in radians (positive = away from middle finger).
    """
    wrist = pos[0]

    # Palm plane: defined by WRIST→INDEX and WRIST→PINKY
    v_index = pos[_INDEX_BASE_IDX] - wrist
    v_pinky = pos[_PINKY_BASE_IDX] - wrist

    palm_normal = np.cross(v_index, v_pinky)
    n_norm = np.linalg.norm(palm_normal)
    if n_norm < 1e-8:
        return 0.0
    palm_normal = palm_normal / n_norm

    # Reference direction: WRIST → MIDDLE_MCP
    ref_dir = pos[_MIDDLE_BASE_IDX] - wrist
    # Project onto palm plane
    ref_dir = ref_dir - np.dot(ref_dir, palm_normal) * palm_normal
    ref_dir = _safe_normalize(ref_dir)

    if np.linalg.norm(ref_dir) < 1e-8:
        return 0.0

    # Finger base direction projected onto palm plane
    finger_dir = pos[finger_base_idx] - wrist
    finger_dir = finger_dir - np.dot(finger_dir, palm_normal) * palm_normal
    finger_dir = _safe_normalize(finger_dir)

    if np.linalg.norm(finger_dir) < 1e-8:
        return 0.0

    # Signed angle via atan2(cross, dot)
    cos_a = np.clip(np.dot(ref_dir, finger_dir), -1.0, 1.0)
    cross = np.cross(ref_dir, finger_dir)
    sin_a = np.dot(cross, palm_normal)  # signed component along normal
    return float(np.arctan2(sin_a, cos_a))


def extract_all_angles(keypoints_3d: np.ndarray) -> np.ndarray:
    """Extract 20 joint angles from (21, 3) keypoint positions.

    Returns (20,) array ordered as DG5F convention:
        [thumb_spread, thumb_mcp, thumb_pip, thumb_dip,
         index_spread, index_mcp, index_pip, index_dip,
         ... (middle, ring, pinky)]

    All values in radians.
    """
    pos = keypoints_3d
    angles = np.zeros(20, dtype=np.float64)

    for f_idx, chain in enumerate(FINGER_CHAINS):
        base = f_idx * 4

        # Joint 1: Spread
        angles[base + 0] = compute_spread(pos, _FINGER_BASE_INDICES[f_idx])

        # Joint 2: MCP flexion — angle at chain[1] between chain[0]→[1] and [1]→[2]
        angles[base + 1] = compute_flexion(pos, chain[0], chain[1], chain[2])

        # Joint 3: PIP flexion — angle at chain[2] between chain[1]→[2] and [2]→[3]
        angles[base + 2] = compute_flexion(pos, chain[1], chain[2], chain[3])

        # Joint 4: DIP flexion — angle at chain[3] between chain[2]→[3] and [3]→[4]
        angles[base + 3] = compute_flexion(pos, chain[2], chain[3], chain[4])

    return angles


def extract_all_angles_debug(keypoints_3d: np.ndarray) -> dict:
    """Same as extract_all_angles but returns detailed breakdown.

    Returns dict with:
        angles: (20,) array
        per_finger: list of dicts with spread, mcp, pip, dip in degrees
    """
    angles = extract_all_angles(keypoints_3d)
    per_finger = []
    for f_idx, name in enumerate(FINGER_NAMES):
        base = f_idx * 4
        per_finger.append({
            "name": name,
            "spread_deg": float(np.degrees(angles[base + 0])),
            "mcp_flex_deg": float(np.degrees(angles[base + 1])),
            "pip_flex_deg": float(np.degrees(angles[base + 2])),
            "dip_flex_deg": float(np.degrees(angles[base + 3])),
        })
    return {"angles": angles, "per_finger": per_finger}
