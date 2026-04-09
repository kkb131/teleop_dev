"""Manus raw skeleton (25 nodes) → MANO 21-keypoint remap.

The Manus SDK / manus_data_publisher emits 25 raw skeleton nodes per hand:
    1 wrist + thumb (4 joints, no Intermediate) + 4 fingers × 5 joints
dex-retargeting (and MediaPipe / MANO standard) expects 21 keypoints:
    wrist + 5 × 4 joints. The 4 non-thumb Metacarpal nodes have no MANO
    counterpart and must be dropped.

This module ports the deterministic remapping helper from
``retarget_dev/sensing/manus/ros2_provider.py`` into teleop_dev so the
manus_reader_ros2.py callback can produce HandData.skeleton in the (21, 7)
MANO layout that DexRetargetWrapper expects.

WARNING — anatomical labels in publisher strings are off-by-one vs SDK enum:
    SDK enum         publisher string   anatomical
    Metacarpal       "MCP"              CMC / metacarpal-base
    Proximal         "PIP"              MCP (knuckle)
    Intermediate     "IP"               PIP
    Distal           "DIP"              DIP
    Tip              "TIP"              fingertip
We match the publisher strings (the actual wire format), not the labels.
"""

from typing import Optional

import numpy as np


# (chain_type, joint_type) → MANO 21-keypoint index
_MANO_REMAP = {
    # Thumb (4 joints — no Intermediate)
    ("Thumb", "MCP"): 1,    # → MANO thumb CMC
    ("Thumb", "PIP"): 2,    # → MANO thumb MCP
    ("Thumb", "DIP"): 3,    # → MANO thumb IP
    ("Thumb", "TIP"): 4,    # → MANO thumb tip
    # Index — drop SDK "MCP" (Metacarpal)
    ("Index", "PIP"): 5,
    ("Index", "IP"): 6,
    ("Index", "DIP"): 7,
    ("Index", "TIP"): 8,
    # Middle
    ("Middle", "PIP"): 9,
    ("Middle", "IP"): 10,
    ("Middle", "DIP"): 11,
    ("Middle", "TIP"): 12,
    # Ring
    ("Ring", "PIP"): 13,
    ("Ring", "IP"): 14,
    ("Ring", "DIP"): 15,
    ("Ring", "TIP"): 16,
    # Pinky
    ("Pinky", "PIP"): 17,
    ("Pinky", "IP"): 18,
    ("Pinky", "DIP"): 19,
    ("Pinky", "TIP"): 20,
}


def remap_to_mano_21(raw_nodes) -> Optional[np.ndarray]:
    """Convert Manus raw skeleton (~25 nodes) → MANO 21-node (21, 7) layout.

    Uses (chain_type, joint_type) string pair on each ManusRawNode to
    deterministically place each node, regardless of publish order. The
    wrist comes from any node with chain_type=="Hand". The 4 non-thumb
    Metacarpal nodes have no MANO counterpart and are dropped.

    Returns
    -------
    np.ndarray of shape (21, 7) with rows [x, y, z, qw, qx, qy, qz],
    or None if any required MANO slot is unfilled (e.g., partial dropout).
    """
    skel = np.full((21, 7), np.nan, dtype=np.float32)
    for node in raw_nodes:
        if node.chain_type == "Hand":
            idx = 0  # wrist root — joint_type may be "Invalid"
        else:
            idx = _MANO_REMAP.get((node.chain_type, node.joint_type))
            if idx is None:
                continue  # SDK Metacarpal of non-thumb finger → drop
        p = node.pose.position
        q = node.pose.orientation
        skel[idx] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    if np.isnan(skel).any():
        return None
    return skel
