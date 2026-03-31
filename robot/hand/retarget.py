#!/usr/bin/env python3
"""Manus glove → Tesollo DG5F M hand retargeting.

Transforms 20 Manus ergonomics joint angles into 20 DG5F motor commands.
Algorithm based on Tesollo's official manus_retarget.py (dg5f_ros2).

Manus joint layout (per finger, 4 joints each):
    Thumb:  CMC_Spread, CMC_Flex, MCP_Flex, IP_Flex
    Index:  MCP_Spread, MCP_Flex, PIP_Flex, DIP_Flex
    Middle: MCP_Spread, MCP_Flex, PIP_Flex, DIP_Flex
    Ring:   MCP_Spread, MCP_Flex, PIP_Flex, DIP_Flex
    Pinky:  MCP_Spread, MCP_Flex, PIP_Flex, DIP_Flex

DG5F joint layout (per finger, 4 joints each):
    rj_dg_[1-5]_[1-4]  (right hand)
    lj_dg_[1-5]_[1-4]  (left hand)

Usage:
    from teleop_dev.robot.hand.retarget import ManusToD5FRetarget

    retarget = ManusToD5FRetarget(hand_side="right")
    dg5f_angles = retarget.retarget(manus_angles)  # ndarray[20] → ndarray[20]
"""

import math
from dataclasses import dataclass, field

import numpy as np


# ─────────────────────────────────────────────────────────
# Joint limits (radians) — from DG5F URDF
# ─────────────────────────────────────────────────────────

@dataclass
class JointLimits:
    """Per-joint min/max in radians."""
    min_rad: float
    max_rad: float


# Right hand joint limits [20 joints]
RIGHT_LIMITS = [
    # Thumb (rj_dg_1_1 ~ 1_4)
    JointLimits(-0.384, 0.890),
    JointLimits(-math.pi, 0.0),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Index (rj_dg_2_1 ~ 2_4)
    JointLimits(-0.419, 0.611),
    JointLimits(0.0, 2.007),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Middle (rj_dg_3_1 ~ 3_4)
    JointLimits(-0.611, 0.611),
    JointLimits(0.0, 1.955),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Ring (rj_dg_4_1 ~ 4_4)
    JointLimits(-0.611, 0.419),
    JointLimits(0.0, 1.902),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Pinky (rj_dg_5_1 ~ 5_4)
    JointLimits(-0.017, 1.047),
    JointLimits(-0.419, 0.611),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
]

# Left hand joint limits [20 joints]
LEFT_LIMITS = [
    # Thumb (lj_dg_1_1 ~ 1_4)
    JointLimits(-0.890, 0.384),
    JointLimits(0.0, math.pi),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Index (lj_dg_2_1 ~ 2_4)
    JointLimits(-0.611, 0.419),
    JointLimits(0.0, 2.007),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Middle (lj_dg_3_1 ~ 3_4)
    JointLimits(-0.611, 0.611),
    JointLimits(0.0, 1.955),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Ring (lj_dg_4_1 ~ 4_4)
    JointLimits(-0.419, 0.611),
    JointLimits(0.0, 1.902),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
    # Pinky (lj_dg_5_1 ~ 5_4)
    JointLimits(-1.047, 0.017),
    JointLimits(-0.611, 0.419),
    JointLimits(-math.pi / 2, math.pi / 2),
    JointLimits(-math.pi / 2, math.pi / 2),
]

# ─────────────────────────────────────────────────────────
# Calibration factors — amplify/dampen joint responsiveness
# ─────────────────────────────────────────────────────────

DEFAULT_CALIBRATION_FACTORS = [
    # Thumb       Index        Middle       Ring         Pinky
    1.0, 1.6, 1.3, 1.3,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.0, 1.0,
]

# Direction arrays — sign convention for anatomical mirroring
RIGHT_DIRECTIONS = [
    # Thumb       Index        Middle       Ring         Pinky
     1, -1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
     1, -1,  1,  1,
]

LEFT_DIRECTIONS = [
    -1,  1, -1, -1,
     1,  1,  1,  1,
     1,  1,  1,  1,
     1,  1,  1,  1,
    -1,  1,  1,  1,
]

DEG2RAD = math.pi / 180.0


class ManusToD5FRetarget:
    """Retargets Manus glove joint angles to DG5F motor commands.

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    calibration_factors : list[float] or None
        Per-joint scaling factors (20 values). None uses defaults.
    """

    def __init__(self, hand_side: str = "right",
                 calibration_factors: list[float] | None = None):
        self._side = hand_side
        self._is_right = (hand_side == "right")

        self._limits = RIGHT_LIMITS if self._is_right else LEFT_LIMITS
        self._directions = np.array(
            RIGHT_DIRECTIONS if self._is_right else LEFT_DIRECTIONS,
            dtype=np.float64,
        )
        self._cal = np.array(
            calibration_factors or DEFAULT_CALIBRATION_FACTORS,
            dtype=np.float64,
        )

    def retarget(self, manus_angles: np.ndarray) -> np.ndarray:
        """Transform Manus ergonomics data to DG5F joint commands.

        Parameters
        ----------
        manus_angles : ndarray[20]
            Raw joint angles from Manus glove (radians or SDK-normalized).
            Layout: [Thumb(4), Index(4), Middle(4), Ring(4), Pinky(4)]

        Returns
        -------
        ndarray[20]
            DG5F target positions in radians, clamped to joint limits.
        """
        # Convert to degrees for the transformation math
        # (Tesollo's reference uses degree-based equations)
        q_deg = np.degrees(manus_angles.astype(np.float64))
        qd = np.zeros(20, dtype=np.float64)

        # ── Thumb (indices 0-3) ─────────────────────────
        # Swap spread/flex + offsets
        qd[0] = (58.5 - q_deg[1]) * DEG2RAD    # stretch inverted with 58.5 deg offset
        qd[1] = (q_deg[0] + 20.0) * DEG2RAD    # spread with +20 deg offset
        qd[2] = q_deg[2] * DEG2RAD             # PIP direct
        qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * DEG2RAD  # DIP averaged

        # ── Index (indices 4-7) ─────────────────────────
        qd[4] = q_deg[4] * DEG2RAD             # spread direct
        qd[5] = q_deg[5] * DEG2RAD             # MCP direct
        qd[6] = (q_deg[6] - 40.0) * DEG2RAD   # PIP with -40 deg offset
        qd[7] = 0.5 * (q_deg[6] + q_deg[7]) * DEG2RAD  # DIP averaged

        # ── Middle (indices 8-11) ───────────────────────
        qd[8] = q_deg[8] * DEG2RAD             # spread direct
        qd[9] = q_deg[9] * DEG2RAD             # MCP direct
        qd[10] = (q_deg[10] - 30.0) * DEG2RAD  # PIP with -30 deg offset
        qd[11] = 0.5 * (q_deg[10] + q_deg[11]) * DEG2RAD  # DIP averaged

        # ── Ring (indices 12-15) ────────────────────────
        qd[12] = q_deg[12] * DEG2RAD           # spread direct
        qd[13] = q_deg[13] * DEG2RAD           # MCP direct
        qd[14] = q_deg[14] * DEG2RAD           # PIP direct
        qd[15] = q_deg[15] * DEG2RAD           # DIP direct

        # ── Pinky (indices 16-19) ───────────────────────
        # Conditional spread scaling based on finger curl
        if q_deg[17] > 55.0 and q_deg[18] > 25.0 and q_deg[19] > 20.0:
            spread_mult = 2.0
        else:
            spread_mult = 1.0 / 1.5

        qd[16] = q_deg[16] * spread_mult * DEG2RAD  # spread conditional
        qd[17] = q_deg[17] * DEG2RAD                # MCP direct
        qd[18] = q_deg[18] * DEG2RAD                # PIP direct
        qd[19] = q_deg[19] * DEG2RAD                # DIP direct

        # ── Apply calibration & direction ───────────────
        mqd = qd * self._cal * self._directions

        # ── Post-processing: reject impossible postures ─
        self._apply_posture_constraints(mqd)

        # ── Clamp to joint limits ───────────────────────
        for i in range(20):
            mqd[i] = np.clip(mqd[i], self._limits[i].min_rad, self._limits[i].max_rad)

        return mqd

    def _apply_posture_constraints(self, mqd: np.ndarray):
        """Reject anatomically impossible joint values in-place."""
        if self._is_right:
            # Thumb: base/PIP/DIP must be >= 0 after transform
            if mqd[0] < 0:
                mqd[0] = 0.0
            if mqd[2] < 0:
                mqd[2] = 0.0
            if mqd[3] < 0:
                mqd[3] = 0.0
            # Index/Middle/Ring spreads must be <= 0
            for idx in [4, 8, 12]:
                if mqd[idx] > 0:
                    mqd[idx] = 0.0
            # Pinky spread must be <= 0
            if mqd[16] > 0:
                mqd[16] = 0.0
        else:
            # Left hand: mirrored constraints
            if mqd[0] > 0:
                mqd[0] = 0.0
            if mqd[2] > 0:
                mqd[2] = 0.0
            if mqd[3] > 0:
                mqd[3] = 0.0
            for idx in [4, 8, 12]:
                if mqd[idx] < 0:
                    mqd[idx] = 0.0
            if mqd[16] < 0:
                mqd[16] = 0.0

    def get_limits(self) -> list[JointLimits]:
        """Return joint limits for the configured hand side."""
        return list(self._limits)


# ─────────────────────────────────────────────────────────
# Standalone demo
# ─────────────────────────────────────────────────────────

def main():
    """Demo: retarget mock Manus data and print DG5F angles."""
    import argparse

    parser = argparse.ArgumentParser(description="Retarget demo")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    args = parser.parse_args()

    retarget = ManusToD5FRetarget(hand_side=args.hand)

    # Mock: hand half-closed
    manus = np.array([
        # Thumb:  spread=0.1, flex=0.5, mcp=0.6, ip=0.4
        0.1, 0.5, 0.6, 0.4,
        # Index:  spread=0.05, mcp=0.8, pip=0.9, dip=0.5
        0.05, 0.8, 0.9, 0.5,
        # Middle: spread=0.0, mcp=0.7, pip=0.8, dip=0.4
        0.0, 0.7, 0.8, 0.4,
        # Ring:   spread=-0.05, mcp=0.6, pip=0.7, dip=0.3
        -0.05, 0.6, 0.7, 0.3,
        # Pinky:  spread=-0.1, mcp=0.5, pip=0.6, dip=0.25
        -0.1, 0.5, 0.6, 0.25,
    ], dtype=np.float64)

    from teleop_dev.robot.hand.dg5f_client import RIGHT_JOINT_NAMES, LEFT_JOINT_NAMES
    names = RIGHT_JOINT_NAMES if args.hand == "right" else LEFT_JOINT_NAMES

    dg5f = retarget.retarget(manus)

    print(f"{'Joint':12s} {'Manus(rad)':>12s} {'DG5F(rad)':>12s} {'DG5F(deg)':>12s}")
    print("-" * 52)
    for i in range(20):
        print(f"{names[i]:12s} {manus[i]:+12.4f} {dg5f[i]:+12.4f} {math.degrees(dg5f[i]):+12.2f}")


if __name__ == "__main__":
    main()
