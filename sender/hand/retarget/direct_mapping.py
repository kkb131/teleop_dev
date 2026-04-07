"""Direct Mapping retargeting — skeleton 3D → angle extraction → linear mapping.

Adapted from retarget_dev/models/direct_mapping for use in teleop_dev sender.
Input is Manus skeleton (N, 7) instead of HandKeypoints.

Pipeline:
    skeleton (N, 7) → extract positions (N, 3) → angle_extractor →
    scale * (angle - baseline) → clip → DG5F q (20,)

2-step calibration:
    1. Open hand (spread): capture baseline angles
    2. Fist: compute scale_factors = q_max / human_ROM
"""

import logging
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

from sender.hand.retarget.base import RetargetBase
from sender.hand.retarget.angle_extractor import (
    extract_all_angles,
    extract_all_angles_debug,
    FINGER_NAMES,
)
from sender.hand.retarget.dg5f_fk import DG5FKinematics

logger = logging.getLogger(__name__)

_DEFAULT_CONFIG = Path(__file__).parent / "config" / "direct_mapping.yaml"


def _skeleton_to_positions(skeleton: np.ndarray) -> np.ndarray:
    """Extract (N, 3) xyz positions from (N, 7) skeleton [x,y,z,qw,qx,qy,qz]."""
    return skeleton[:, :3].copy()


class DirectMappingRetarget(RetargetBase):
    """Direct angle mapping with baseline calibration.

    Formula:
        delta = human_angles - baseline     (subtract open-hand resting angles)
        delta = max(delta, 0)               (flexion only: no negative values)
        q     = scale * delta + offset      (map to DG5F range)
        q     = clip(q, q_min, q_max)       (enforce joint limits)
    """

    def __init__(self, hand_side: str = "right", config_path: str = None):
        self._fk = DG5FKinematics(hand_side=hand_side)
        self._q_min = self._fk.q_min
        self._q_max = self._fk.q_max

        self._scale = np.ones(20, dtype=np.float64)
        self._offsets = np.zeros(20, dtype=np.float64)
        self._baseline = np.zeros(20, dtype=np.float64)
        self._calibrated = False

        # Load config if provided
        cfg = config_path or str(_DEFAULT_CONFIG)
        if Path(cfg).exists():
            self._load_config(cfg)

        # Cache for debug
        self._last_human_angles: Optional[np.ndarray] = None
        self._last_delta: Optional[np.ndarray] = None

    def calibrate_baseline(self, skeleton: np.ndarray) -> None:
        """Set baseline from open-hand skeleton."""
        pos = _skeleton_to_positions(skeleton)
        self._baseline = extract_all_angles(pos)
        self._calibrated = True
        logger.info(
            "Baseline set. MCP baselines: [%.1f° %.1f° %.1f° %.1f° %.1f°]",
            *[np.degrees(self._baseline[f * 4 + 1]) for f in range(5)],
        )
        print(f"[DirectMapping] Baseline calibrated (MCP: "
              f"{[f'{np.degrees(self._baseline[f*4+1]):.1f}°' for f in range(5)]})")

    def calibrate_baseline_from_frames(self, frames: list[np.ndarray]) -> None:
        """Set baseline from averaged multiple skeleton frames."""
        all_angles = np.array([extract_all_angles(_skeleton_to_positions(f)) for f in frames])
        self._baseline = np.mean(all_angles, axis=0)
        self._calibrated = True
        print(f"[DirectMapping] Baseline calibrated from {len(frames)} frames")

    def calibrate_fist(self, skeleton: np.ndarray) -> None:
        """Compute scale_factors from fist skeleton. Requires baseline set first."""
        pos = _skeleton_to_positions(skeleton)
        fist_angles = extract_all_angles(pos)
        self._compute_scale_from_fist(fist_angles)

    def calibrate_fist_from_frames(self, frames: list[np.ndarray]) -> None:
        """Compute scale_factors from averaged fist frames."""
        all_angles = np.array([extract_all_angles(_skeleton_to_positions(f)) for f in frames])
        fist_angles = np.mean(all_angles, axis=0)
        self._compute_scale_from_fist(fist_angles)

    def _compute_scale_from_fist(self, fist_angles: np.ndarray) -> None:
        """Internal: scale = q_max / human_ROM for flexion joints."""
        human_rom = self._baseline - fist_angles
        min_rom = np.radians(5.0)

        for f in range(5):
            base = f * 4
            for j in [1, 2, 3]:  # MCP, PIP, DIP (skip spread)
                idx = base + j
                if human_rom[idx] > min_rom:
                    self._scale[idx] = self._q_max[idx] / human_rom[idx]

        print(f"[DirectMapping] Fist calibrated. MCP scales: "
              f"{[f'{self._scale[f*4+1]:.2f}' for f in range(5)]}")

    def retarget(self, skeleton: np.ndarray) -> np.ndarray:
        """Convert skeleton (N, 7) → DG5F joint angles (20,)."""
        pos = _skeleton_to_positions(skeleton)
        human_angles = extract_all_angles(pos)
        self._last_human_angles = human_angles.copy()

        if not self._calibrated:
            # Auto-calibrate from first frame
            self._baseline = human_angles.copy()
            self._calibrated = True
            print("[DirectMapping] Auto-calibrated from first frame (hold hand open for better results)")

        # Compute delta from baseline
        delta = human_angles.copy()
        for f in range(5):
            base = f * 4
            # Spread: human - baseline (signed)
            delta[base + 0] = human_angles[base + 0] - self._baseline[base + 0]
            # Flexion (MCP, PIP, DIP): baseline - human → positive = flexion
            for j in [1, 2, 3]:
                idx = base + j
                delta[idx] = max(self._baseline[idx] - human_angles[idx], 0.0)

        # Thumb MCP: DG5F uses negative direction [-180°, 0°]
        delta[1] = -delta[1]

        self._last_delta = delta.copy()

        q = self._scale * delta + self._offsets
        return np.clip(q, self._q_min, self._q_max)

    def get_debug_info(self, skeleton: np.ndarray, q: np.ndarray) -> Optional[dict]:
        pos = _skeleton_to_positions(skeleton)
        debug = extract_all_angles_debug(pos)
        delta_deg = np.degrees(self._last_delta) if self._last_delta is not None else np.zeros(20)
        return {
            "human_angles_deg": np.degrees(debug["angles"]),
            "delta_deg": delta_deg,
            "dg5f_angles_deg": np.degrees(q),
            "per_finger": debug["per_finger"],
            "baseline_deg": np.degrees(self._baseline),
            "scale_factors": self._scale.copy(),
            "calibrated": self._calibrated,
        }

    def save_config(self, path: str) -> None:
        """Save current calibration to YAML."""
        if yaml is None:
            print("[DirectMapping] PyYAML not installed, cannot save config")
            return
        cfg = {
            "baseline": [round(v, 4) for v in self._baseline.tolist()],
            "scale_factors": [round(v, 4) for v in self._scale.tolist()],
            "offsets": [round(v, 4) for v in self._offsets.tolist()],
        }
        with open(path, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
        print(f"[DirectMapping] Config saved to {path}")

    def _load_config(self, path: str) -> None:
        if yaml is None:
            return
        p = Path(path)
        if not p.exists():
            return
        with open(p) as f:
            cfg = yaml.safe_load(f)
        if cfg is None:
            return
        if "baseline" in cfg and len(cfg["baseline"]) == 20:
            self._baseline = np.array(cfg["baseline"], dtype=np.float64)
            self._calibrated = True
        if "scale_factors" in cfg and len(cfg["scale_factors"]) == 20:
            self._scale = np.array(cfg["scale_factors"], dtype=np.float64)
        if "offsets" in cfg and len(cfg["offsets"]) == 20:
            self._offsets = np.array(cfg["offsets"], dtype=np.float64)
