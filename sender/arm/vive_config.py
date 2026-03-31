"""Vive Tracker configuration — YAML loader with typed access."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml


_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class NetworkConfig:
    target_ip: str = "192.168.0.10"
    port: int = 9871
    hz: int = 50


@dataclass
class TrackerEntry:
    name: str = "right_hand"
    serial: Optional[str] = None
    role: str = "teleop"


@dataclass
class CalibrationConfig:
    file: Optional[str] = None
    samples_per_point: int = 50


@dataclass
class ViveConfig:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    trackers: List[TrackerEntry] = field(
        default_factory=lambda: [TrackerEntry()]
    )
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)

    def get_teleop_tracker(self) -> TrackerEntry:
        """Get the first tracker with role='teleop'."""
        for t in self.trackers:
            if t.role == "teleop":
                return t
        return self.trackers[0] if self.trackers else TrackerEntry()

    @classmethod
    def load(cls, path: Optional[str] = None) -> "ViveConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[ViveConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()

        if "network" in data:
            cfg.network = NetworkConfig(**data["network"])

        if "trackers" in data:
            cfg.trackers = [
                TrackerEntry(**t) for t in data["trackers"]
            ]

        if "calibration" in data:
            cfg.calibration = CalibrationConfig(**data["calibration"])

        return cfg
