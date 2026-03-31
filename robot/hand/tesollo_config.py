"""Tesollo DG 5F M configuration — YAML loader with typed access."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class NetworkConfig:
    listen_port: int = 9872
    listen_ip: str = "0.0.0.0"


@dataclass
class HandConfig:
    side: str = "right"
    ip: str = "169.254.186.72"
    port: int = 502


@dataclass
class RetargetConfig:
    calibration_factors: list[float] = field(default_factory=lambda: [
        1.0, 1.6, 1.3, 1.3,
        1.0, 1.0, 1.3, 1.7,
        1.0, 1.0, 1.3, 1.7,
        1.0, 1.0, 1.3, 1.7,
        1.0, 1.0, 1.0, 1.0,
    ])


@dataclass
class ControlConfig:
    hz: int = 60
    motion_time_ms: int = 20
    enable_on_start: bool = True


@dataclass
class TesolloConfig:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    hand: HandConfig = field(default_factory=HandConfig)
    retarget: RetargetConfig = field(default_factory=RetargetConfig)
    control: ControlConfig = field(default_factory=ControlConfig)

    @classmethod
    def load(cls, path: Optional[str] = None) -> "TesolloConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[TesolloConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()

        if "network" in data:
            cfg.network = NetworkConfig(**data["network"])
        if "hand" in data:
            cfg.hand = HandConfig(**data["hand"])
        if "retarget" in data:
            cfg.retarget = RetargetConfig(**data["retarget"])
        if "control" in data:
            cfg.control = ControlConfig(**data["control"])

        return cfg
