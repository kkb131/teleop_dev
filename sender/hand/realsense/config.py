"""RealSense sender configuration — YAML loader with typed access.

Mirrors sender/hand/manus_config.py structure so realsense_sender.py
can use the same CLI-override pattern.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


_DEFAULT_CONFIG = Path(__file__).parent / "default_config.yaml"


@dataclass
class NetworkConfig:
    target_ip: str = "192.168.0.10"
    port: int = 9872
    hz: int = 30          # RealSense 30Hz sweet spot (D405 native)


@dataclass
class HandConfig:
    side: str = "right"   # "left" or "right" (no "both" — single MediaPipe stream)


@dataclass
class CameraConfig:
    serial: Optional[str] = None
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass
class RetargetConfig:
    optimizer: str = "dexpilot"   # "dexpilot" or "vector"


@dataclass
class DisplayConfig:
    viz: bool = False             # OpenCV preview window


@dataclass
class RealsenseConfig:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    hand: HandConfig = field(default_factory=HandConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    retarget: RetargetConfig = field(default_factory=RetargetConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)

    @classmethod
    def load(cls, path: Optional[str] = None) -> "RealsenseConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[RealsenseConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()

        if "network" in data:
            cfg.network = NetworkConfig(**data["network"])
        if "hand" in data:
            cfg.hand = HandConfig(**data["hand"])
        if "camera" in data:
            cfg.camera = CameraConfig(**data["camera"])
        if "retarget" in data:
            cfg.retarget = RetargetConfig(**data["retarget"])
        if "display" in data:
            cfg.display = DisplayConfig(**data["display"])

        return cfg
