"""Manus Quantum Metagloves configuration — YAML loader with typed access."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class NetworkConfig:
    target_ip: str = "192.168.0.10"
    port: int = 9872
    hz: int = 60


@dataclass
class HandConfig:
    side: str = "right"  # "left", "right", or "both"
    dongle_id: int = 0


@dataclass
class JointMappingConfig:
    num_joints: int = 20
    calibration_file: Optional[str] = None


@dataclass
class SdkConfig:
    bin_path: str = "operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out"


@dataclass
class ManusConfig:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    hand: HandConfig = field(default_factory=HandConfig)
    joint_mapping: JointMappingConfig = field(default_factory=JointMappingConfig)
    sdk: SdkConfig = field(default_factory=SdkConfig)

    @classmethod
    def load(cls, path: Optional[str] = None) -> "ManusConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[ManusConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()

        if "network" in data:
            cfg.network = NetworkConfig(**data["network"])

        if "hand" in data:
            cfg.hand = HandConfig(**data["hand"])

        if "joint_mapping" in data:
            cfg.joint_mapping = JointMappingConfig(**data["joint_mapping"])

        if "sdk" in data:
            cfg.sdk = SdkConfig(**data["sdk"])

        return cfg
