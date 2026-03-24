"""Teleop configuration — YAML loader with typed access."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml

from teleop_dev.robot.config import URDF_PATH, JOINT_NAMES, DEFAULT_ROBOT_IP


_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class RobotConfig:
    ip: str = DEFAULT_ROBOT_IP
    mode: str = "sim"


@dataclass
class ControlConfig:
    frequency_sim: int = 50
    frequency_rtde: int = 125


@dataclass
class InputConfig:
    type: str = "keyboard"
    cartesian_step: float = 0.01
    rotation_step: float = 0.05
    xbox_linear_scale: float = 0.03
    xbox_angular_scale: float = 0.08
    network_port: int = 9870
    network_linear_scale: float = 0.005
    network_angular_scale: float = 0.015
    # Vive Tracker input
    vive_port: int = 9871
    vive_linear_scale: float = 1.0
    vive_angular_scale: float = 1.0
    vive_deadzone: float = 0.002
    vive_calibration_file: Optional[str] = None
    # Unified teleop protocol input
    unified_port: int = 9871


@dataclass
class FilterConfig:
    alpha_position: float = 0.85
    alpha_orientation: float = 0.85


@dataclass
class IKConfig:
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    posture_cost: float = 1e-3
    damping: float = 1e-12
    soft_sync_alpha: float = 0.05

    def __post_init__(self):
        # PyYAML safe_load parses "1e-12" (no dot) as string
        self.position_cost = float(self.position_cost)
        self.orientation_cost = float(self.orientation_cost)
        self.posture_cost = float(self.posture_cost)
        self.damping = float(self.damping)
        self.soft_sync_alpha = float(self.soft_sync_alpha)


@dataclass
class WorkspaceConfig:
    x: List[float] = field(default_factory=lambda: [-0.8, 0.8])
    y: List[float] = field(default_factory=lambda: [-0.8, 0.8])
    z: List[float] = field(default_factory=lambda: [0.05, 1.2])


@dataclass
class SafetyConfig:
    packet_timeout_ms: int = 200
    max_joint_vel: float = 0.5
    max_ee_velocity: float = 0.1
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)


@dataclass
class AdmittanceConfig:
    enabled_by_default: bool = False
    default_preset: str = "MEDIUM"
    max_displacement_trans: float = 0.15
    max_displacement_rot: float = 0.3
    force_deadzone: List[float] = field(
        default_factory=lambda: [3.0, 3.0, 3.0, 0.3, 0.3, 0.3]
    )
    force_saturation: float = 100.0
    torque_saturation: float = 10.0


@dataclass
class TeleopConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    input: InputConfig = field(default_factory=InputConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    admittance: AdmittanceConfig = field(default_factory=AdmittanceConfig)

    # From standalone.config (read-only)
    urdf_path: str = URDF_PATH
    joint_names: List[str] = field(default_factory=lambda: list(JOINT_NAMES))

    @property
    def frequency(self) -> int:
        if self.robot.mode == "rtde":
            return self.control.frequency_rtde
        return self.control.frequency_sim

    @property
    def dt(self) -> float:
        return 1.0 / self.frequency

    @classmethod
    def load(cls, path: Optional[str] = None) -> "TeleopConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[TeleopConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()

        if "robot" in data:
            cfg.robot = RobotConfig(**data["robot"])
        if "control" in data:
            cfg.control = ControlConfig(**data["control"])
        if "input" in data:
            cfg.input = InputConfig(**data["input"])
        if "filter" in data:
            cfg.filter = FilterConfig(**data["filter"])
        if "ik" in data:
            cfg.ik = IKConfig(**data["ik"])
        if "safety" in data:
            sd = data["safety"]
            ws = WorkspaceConfig(**sd.pop("workspace")) if "workspace" in sd else WorkspaceConfig()
            cfg.safety = SafetyConfig(workspace=ws, **sd)
        if "admittance" in data:
            cfg.admittance = AdmittanceConfig(**data["admittance"])

        return cfg
