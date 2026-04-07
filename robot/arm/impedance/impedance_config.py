"""Impedance teleop configuration — YAML loader with typed access."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml

from robot.config import URDF_PATH, JOINT_NAMES, DEFAULT_ROBOT_IP


_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class RobotConfig:
    ip: str = DEFAULT_ROBOT_IP
    mode: str = "sim"


@dataclass
class ControlConfig:
    frequency_sim: int = 50
    frequency_rtde: int = 500


@dataclass
class InputConfig:
    type: str = "keyboard"
    cartesian_step: float = 0.01
    rotation_step: float = 0.05
    unified_port: int = 9871


@dataclass
class FilterConfig:
    alpha_position: float = 0.85
    alpha_orientation: float = 0.85


@dataclass
class IKConfig:
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    posture_cost: float = 1e-2
    damping: float = 1e-12
    soft_sync_alpha: float = 0.05
    impedance_lookahead: float = 0.15

    def __post_init__(self):
        self.position_cost = float(self.position_cost)
        self.orientation_cost = float(self.orientation_cost)
        self.posture_cost = float(self.posture_cost)
        self.damping = float(self.damping)
        self.soft_sync_alpha = float(self.soft_sync_alpha)
        self.impedance_lookahead = float(self.impedance_lookahead)


@dataclass
class WorkspaceConfig:
    x: List[float] = field(default_factory=lambda: [-0.8, 0.8])
    y: List[float] = field(default_factory=lambda: [-0.8, 0.8])
    z: List[float] = field(default_factory=lambda: [0.05, 1.2])


@dataclass
class SafetyConfig:
    packet_timeout_ms: int = 100  # tighter than admittance
    max_joint_vel: float = 1.0
    max_position_deviation: float = 0.3  # rad
    max_ee_velocity: float = 0.2
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)


@dataclass
class ImpedanceControlConfig:
    default_preset: str = "SOFT"
    gain_scale_range: List[float] = field(default_factory=lambda: [0.25, 2.0])
    enable_coriolis_comp: bool = True
    max_joint_error: List[float] = field(
        default_factory=lambda: [0.10, 0.10, 0.12, 0.18, 0.18, 0.2]
    )


@dataclass
class ImpedanceConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    input: InputConfig = field(default_factory=InputConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    impedance: ImpedanceControlConfig = field(default_factory=ImpedanceControlConfig)

    # From robot.config (read-only)
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
    def load(cls, path: Optional[str] = None) -> "ImpedanceConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[ImpedanceConfig] No config at {config_path}, using defaults")
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
        if "impedance" in data:
            cfg.impedance = ImpedanceControlConfig(**data["impedance"])

        return cfg
