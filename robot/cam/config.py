"""robot/cam 설정 — YAML loader with typed access.

sender/hand/realsense/config.py 와 같은 dataclass + YAML + CLI override 패턴.
카메라 이름(name)이 그대로 ZMQ topic 이 되므로 operator 측
sender/cam/config.yaml 의 zmq.cameras 리스트와 일치해야 한다.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml

from protocol.cam_protocol import DEFAULT_CAM_ZMQ_PORT

_DEFAULT_CONFIG = Path(__file__).parent / "config" / "default.yaml"


@dataclass
class StreamConfig:
    bind_host: str = "0.0.0.0"
    port: int = DEFAULT_CAM_ZMQ_PORT   # 9873
    jpeg_quality: int = 80             # 1-100


@dataclass
class CameraEntry:
    name: str = "head"     # ZMQ topic. operator 측 cameras 리스트와 일치 필요
    serial: str = ""       # "" = 첫 번째 연결 장치 (robot.cam.list_cameras 로 확인)
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass
class RobotCamConfig:
    stream: StreamConfig = field(default_factory=StreamConfig)
    cameras: List[CameraEntry] = field(default_factory=lambda: [CameraEntry()])

    @classmethod
    def load(cls, path: Optional[str] = None) -> "RobotCamConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[RobotCamConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()
        if "stream" in data:
            cfg.stream = StreamConfig(**data["stream"])
        if "cameras" in data:
            cfg.cameras = [CameraEntry(**c) for c in data["cameras"]]
        return cfg
