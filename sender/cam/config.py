"""sender/cam 설정 — YAML loader with typed access.

zmq.cameras 순서가 곧 cam index (WS binary 헤더의 cam_index) 이고,
vr.yaw_deg 의 인덱스와도 1:1 대응한다. 로봇 측 robot/cam config 의
cameras[].name 과 이름이 일치해야 구독이 성립한다.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml

from protocol.cam_protocol import DEFAULT_CAM_HTTP_PORT, DEFAULT_CAM_ZMQ_PORT

_DEFAULT_CONFIG = Path(__file__).parent / "config.yaml"


@dataclass
class ZmqConfig:
    robot_ip: str = "192.168.0.2"
    port: int = DEFAULT_CAM_ZMQ_PORT          # 9873
    cameras: List[str] = field(default_factory=lambda: ["head"])


@dataclass
class HttpConfig:
    port: int = DEFAULT_CAM_HTTP_PORT         # 8014 — 헤드셋 USB 모드: adb reverse 필요
    ws_max_fps: int = 30                      # WS 클라이언트당 push 상한


@dataclass
class VrConfig:
    mode: str = "head_locked"                 # head_locked | world_locked (화면 고정)
    plane_distance_m: float = 1.0
    plane_width_m: float = 1.06               # 640x480 → 4:3
    plane_height_m: float = 0.8
    plane_height_offset_m: float = 0.0
    yaw_deg: List[float] = field(default_factory=lambda: [0.0, -40.0, 40.0])


@dataclass
class SenderCamConfig:
    zmq: ZmqConfig = field(default_factory=ZmqConfig)
    http: HttpConfig = field(default_factory=HttpConfig)
    vr: VrConfig = field(default_factory=VrConfig)

    @classmethod
    def load(cls, path: Optional[str] = None) -> "SenderCamConfig":
        config_path = Path(path) if path else _DEFAULT_CONFIG
        if not config_path.exists():
            print(f"[SenderCamConfig] No config at {config_path}, using defaults")
            return cls()

        with open(config_path) as f:
            data = yaml.safe_load(f) or {}

        cfg = cls()
        if "zmq" in data:
            cfg.zmq = ZmqConfig(**data["zmq"])
        if "http" in data:
            cfg.http = HttpConfig(**data["http"])
        if "vr" in data:
            cfg.vr = VrConfig(**data["vr"])
        return cfg
