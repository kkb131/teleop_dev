"""sender/cam 설정 — YAML loader with typed access.

zmq.cameras 순서가 곧 cam index (WS binary 헤더의 cam_index) 이고,
vr.yaw_deg 의 인덱스와도 1:1 대응한다. 로봇 측 robot/cam config 의
cameras[].name 과 이름이 일치해야 구독이 성립한다.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

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
class PlaneConfig:
    """카메라 1대의 VR plane 배치 override — None 필드는 VrConfig 전역값 사용."""
    yaw_deg: Optional[float] = None           # 좌우 각도 (+=왼쪽)
    pitch_deg: Optional[float] = None         # 상하 각도 (+=위)
    distance_m: Optional[float] = None
    width_m: Optional[float] = None
    height_m: Optional[float] = None
    height_offset_m: Optional[float] = None


@dataclass
class VrConfig:
    mode: str = "head_locked"                 # head_locked | world_locked (화면 고정)
    plane_distance_m: float = 1.0
    plane_width_m: float = 1.06               # 640x480 → 4:3
    plane_height_m: float = 0.8
    plane_height_offset_m: float = 0.0
    yaw_deg: List[float] = field(default_factory=lambda: [0.0, -40.0, 40.0])
    planes: Dict[str, PlaneConfig] = field(default_factory=dict)  # key = 카메라 이름

    def resolve_plane(self, name: str, index: int) -> dict:
        """카메라별 최종 배치값 — planes[name] override + 전역/legacy fallback."""
        p = self.planes.get(name)
        yaw_default = self.yaw_deg[index] if index < len(self.yaw_deg) else 0.0
        return {
            "yaw_deg":         p.yaw_deg         if p and p.yaw_deg         is not None else yaw_default,
            "pitch_deg":       p.pitch_deg       if p and p.pitch_deg       is not None else 0.0,
            "distance_m":      p.distance_m      if p and p.distance_m      is not None else self.plane_distance_m,
            "width_m":         p.width_m         if p and p.width_m         is not None else self.plane_width_m,
            "height_m":        p.height_m        if p and p.height_m        is not None else self.plane_height_m,
            "height_offset_m": p.height_offset_m if p and p.height_offset_m is not None else self.plane_height_offset_m,
        }


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
            vr_data = dict(data["vr"])
            planes_data = vr_data.pop("planes", None) or {}
            cfg.vr = VrConfig(**vr_data)
            cfg.vr.planes = {
                name: PlaneConfig(**(d or {})) for name, d in planes_data.items()
            }
        return cfg
