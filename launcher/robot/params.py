"""로봇 PC 파라미터 스키마 — 편집 엔진은 launcher/params_engine.py (공용).

- 팔: robot/arm/admittance/config/{right,left}.yaml (ARM_SCHEMA)
- 캠: robot/cam/config/default.yaml (CAM_SCHEMA)

get_params/set_params 는 스키마 이름("arm"|"cam") 기반 thin wrapper —
robot api.py 의 기존 호출 시그니처 유지.
"""

from __future__ import annotations

from typing import Any, Dict, List

from launcher.params_engine import (   # noqa: F401 — re-export (기존 import 호환)
    ParamField,
    SaveResult,
    _get_dotted,
    _set_dotted,
)
from launcher import params_engine

ARM_SCHEMA: List[ParamField] = [
    ParamField("robot.ip", "str", "로봇 IP", help="UR10e 주소. 변경 시 네트워크 확인"),
    ParamField("input.unified_port", "int", "UDP 수신 포트", min=1024, max=65535,
               help="sender(xr_dual.yaml)의 해당 팔 port 와 일치해야 함"),
    ParamField("filter.alpha_position", "float", "위치 필터 α", min=0.01, max=1.0,
               help="높을수록 반응 빠름 (1=필터 없음)"),
    ParamField("filter.alpha_orientation", "float", "자세 필터 α", min=0.01, max=1.0),
    ParamField("safety.max_joint_vel", "float", "관절 속도 한계", unit="rad/s",
               min=0.05, max=1.0, danger=True),
    ParamField("safety.max_ee_velocity", "float", "EE 속도 한계", unit="m/s",
               min=0.01, max=0.5, danger=True),
    ParamField("safety.workspace.x", "range2", "workspace X", unit="m", danger=True),
    ParamField("safety.workspace.y", "range2", "workspace Y", unit="m", danger=True),
    ParamField("safety.workspace.z", "range2", "workspace Z", unit="m", danger=True),
    ParamField("admittance.default_preset", "enum", "어드미턴스 프리셋",
               choices=["STIFF", "MEDIUM", "SOFT", "FREE"]),
    ParamField("admittance.max_displacement_trans", "float", "어드미턴스 최대 변위",
               unit="m", min=0.0, max=0.3),
    ParamField("admittance.max_displacement_rot", "float", "어드미턴스 최대 회전",
               unit="rad", min=0.0, max=0.6),
    ParamField("initial_pose.enabled", "bool", "초기 자세 이동 사용", danger=True,
               help="false = home 미실측 (move_home 거부됨)"),
    ParamField("initial_pose.joint_values", "joints6", "초기 자세 joints", unit="rad",
               danger=True),
    ParamField("initial_pose.move_duration_s", "float", "초기 이동 시간", unit="s",
               min=0.5, max=10.0),
]

CAM_SCHEMA: List[ParamField] = [
    ParamField("stream.port", "int", "ZMQ PUB 포트", min=1024, max=65535,
               help="조종측 sender.cam 의 zmq.port 와 일치해야 함"),
    ParamField("stream.jpeg_quality", "int", "JPEG 품질", min=1, max=100),
    ParamField("cameras", "cam_list", "카메라 목록",
               help="name/serial(빈값=자동)/width/height/fps 행 편집"),
]

SCHEMAS: Dict[str, List[ParamField]] = {"arm": ARM_SCHEMA, "cam": CAM_SCHEMA}


def get_params(file_path: str, schema_name: str) -> dict:
    return params_engine.get_params(file_path, SCHEMAS[schema_name])


def set_params(file_path: str, schema_name: str,
               values: Dict[str, Any]) -> SaveResult:
    return params_engine.set_params(file_path, SCHEMAS[schema_name], values)
