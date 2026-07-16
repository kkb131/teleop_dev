"""조종 PC 파라미터 스키마 — 대상: scripts/config/xr_dual.yaml.

편집 엔진은 launcher/params_engine.py (robot 측과 공유).
gestures 섹션은 C6 커밋에서 xr_dual.yaml 에 추가된다.
"""

from __future__ import annotations

from typing import Dict, List

from launcher.params_engine import ParamField


def _arm_fields(side: str) -> List[ParamField]:
    label = "오른팔" if side == "right" else "왼팔"
    return [
        ParamField(f"arms.{side}.enabled", "bool", f"{label} 사용"),
        ParamField(f"arms.{side}.port", "int", f"{label} UDP 포트",
                   min=1024, max=65535,
                   help="robot 측 해당 팔 input.unified_port 와 일치해야 함"),
        ParamField(f"arms.{side}.hz", "int", f"{label} 송신 Hz", min=10, max=100),
        ParamField(f"arms.{side}.scale", "float", f"{label} 위치 scale",
                   min=0.05, max=1.0, danger=True,
                   help="손 이동 → 로봇 이동 배율 (미검증 팔은 0.3 이하 권장)"),
        ParamField(f"arms.{side}.remap_rpy_deg", "floats3",
                   f"{label} remap RPY", unit="deg", danger=True,
                   help="WebXR→base_link 축 정렬 — xr_dual_arm_left_tuning_ko.md 참조"),
        ParamField(f"arms.{side}.workspace.x", "range2", f"{label} workspace X",
                   unit="m", danger=True),
        ParamField(f"arms.{side}.workspace.y", "range2", f"{label} workspace Y",
                   unit="m", danger=True),
        ParamField(f"arms.{side}.workspace.z", "range2", f"{label} workspace Z",
                   unit="m", danger=True),
    ]


def _hand_fields(side: str) -> List[ParamField]:
    label = "오른손" if side == "right" else "왼손"
    return [
        ParamField(f"hands.{side}.enabled", "bool", f"{label} 사용"),
        ParamField(f"hands.{side}.port", "int", f"{label} UDP 포트",
                   min=1024, max=65535),
        ParamField(f"hands.{side}.hz", "int", f"{label} 송신 Hz", min=10, max=120),
        ParamField(f"hands.{side}.convention", "enum", f"{label} convention",
                   choices=["mediapipe", "manus"],
                   help="오므림↔벌림 반전 시 manus 로 토글"),
    ]


XR_DUAL_SCHEMA: List[ParamField] = [
    ParamField("network.robot_pc_ip", "str", "robot PC IP",
               help="sender.yaml 의 robot_pc_ip 와 불일치 시 대시보드에 경고 표시"),
    ParamField("safety.watchdog_timeout_s", "float", "watchdog timeout", unit="s",
               min=0.05, max=2.0, danger=True,
               help="헤드셋 데이터 stale 판정 시간"),
    ParamField("safety.enforce_workspace", "bool", "workspace clamp 사용", danger=True),
    *_arm_fields("right"),
    *_arm_fields("left"),
    *_hand_fields("right"),
    *_hand_fields("left"),
    ParamField("gestures.enabled", "bool", "제스처 명령 사용", danger=True,
               help="양손 동시 pinch→캘리(r) / squeeze→일시정지(p). "
                    "파지 중 오발동 위험 — 위험 고지 숙지 후 사용"),
    ParamField("gestures.hold_s", "float", "제스처 유지 시간", unit="s",
               min=0.3, max=5.0),
    ParamField("gestures.refractory_s", "float", "제스처 불응기", unit="s",
               min=0.5, max=10.0),
]

SCHEMAS: Dict[str, List[ParamField]] = {"xr_dual": XR_DUAL_SCHEMA}
