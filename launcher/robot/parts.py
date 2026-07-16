"""robot.yaml 로더 — 논리 파츠(팔/손/캠) 정의 + 컴포넌트 선택.

기존 LauncherConfig.load (components/setups/${var}/topo 검증) 를 재사용하고,
같은 yaml 의 robot 런처 전용 섹션 (parts / estop / move_home) 을 추가 파싱.

파츠(part) = 웹 카드와 CLI --side 선택의 단위. 예: hand-right 파츠는
dg5f-right-driver + hand-right-receiver 두 컴포넌트로 구성.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from launcher.config import LauncherConfig, _substitute

_DEFAULT_CONFIG = Path(__file__).parent / "config" / "robot.yaml"

VALID_KINDS = ("arm", "hand", "cam")


@dataclass
class PartSpec:
    name: str                       # arm-right, hand-left, cam ...
    label: str                      # 표시명 (오른팔 등)
    kind: str                       # arm | hand | cam
    side: str = ""                  # right | left | "" (cam)
    components: List[str] = field(default_factory=list)
    probe: Optional[Tuple[str, int]] = None      # (host, port) TCP 도달성 검사
    params_file: str = ""           # 파라미터 페이지가 편집할 yaml (teleop_dir 기준 상대)
    params_schema: str = ""         # "arm" | "cam" | ""


@dataclass
class EstopCfg:
    burst_hz: int = 200
    burst_duration_s: float = 0.5
    fallback_arm_ports: Dict[str, int] = field(
        default_factory=lambda: {"right": 9871, "left": 9875})


@dataclass
class MoveHomeCfg:
    config: str = ""                # 팔 yaml (teleop_dir 기준 상대)
    robot_ip: str = ""
    speed: float = 0.3              # rad/s
    accel: float = 0.3              # rad/s^2


@dataclass
class RobotConfig:
    launcher: LauncherConfig
    parts: List[PartSpec] = field(default_factory=list)
    estop: EstopCfg = field(default_factory=EstopCfg)
    move_home: Dict[str, MoveHomeCfg] = field(default_factory=dict)
    teleop_dir: str = ""
    config_path: str = ""

    def part(self, name: str) -> PartSpec:
        for p in self.parts:
            if p.name == name:
                return p
        raise KeyError(f"part {name!r} 없음")

    def part_of_component(self, comp_name: str) -> Optional[PartSpec]:
        for p in self.parts:
            if comp_name in p.components:
                return p
        return None

    def resolve_path(self, rel: str) -> str:
        """teleop_dir 기준 상대 경로 → 절대 경로."""
        p = Path(rel)
        if p.is_absolute():
            return str(p)
        return str(Path(self.teleop_dir) / p)


def _build_variables(raw: dict) -> Dict[str, str]:
    """LauncherConfig.load 와 동일한 ${var} 소스 (network+vars, 반복 해석)."""
    variables: Dict[str, str] = {}
    for section in ("network", "vars"):
        for k, v in (raw.get(section) or {}).items():
            variables[str(k)] = str(v)
    for _ in range(5):
        changed = False
        for k, v in variables.items():
            nv = _substitute(v, variables)
            if nv != v:
                variables[k] = nv
                changed = True
        if not changed:
            break
    return variables


def load_robot_config(path: Optional[str] = None) -> RobotConfig:
    config_path = str(path or _DEFAULT_CONFIG)
    launcher_cfg = LauncherConfig.load(config_path)

    raw = yaml.safe_load(Path(config_path).read_text()) or {}
    variables = _build_variables(raw)

    def sub(v: str) -> str:
        return _substitute(str(v), variables)

    teleop_dir = variables.get("teleop_dir", str(Path(__file__).parents[2]))

    comp_names = {c.name for c in launcher_cfg.components}
    parts: List[PartSpec] = []
    seen = set()
    for p in raw.get("parts") or []:
        name = str(p["name"])
        if name in seen:
            raise ValueError(f"part 이름 중복: {name!r}")
        seen.add(name)
        kind = str(p.get("kind", ""))
        if kind not in VALID_KINDS:
            raise ValueError(f"part {name!r}: kind 는 {VALID_KINDS} 중 하나 ({kind!r})")
        components = [str(c) for c in (p.get("components") or [])]
        for c in components:
            if c not in comp_names:
                raise ValueError(f"part {name!r}: 알 수 없는 component {c!r}")
        probe = None
        if p.get("probe"):
            probe = (sub(p["probe"]["host"]), int(sub(str(p["probe"]["port"]))))
        params = p.get("params") or {}
        parts.append(PartSpec(
            name=name,
            label=str(p.get("label", name)),
            kind=kind,
            side=str(p.get("side", "")),
            components=components,
            probe=probe,
            params_file=sub(params.get("file", "")),
            params_schema=str(params.get("schema", "")),
        ))

    es_raw = raw.get("estop") or {}
    estop = EstopCfg(
        burst_hz=int(es_raw.get("burst_hz", 200)),
        burst_duration_s=float(es_raw.get("burst_duration_s", 0.5)),
        fallback_arm_ports={
            str(k): int(v) for k, v in
            (es_raw.get("fallback_arm_ports") or {"right": 9871, "left": 9875}).items()
        },
    )

    move_home: Dict[str, MoveHomeCfg] = {}
    for side, m in (raw.get("move_home") or {}).items():
        move_home[str(side)] = MoveHomeCfg(
            config=sub(m.get("config", "")),
            robot_ip=sub(m.get("robot_ip", "")),
            speed=float(m.get("speed", 0.3)),
            accel=float(m.get("accel", 0.3)),
        )

    return RobotConfig(launcher=launcher_cfg, parts=parts, estop=estop,
                       move_home=move_home, teleop_dir=teleop_dir,
                       config_path=config_path)


def select_parts(cfg: RobotConfig, side: str = "both",
                 no_arms: bool = False, no_hands: bool = False,
                 no_cam: bool = False,
                 only: Optional[List[str]] = None) -> List[PartSpec]:
    """CLI/웹의 파츠 선택 로직.

    only 지정 시 side/no-* 무시하고 이름으로 직접 선택.
    side: right|left|both — side 가 있는 파츠(팔/손)에만 적용, cam 은 항상 포함.
    """
    if only:
        by_name = {p.name: p for p in cfg.parts}
        missing = [n for n in only if n not in by_name]
        if missing:
            raise KeyError(f"알 수 없는 파츠: {missing} (가능: {sorted(by_name)})")
        return [by_name[n] for n in only]

    out = []
    for p in cfg.parts:
        if p.side and side != "both" and p.side != side:
            continue
        if p.kind == "arm" and no_arms:
            continue
        if p.kind == "hand" and no_hands:
            continue
        if p.kind == "cam" and no_cam:
            continue
        out.append(p)
    return out


def components_for(parts: List[PartSpec]) -> List[str]:
    out: List[str] = []
    for p in parts:
        for c in p.components:
            if c not in out:
                out.append(c)
    return out
