"""sender.yaml 로더 — 러너 슬롯 / adb / 세션 / cam 정의.

기존 LauncherConfig.load (components/${var}/web 섹션) 재사용 + sender 전용
섹션 (runners / adb / session / cam / params) 추가 파싱.

러너(runner) = 상호배타 실행 슬롯. xr-dual / xr-single-right / xr-single-left
는 모두 BridgePoseStore(8013) 와 UDP 포트를 사용하므로 동시에 하나만 실행.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from launcher.config import LauncherConfig, _substitute

_DEFAULT_CONFIG = Path(__file__).parent / "config" / "sender.yaml"


@dataclass
class RunnerSpec:
    name: str                 # xr-dual / xr-single-right / xr-single-left
    label: str
    component: str            # components 항목 이름
    kind: str = "dual"        # dual | single
    side: str = ""            # single 일 때 right|left


@dataclass
class AdbCfg:
    binary: str = "adb"
    reverse_ports: List[int] = field(default_factory=lambda: [8013, 8014, 9877])
    probe_interval_s: float = 3.0


@dataclass
class SessionCfg:
    default_mode: str = "xr-dual"
    countdown_s: float = 4.0
    start_timeout_s: float = 15.0
    confirm_timeout_s: float = 6.0     # query_initial_pose 5s fallback 커버 (≥6 유지)
    retry_delay_s: float = 2.0
    max_retries: int = 10


@dataclass
class SenderConfig:
    launcher: LauncherConfig
    runners: List[RunnerSpec] = field(default_factory=list)
    adb: AdbCfg = field(default_factory=AdbCfg)
    session: SessionCfg = field(default_factory=SessionCfg)
    cam_component: str = ""            # cam viewer 컴포넌트 이름 ("" = 없음)
    cam_http_port: int = 8014
    bridge_port: int = 8013
    params: Dict[str, dict] = field(default_factory=dict)  # target → {file, schema}
    robot_pc_ip: str = ""
    teleop_dir: str = ""
    config_path: str = ""

    def runner(self, name: str) -> RunnerSpec:
        for r in self.runners:
            if r.name == name:
                return r
        raise KeyError(f"runner {name!r} 없음 (가능: {[r.name for r in self.runners]})")

    def resolve_path(self, rel: str) -> str:
        p = Path(rel)
        if p.is_absolute():
            return str(p)
        return str(Path(self.teleop_dir) / p)


def _build_variables(raw: dict) -> Dict[str, str]:
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


def load_sender_config(path: Optional[str] = None) -> SenderConfig:
    config_path = str(path or _DEFAULT_CONFIG)
    launcher_cfg = LauncherConfig.load(config_path)

    raw = yaml.safe_load(Path(config_path).read_text()) or {}
    variables = _build_variables(raw)

    def sub(v) -> str:
        return _substitute(str(v), variables)

    teleop_dir = variables.get("teleop_dir", str(Path(__file__).parents[2]))
    comp_names = {c.name for c in launcher_cfg.components}

    runners: List[RunnerSpec] = []
    seen = set()
    for r in raw.get("runners") or []:
        name = str(r["name"])
        if name in seen:
            raise ValueError(f"runner 이름 중복: {name!r}")
        seen.add(name)
        component = str(r["component"])
        if component not in comp_names:
            raise ValueError(f"runner {name!r}: 알 수 없는 component {component!r}")
        spec = launcher_cfg.get(component)
        if not spec.use_pty:
            raise ValueError(
                f"runner {name!r}: component {component!r} 는 use_pty: true 필요 "
                "(웹 키패드/자동 캘리가 pty 로 키를 전달)")
        runners.append(RunnerSpec(
            name=name,
            label=str(r.get("label", name)),
            component=component,
            kind=str(r.get("kind", "dual")),
            side=str(r.get("side", "")),
        ))

    adb_raw = raw.get("adb") or {}
    adb = AdbCfg(
        binary=str(adb_raw.get("binary", "adb")),
        reverse_ports=[int(p) for p in
                       (adb_raw.get("reverse_ports") or [8013, 8014, 9877])],
        probe_interval_s=float(adb_raw.get("probe_interval_s", 3.0)),
    )

    ses_raw = raw.get("session") or {}
    session = SessionCfg(
        default_mode=str(ses_raw.get("default_mode", "xr-dual")),
        countdown_s=float(ses_raw.get("countdown_s", 4.0)),
        start_timeout_s=float(ses_raw.get("start_timeout_s", 15.0)),
        confirm_timeout_s=float(ses_raw.get("confirm_timeout_s", 6.0)),
        retry_delay_s=float(ses_raw.get("retry_delay_s", 2.0)),
        max_retries=int(ses_raw.get("max_retries", 10)),
    )

    cam_raw = raw.get("cam") or {}
    cam_component = str(cam_raw.get("component", ""))
    if cam_component and cam_component not in comp_names:
        raise ValueError(f"cam.component {cam_component!r} 없음")

    params: Dict[str, dict] = {}
    for target, p in (raw.get("params") or {}).items():
        params[str(target)] = {
            "file": sub(p["file"]),
            "schema": str(p.get("schema", "")),
        }

    cfg = SenderConfig(
        launcher=launcher_cfg,
        runners=runners,
        adb=adb,
        session=session,
        cam_component=cam_component,
        cam_http_port=int(cam_raw.get("http_port", 8014)),
        bridge_port=int((raw.get("bridge") or {}).get("port", 8013)),
        params=params,
        robot_pc_ip=variables.get("robot_pc_ip", ""),
        teleop_dir=teleop_dir,
        config_path=config_path,
    )
    if session.default_mode and not any(r.name == session.default_mode
                                        for r in cfg.runners):
        raise ValueError(f"session.default_mode {session.default_mode!r} 가 runners 에 없음")
    return cfg
