"""런처 설정 로더 — 구성요소를 데이터(yaml)로 정의.

스키마 (launcher/robot/config/robot.yaml 참조):

    network:            # ${var} 치환 소스 (IP/포트의 단일 소스)
      right_robot_ip: "192.168.0.2"
      ...
    vars:               # 추가 치환 변수 (network 와 병합)
      teleop_dir: "/workspaces/tamp_ws/src/teleop_dev"
    setups:             # 이름 붙은 shell 스니펫 — 컴포넌트가 setup: [이름] 으로 참조
      ros: "source /opt/ros/humble/setup.bash && ..."
    web:                # 웹 대시보드 (launcher.robot.web) 바인딩
      host: "0.0.0.0"
      port: 9876
      token: ""         # 비우면 인증 없음 (신뢰 네트워크 전용)
      tls_cert: ""      # tls_cert+tls_key 지정 시 HTTPS
      tls_key: ""
    defaults:
      stop_grace_s: 5.0
      start_delay_s: 0.0
    components:
      - name: dg5f-right-driver
        group: robot            # robot | operator
        setup: [ros]
        cwd: "${teleop_dir}"
        command: "ros2 launch ..."
        depends_on: []          # start_all 시 위상 정렬 순서
        start_delay_s: 3.0      # 시작 후 다음 컴포넌트까지 대기 (드라이버 초기화)
        stop_grace_s: 5.0
        use_pty: false          # termios 필요한 프로세스 (키 입력 전달 가능)
        oneshot: false          # true 면 정상 종료(rc=0)가 완료 상태
        env: {}

parts/estop/move_home 등 robot 런처 전용 섹션은 launcher.robot.parts 가
같은 yaml 에서 추가로 파싱한다 (이 로더는 그대로 통과).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

_VAR_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


@dataclass
class ComponentSpec:
    name: str
    group: str = "robot"                 # "robot" | "operator"
    command: str = ""
    setup: List[str] = field(default_factory=list)   # setups 스니펫 이름들
    cwd: Optional[str] = None
    env: Dict[str, str] = field(default_factory=dict)
    depends_on: List[str] = field(default_factory=list)
    start_delay_s: float = 0.0
    stop_grace_s: float = 5.0
    use_pty: bool = False
    oneshot: bool = False

    def build_argv(self, setups: Dict[str, str]) -> List[str]:
        """setup 스니펫 + command 를 bash -c 한 줄로 조립.

        마지막 command 는 exec 로 실행해 bash 가 프로세스를 대체 —
        killpg(SIGINT) 시맨틱이 대상 프로세스에 그대로 적용된다.
        (단 command 에 쉘 연산자가 있으면 exec 생략)
        """
        parts = []
        for s in self.setup:
            if s not in setups:
                raise ValueError(f"component {self.name!r}: 알 수 없는 setup 스니펫 {s!r}")
            parts.append(setups[s])
        cmd = self.command.strip()
        if not any(op in cmd for op in ("&&", "||", ";", "|", ">", "<")):
            cmd = "exec " + cmd
        parts.append(cmd)
        return ["bash", "-c", " && ".join(parts)]


@dataclass
class WebConfig:
    """웹 대시보드 (launcher.robot.web) 바인딩/보안 설정 — yaml `web:` 섹션."""
    host: str = "0.0.0.0"
    port: int = 9876
    token: str = ""          # 비우면 인증 없음 (신뢰 LAN 전용)
    tls_cert: str = ""       # tls_cert + tls_key 둘 다 지정 시 HTTPS
    tls_key: str = ""


@dataclass
class LauncherConfig:
    network: Dict[str, str] = field(default_factory=dict)
    setups: Dict[str, str] = field(default_factory=dict)
    web: WebConfig = field(default_factory=WebConfig)
    components: List[ComponentSpec] = field(default_factory=list)

    def by_group(self, group: str) -> List[ComponentSpec]:
        return [c for c in self.components if c.group == group]

    def get(self, name: str) -> ComponentSpec:
        for c in self.components:
            if c.name == name:
                return c
        raise KeyError(f"component {name!r} 없음")

    @classmethod
    def load(cls, path: str) -> "LauncherConfig":
        raw = yaml.safe_load(Path(path).read_text()) or {}

        # ${var} 치환 소스: network + vars 병합, 상호 참조 위해 반복 해석 (max 5)
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

        def sub(value):
            if isinstance(value, str):
                return _substitute(value, variables)
            if isinstance(value, dict):
                return {k: sub(v) for k, v in value.items()}
            if isinstance(value, list):
                return [sub(v) for v in value]
            return value

        setups = {str(k): sub(str(v)) for k, v in (raw.get("setups") or {}).items()}

        web_raw = raw.get("web") or {}
        web = WebConfig(
            host=str(web_raw.get("host", "0.0.0.0")),
            port=int(sub(str(web_raw.get("port", variables.get("web_port", 9876))))),
            token=str(web_raw.get("token", "")),
            tls_cert=sub(str(web_raw.get("tls_cert", ""))),
            tls_key=sub(str(web_raw.get("tls_key", ""))),
        )

        defaults = raw.get("defaults") or {}
        d_grace = float(defaults.get("stop_grace_s", 5.0))
        d_delay = float(defaults.get("start_delay_s", 0.0))

        components: List[ComponentSpec] = []
        seen = set()
        for c in raw.get("components") or []:
            name = str(c["name"])
            if name in seen:
                raise ValueError(f"component 이름 중복: {name!r}")
            seen.add(name)
            group = str(c.get("group", "robot"))
            if group not in ("robot", "operator"):
                raise ValueError(f"component {name!r}: group 은 robot|operator ({group!r})")
            components.append(ComponentSpec(
                name=name,
                group=group,
                command=sub(str(c.get("command", ""))),
                setup=[str(s) for s in (c.get("setup") or [])],
                cwd=sub(c["cwd"]) if c.get("cwd") else None,
                env={str(k): sub(str(v)) for k, v in (c.get("env") or {}).items()},
                depends_on=[str(d) for d in (c.get("depends_on") or [])],
                start_delay_s=float(c.get("start_delay_s", d_delay)),
                stop_grace_s=float(c.get("stop_grace_s", d_grace)),
                use_pty=bool(c.get("use_pty", False)),
                oneshot=bool(c.get("oneshot", False)),
            ))

        cfg = cls(network={k: str(v) for k, v in (raw.get("network") or {}).items()},
                  setups=setups, web=web, components=components)

        # depends_on 대상 존재 + cycle 검증 (로드 시점에 실패)
        names = {c.name for c in cfg.components}
        for c in cfg.components:
            for d in c.depends_on:
                if d not in names:
                    raise ValueError(f"component {c.name!r}: depends_on {d!r} 없음")
        topo_order(cfg.components)
        return cfg


def _substitute(text: str, variables: Dict[str, str]) -> str:
    def repl(m):
        key = m.group(1)
        if key not in variables:
            raise ValueError(f"치환 변수 ${{{key}}} 가 network/vars 에 없음")
        return variables[key]
    return _VAR_RE.sub(repl, text)


def topo_order(components: List[ComponentSpec]) -> List[ComponentSpec]:
    """Kahn 위상 정렬 — depends_on 이 먼저 오도록. cycle 이면 ValueError."""
    by_name = {c.name: c for c in components}
    indeg = {c.name: 0 for c in components}
    for c in components:
        for d in c.depends_on:
            if d in indeg:
                indeg[c.name] += 1
    queue = [n for n, deg in sorted(indeg.items()) if deg == 0]
    out: List[ComponentSpec] = []
    while queue:
        n = queue.pop(0)
        out.append(by_name[n])
        for c in components:
            if n in c.depends_on:
                indeg[c.name] -= 1
                if indeg[c.name] == 0:
                    queue.append(c.name)
    if len(out) != len(components):
        remaining = set(by_name) - {c.name for c in out}
        raise ValueError(f"depends_on cycle: {sorted(remaining)}")
    return out
