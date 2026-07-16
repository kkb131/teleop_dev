"""LocalManager — 이 머신에서 도는 컴포넌트들의 관리자.

bringup CLI 와 웹 대시보드가 같은 인터페이스로 사용:
    names() / spec(name) / start(name) / stop(name) / status(name)
    / log_since(name, seq) / send_input(name, text)
    / start_all() / stop_all()
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

from launcher.config import ComponentSpec, LauncherConfig, topo_order
from launcher.process import ManagedProcess


class LocalManager:
    def __init__(self, config: LauncherConfig, groups: Optional[List[str]] = None):
        """groups: 이 머신이 담당할 group 목록 (예: ["robot"]). None = 전부."""
        self._config = config
        specs = [c for c in config.components
                 if groups is None or c.group in groups]
        self._procs: Dict[str, ManagedProcess] = {
            c.name: ManagedProcess(c, config.setups) for c in specs
        }

    # ── 단건 조작 ────────────────────────────────────────────────────────

    def names(self) -> List[str]:
        return list(self._procs.keys())

    def spec(self, name: str) -> ComponentSpec:
        return self._procs[name].spec

    def start(self, name: str) -> dict:
        self._procs[name].start()
        return self.status(name)

    def stop(self, name: str) -> dict:
        self._procs[name].stop()
        return self.status(name)

    def status(self, name: str) -> dict:
        return self._procs[name].status()

    def status_all(self) -> List[dict]:
        return [p.status() for p in self._procs.values()]

    def log_since(self, name: str, since: int = 0,
                  max_lines: int = 200) -> List[Tuple[int, str]]:
        return self._procs[name].log_since(since, max_lines)

    def send_input(self, name: str, text: str) -> bool:
        return self._procs[name].send_input(text)

    # ── 일괄 조작 ────────────────────────────────────────────────────────

    def start_all(self, names: Optional[List[str]] = None) -> List[str]:
        """depends_on 위상 정렬 순서로 시작. start_delay_s 존중.

        names: 시작할 부분집합 (None = 전부). 반환: 시작한 이름 순서.
        """
        selected = set(names if names is not None else self._procs.keys())
        ordered = [c for c in topo_order([p.spec for p in self._procs.values()])
                   if c.name in selected]
        started = []
        for spec in ordered:
            proc = self._procs[spec.name]
            if not proc.is_running():
                proc.start()
                started.append(spec.name)
                if spec.start_delay_s > 0:
                    time.sleep(spec.start_delay_s)
        return started

    def stop_all(self, names: Optional[List[str]] = None) -> List[str]:
        """역-위상 순서로 정지 (sender 먼저, 드라이버 나중)."""
        selected = set(names if names is not None else self._procs.keys())
        ordered = [c for c in topo_order([p.spec for p in self._procs.values()])
                   if c.name in selected]
        stopped = []
        for spec in reversed(ordered):
            proc = self._procs[spec.name]
            if proc.is_running():
                proc.stop()
                stopped.append(spec.name)
        return stopped
