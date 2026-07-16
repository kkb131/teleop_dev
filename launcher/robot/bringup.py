#!/usr/bin/env python3
"""로봇 PC 브링업 CLI — 팔/손/캠 구성요소를 터미널에서 일괄 기동.

웹 대시보드(launcher.robot.web) 없이 터미널만으로 운용할 때 사용.
(웹 데몬과 동시 실행 불가 — flock 상호배제)

사용 예:
    # 양팔 + 양손 + 캠 전체
    python3 -m launcher.robot.bringup

    # 오른쪽만 (오른팔 + 오른손), 캠 제외
    python3 -m launcher.robot.bringup --side right --no-cam

    # 파츠 직접 지정
    python3 -m launcher.robot.bringup --parts arm-left cam

    # 선택 결과 / 실행될 명령 확인만
    python3 -m launcher.robot.bringup --side left --list
    python3 -m launcher.robot.bringup --dry-run

종료: Ctrl+C → 역-위상 순서로 전체 정지 (수신부 먼저, 드라이버 나중).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List

from launcher.config import topo_order
from launcher.manager import LocalManager
from launcher.robot.lock import LauncherLock, acquire_or_exit_message
from launcher.robot.parts import (
    RobotConfig,
    components_for,
    load_robot_config,
    select_parts,
)


def _print_selection(cfg: RobotConfig, parts, components: List[str]) -> None:
    print("=" * 62)
    print("  Robot Bringup")
    print(f"  config: {cfg.config_path}")
    for p in parts:
        print(f"  [{p.kind:<4s}] {p.label:<8s} ({p.name}) → {', '.join(p.components)}")
    print(f"  시작 순서(topo): {' → '.join(components)}")
    print("=" * 62)


def _print_dry_run(cfg: RobotConfig, components: List[str]) -> None:
    for name in components:
        spec = cfg.launcher.get(name)
        argv = spec.build_argv(cfg.launcher.setups)
        print(f"\n── {name} ──")
        print(f"  cwd: {spec.cwd or '(현재 디렉토리)'}")
        if spec.depends_on:
            print(f"  depends_on: {spec.depends_on}")
        print(f"  argv: {argv}")


def _monitor_loop(manager: LocalManager, components: List[str],
                  summary_interval: float) -> None:
    """0.5s 폴링 — 상태 변화 시 한 줄 출력, 주기적으로 요약 테이블."""
    last_state: Dict[str, str] = {}
    last_summary = 0.0
    while True:
        time.sleep(0.5)
        statuses = {s["name"]: s for s in manager.status_all()
                    if s["name"] in components}
        for name, st in statuses.items():
            state = st["state"]
            prev = last_state.get(name)
            if prev is not None and prev != state:
                rc = st.get("returncode")
                extra = f" (rc={rc})" if rc is not None else ""
                mark = "⚠ " if state == "exited" else ""
                print(f"[bringup] {mark}{name}: {prev} → {state}{extra}", flush=True)
            last_state[name] = state

        now = time.time()
        if now - last_summary >= summary_interval:
            last_summary = now
            cells = []
            for name in components:
                st = statuses.get(name, {})
                state = st.get("state", "?")
                icon = {"running": "●", "done": "✔", "stopped": "○",
                        "exited": "✖"}.get(state, "?")
                cells.append(f"{icon} {name}")
            print(f"[bringup] {'  '.join(cells)}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="로봇 PC 브링업 (팔/손/캠 일괄 기동 — 터미널용)")
    parser.add_argument("--config", default=None,
                        help="robot.yaml 경로 (default: launcher/robot/config/robot.yaml)")
    parser.add_argument("--side", choices=["right", "left", "both"], default="both",
                        help="팔/손의 좌우 선택 (default: both)")
    parser.add_argument("--no-arms", action="store_true", help="팔 수신부 제외")
    parser.add_argument("--no-hands", action="store_true", help="손 드라이버+수신부 제외")
    parser.add_argument("--no-cam", action="store_true", help="캠 서버 제외")
    parser.add_argument("--parts", nargs="+", default=None, metavar="NAME",
                        help="파츠 이름 직접 지정 (side/no-* 무시)")
    parser.add_argument("--list", action="store_true",
                        help="선택된 파츠/컴포넌트 출력 후 종료")
    parser.add_argument("--dry-run", action="store_true",
                        help="실행될 argv 출력 후 종료 (실행 안 함)")
    parser.add_argument("--status-interval", type=float, default=10.0,
                        help="상태 요약 출력 주기 (s, default 10)")
    args = parser.parse_args()

    cfg = load_robot_config(args.config)
    try:
        parts = select_parts(cfg, side=args.side, no_arms=args.no_arms,
                             no_hands=args.no_hands, no_cam=args.no_cam,
                             only=args.parts)
    except KeyError as e:
        print(f"[bringup] ERROR: {e}")
        return 1
    if not parts:
        print("[bringup] 선택된 파츠 없음 — --side/--no-*/--parts 확인")
        return 1

    manager = LocalManager(cfg.launcher, groups=["robot"])
    selected = components_for(parts)
    # 표시용 순서 = topo (LocalManager.start_all 이 실제 순서도 보장)
    ordered = [c.name for c in topo_order([manager.spec(n) for n in selected])]

    _print_selection(cfg, parts, ordered)

    if args.list:
        return 0
    if args.dry_run:
        _print_dry_run(cfg, ordered)
        return 0

    lock = LauncherLock()
    msg = acquire_or_exit_message(lock)
    if msg:
        print(f"[bringup] {msg}")
        return 1

    try:
        started = manager.start_all(selected)
        print(f"[bringup] 시작됨: {', '.join(started)}")
        print("[bringup] Ctrl+C 로 전체 정지")
        _monitor_loop(manager, selected, args.status_interval)
    except KeyboardInterrupt:
        print("\n[bringup] 정지 중 (역순)...")
    finally:
        stopped = manager.stop_all(selected)
        if stopped:
            print(f"[bringup] 정지됨: {', '.join(stopped)}")
        lock.release()

    print("[bringup] exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
