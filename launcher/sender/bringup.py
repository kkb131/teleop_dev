#!/usr/bin/env python3
"""조종 PC 브링업 CLI — adb reverse 후 XR 러너 실행.

    # adb reverse → 듀얼 러너를 현재 터미널에서 그대로 실행 (키보드 직접 사용)
    python3 -m launcher.sender.bringup --mode xr-dual

    # 자동 캘리: 러너를 pty 로 띄우고 4초 카운트다운 후 'r' 자동 전송
    python3 -m launcher.sender.bringup --mode xr-dual --auto-calib

    python3 -m launcher.sender.bringup --list       # 모드 목록
    python3 -m launcher.sender.bringup --dry-run    # 실행될 명령 확인

기본 모드는 foreground exec — 러너가 이 터미널을 그대로 물려받아 수동
실행과 동일한 UX (키 입력 직접). lock fd 는 exec 후에도 유지되어 러너가
도는 동안 웹 데몬의 이중 관리를 차단한다.

--auto-calib 모드는 pty 로 스폰 + SessionRunner FSM + 라인 중계:
러너 출력은 그대로 표시되고, 이 터미널의 키 입력도 러너로 전달된다.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

from launcher.lock import LauncherLock, acquire_or_exit_message
from launcher.manager import LocalManager
from launcher.sender.actions import RunnerControl, SessionRunner
from launcher.sender.adb import AdbHelper
from launcher.sender.parts import load_sender_config
from launcher.sender.web import LOCK_PATH


def _do_adb(cfg, skip: bool) -> bool:
    if skip:
        return True
    helper = AdbHelper(cfg.adb.binary)
    if not helper.available():
        print("[bringup] adb 미설치 — xr_input_guide.md 설치 절차 참조 "
              "(--skip-adb 로 건너뛰기 가능)")
        return False
    devices = helper.devices()
    if not any(d["state"] == "device" for d in devices):
        bad = ", ".join(f"{d['serial']}:{d['state']}" for d in devices) or "없음"
        print(f"[bringup] 헤드셋 미연결 (adb devices: {bad}) — USB/디버깅 승인 확인")
        return False
    rev = helper.ensure_reverses(cfg.adb.reverse_ports)
    for port, r in rev["results"].items():
        print(f"[bringup] adb reverse tcp:{port}: {'OK' if r['ok'] else r['output']}")
    return rev["ok"]


def _auto_calib_loop(cfg, mode: str) -> int:
    """pty 스폰 + 자동 캘리 FSM + 라인 중계 (출력↓ / 키 입력↑)."""
    manager = LocalManager(cfg.launcher, groups=["operator"])
    runner = RunnerControl(cfg, manager)
    session = SessionRunner(cfg, manager, runner, AdbHelper(cfg.adb.binary))
    comp = cfg.runner(mode).component

    session.start(mode=mode, skip_adb=True)   # adb 는 이미 CLI 에서 처리
    print(f"[bringup] 자동 캘리 시퀀스 시작 — countdown {cfg.session.countdown_s:.0f}s "
          f"후 'r' 자동 전송. 키 입력은 러너로 전달됨 (x=종료)")

    old_settings = None
    if sys.stdin.isatty():
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    log_cursor = 0
    last_session = ""
    try:
        while True:
            # 러너 출력 중계
            for seq, line in manager.log_since(comp, log_cursor, 200):
                log_cursor = max(log_cursor, seq)
                print(line, flush=True)
            # 세션 상태 변화 표시
            st = session.state()
            desc = f"{st['state']}" + (f" — {st.get('message', '')}"
                                       if st.get("message") else "")
            if desc != last_session:
                last_session = desc
                print(f"[bringup:session] {desc}", flush=True)
            # 키 중계
            if old_settings and select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                manager.send_input(comp, ch)
                if ch in ("x", "q"):
                    break
            else:
                time.sleep(0.1 if not old_settings else 0.0)
            # 러너 종료 감지
            if manager.status(comp)["state"] != "running" and \
                    st["state"] in ("active", "failed", "cancelled", "idle"):
                if manager.status(comp)["state"] != "running":
                    print("[bringup] 러너 종료됨")
                    break
    except KeyboardInterrupt:
        print("\n[bringup] 중단")
    finally:
        if old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        session.cancel()
        manager.stop_all()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="조종 PC 브링업 (adb + XR 러너)")
    parser.add_argument("--config", default=None,
                        help="sender.yaml 경로 (default: launcher/sender/config/sender.yaml)")
    parser.add_argument("--mode", default=None,
                        help="러너 모드 (default: session.default_mode)")
    parser.add_argument("--skip-adb", action="store_true",
                        help="adb 확인/reverse 생략 (Wi-Fi adb 등)")
    parser.add_argument("--auto-calib", action="store_true",
                        help="러너를 pty 로 띄우고 카운트다운 후 'r' 자동 전송")
    parser.add_argument("--list", action="store_true", help="모드 목록 출력")
    parser.add_argument("--dry-run", action="store_true", help="실행될 명령 출력")
    args = parser.parse_args()

    cfg = load_sender_config(args.config)
    mode = args.mode or cfg.session.default_mode

    if args.list:
        for r in cfg.runners:
            mark = " (default)" if r.name == cfg.session.default_mode else ""
            print(f"  {r.name:<18s} {r.label}{mark}")
        return 0

    try:
        runner = cfg.runner(mode)
    except KeyError as e:
        print(f"[bringup] {e}")
        return 1
    spec = cfg.launcher.get(runner.component)
    argv = spec.build_argv(cfg.launcher.setups)

    if args.dry_run:
        print(f"mode: {mode} → component: {runner.component}")
        print(f"cwd:  {spec.cwd or '(현재)'}")
        print(f"argv: {argv}")
        print(f"adb reverse ports: {cfg.adb.reverse_ports}"
              + (" (--skip-adb)" if args.skip_adb else ""))
        return 0

    lock = LauncherLock(LOCK_PATH)
    msg = acquire_or_exit_message(
        lock, f"http://localhost:{cfg.launcher.web.port}/api/health")
    if msg:
        print(f"[bringup] {msg}")
        return 1

    if not _do_adb(cfg, args.skip_adb):
        lock.release()
        return 1

    if args.auto_calib:
        try:
            return _auto_calib_loop(cfg, mode)
        finally:
            lock.release()

    # foreground exec — 러너가 이 터미널을 물려받음 (lock fd 는 exec 생존)
    print(f"[bringup] exec: {mode} ({runner.component})")
    lock.keep_across_exec()
    if spec.cwd:
        os.chdir(spec.cwd)
    env = {**os.environ, **spec.env}
    os.execvpe(argv[0], argv, env)
    return 0   # unreachable


if __name__ == "__main__":
    sys.exit(main())
