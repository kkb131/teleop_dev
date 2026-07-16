"""러너 슬롯 제어 + "조종 시작" 자동 캘리브레이션 FSM.

RunnerControl — 상호배타 러너 슬롯:
    xr-dual / xr-single-* 은 모두 BridgePoseStore(8013)와 UDP 포트를 쓰므로
    동시에 하나만. 다른 모드 실행 중 start 는 {"running_mode": ...} 로 거부
    (API 가 409 로 응답, 프론트가 confirm 후 stop→start 스왑).

SessionRunner — 혼자 운용을 위한 자동 시퀀스:
    idle → adb(디바이스+reverse) → starting(러너 기동, Running. 대기)
         → countdown(자세 잡기, 기본 4s) → calibrating('r' 전송 → 로그에서
           팔별 calibrate 확인; 거부 WARN 은 재시도, Pose query failed 는
           hard-fail) → active | failed | cancelled

    확인 윈도우는 6s 이상 유지 — query_initial_pose 가 5s 후 DEFAULT_HOME
    fallback 하며 calibrate 가 "성공"으로 찍히는 함정을 hard-fail 로 잡기 위함.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional

import yaml as pyyaml

from launcher.manager import LocalManager
from launcher.sender.adb import AdbHelper
from launcher.sender.parts import SenderConfig
from launcher.sender.status import (
    count_calibrates,
    find_hard_fail,
    find_soft_fail,
    parse_runner_log,
)

# 웹 키패드 허용 키 (러너의 키 계약)
KEY_MAP = {
    "r": "r", "p": "p", "c": "c",
    "space": " ", " ": " ",
    "+": "+", "-": "-",
    "x": "x", "q": "q",
}


class RunnerControl:
    def __init__(self, cfg: SenderConfig, manager: LocalManager):
        self._cfg = cfg
        self._manager = manager
        self._lock = threading.Lock()

    def current_mode(self) -> Optional[str]:
        for r in self._cfg.runners:
            if self._manager.status(r.component)["state"] == "running":
                return r.name
        return None

    def start(self, mode: str) -> dict:
        with self._lock:
            runner = self._cfg.runner(mode)          # KeyError → api 404
            current = self.current_mode()
            if current == mode:
                return {"ok": True, "already_running": True, "mode": mode}
            if current is not None:
                return {"ok": False, "running_mode": current,
                        "error": f"다른 러너({current})가 실행 중 — 정지 후 시작하세요"}
            self._manager.start(runner.component)
            return {"ok": True, "mode": mode, "component": runner.component}

    def stop(self) -> dict:
        with self._lock:
            current = self.current_mode()
            if current is None:
                return {"ok": True, "stopped": None}
            self._manager.stop(self._cfg.runner(current).component)
            return {"ok": True, "stopped": current}

    def send_key(self, key: str) -> dict:
        ch = KEY_MAP.get(key)
        if ch is None:
            return {"ok": False, "error": f"허용되지 않은 키: {key!r} "
                                          f"(가능: {sorted(set(KEY_MAP) - {' '})})"}
        current = self.current_mode()
        if current is None:
            return {"ok": False, "error": "실행 중인 러너 없음"}
        comp = self._cfg.runner(current).component
        sent = self._manager.send_input(comp, ch)
        return {"ok": sent, "mode": current, "key": key} if sent else \
            {"ok": False, "error": "키 전달 실패 (pty 없음?)"}


class SessionRunner:
    """자동 캘리 시퀀스 — 한 번에 하나의 세션만."""

    def __init__(self, cfg: SenderConfig, manager: LocalManager,
                 runner: RunnerControl, adb: AdbHelper):
        self._cfg = cfg
        self._manager = manager
        self._runner = runner
        self._adb = adb
        # RLock — start() 가 lock 을 쥔 채 _set()/state() 를 호출 (재진입)
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._cancel = threading.Event()
        self._state: dict = {"state": "idle"}
        self._history: deque = deque(maxlen=12)

    # ── public ──────────────────────────────────────────────────────────

    def start(self, mode: Optional[str] = None,
              skip_adb: bool = False) -> dict:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return {"ok": False, "error": "세션이 이미 진행 중",
                        "state": self.state()}
            mode = mode or self._cfg.session.default_mode
            self._cfg.runner(mode)      # KeyError → api 404
            self._cancel.clear()
            self._history.clear()
            self._set("adb", mode=mode, attempt=0, message="시작")
            self._thread = threading.Thread(
                target=self._run, args=(mode, skip_adb),
                daemon=True, name="session-runner")
            self._thread.start()
            return {"ok": True, "mode": mode}

    def cancel(self) -> dict:
        self._cancel.set()
        return {"ok": True}

    def state(self) -> dict:
        with self._lock:
            st = dict(self._state)
            st["history"] = list(self._history)
            if st.get("state") == "countdown" and "countdown_until" in st:
                st["countdown_remaining_s"] = round(
                    max(0.0, st["countdown_until"] - time.time()), 1)
            return st

    # ── internals ───────────────────────────────────────────────────────

    def _set(self, state: str, **kw) -> None:
        with self._lock:
            prev = self._state if self._state.get("state") != "idle" else {}
            self._state = {"state": state, "ts": time.time(),
                           **{k: prev.get(k) for k in ("mode", "attempt")
                              if k in prev}, **kw}
            msg = kw.get("message", "")
            self._history.append(f"{time.strftime('%H:%M:%S')} {state}"
                                 + (f" — {msg}" if msg else ""))

    def _cancelled(self) -> bool:
        return self._cancel.is_set()

    def _expected_sides(self, mode: str) -> List[str]:
        runner = self._cfg.runner(mode)
        if runner.kind == "single":
            return ["single"]
        # dual: xr_dual.yaml 에서 enabled 팔을 fresh 하게 읽기
        target = self._cfg.params.get("xr-dual")
        sides = ["right", "left"]
        if target:
            try:
                data = pyyaml.safe_load(
                    Path(self._cfg.resolve_path(target["file"])).read_text()) or {}
                arms = data.get("arms") or {}
                sides = [s for s in ("right", "left")
                         if (arms.get(s) or {}).get("enabled", True)]
            except (OSError, pyyaml.YAMLError):
                pass
        return sides or ["right", "left"]

    def _run(self, mode: str, skip_adb: bool) -> None:
        ses = self._cfg.session
        comp = self._cfg.runner(mode).component

        # 1) adb
        if not skip_adb:
            self._set("adb", mode=mode, message="헤드셋 연결 확인")
            if not self._adb.available():
                return self._set("failed", mode=mode,
                                 message="adb 미설치 — xr_input_guide.md 설치 절차 참조")
            devices = self._adb.devices()
            if not any(d["state"] == "device" for d in devices):
                bad = ", ".join(f"{d['serial']}:{d['state']}" for d in devices) or "없음"
                return self._set("failed", mode=mode,
                                 message=f"헤드셋 미연결 (adb devices: {bad})")
            rev = self._adb.ensure_reverses(self._cfg.adb.reverse_ports)
            if not rev["ok"]:
                fails = [p for p, r in rev["results"].items() if not r["ok"]]
                return self._set("failed", mode=mode,
                                 message=f"adb reverse 실패: {fails}")
        if self._cancelled():
            return self._set("cancelled", mode=mode)

        # 2) starting — 러너 기동 (이미 실행 중이면 재사용 → 재캘리 용도)
        self._set("starting", mode=mode, message="러너 기동")
        current = self._runner.current_mode()
        if current is not None and current != mode:
            return self._set("failed", mode=mode,
                             message=f"다른 러너({current}) 실행 중 — 정지 후 다시 시작")
        if current is None:
            r = self._runner.start(mode)
            if not r.get("ok"):
                return self._set("failed", mode=mode, message=str(r.get("error")))
        deadline = time.time() + ses.start_timeout_s
        while time.time() < deadline:
            if self._cancelled():
                return self._set("cancelled", mode=mode)
            lines = [l for _, l in self._manager.log_since(comp, 0, 200)]
            if parse_runner_log(lines)["running_seen"]:
                break
            if self._manager.status(comp)["state"] != "running":
                return self._set("failed", mode=mode,
                                 message="러너가 기동 중 종료됨 — 로그 확인")
            time.sleep(0.2)
        else:
            return self._set("failed", mode=mode,
                             message=f"러너 준비 대기 초과 ({ses.start_timeout_s}s)")

        # 3) countdown — 자세 잡을 시간
        until = time.time() + ses.countdown_s
        self._set("countdown", mode=mode, countdown_until=until,
                  message=f"{ses.countdown_s:.0f}초 후 캘리브레이션 — "
                          "표준 자세에서 손을 시야에")
        while time.time() < until:
            if self._cancelled():
                return self._set("cancelled", mode=mode)
            time.sleep(0.1)

        # 4) calibrating — 'r' 전송 → 로그 확인 → 재시도
        expected = self._expected_sides(mode)
        last_reason = ""
        for attempt in range(1, ses.max_retries + 1):
            if self._cancelled():
                return self._set("cancelled", mode=mode)
            seq0 = self._manager.status(comp)["log_seq"]
            if not self._manager.send_input(comp, "r"):
                return self._set("failed", mode=mode, attempt=attempt,
                                 message="키 전달 실패 (러너 종료?)")
            self._set("calibrating", mode=mode, attempt=attempt,
                      expected_arms=expected, confirmed_arms=[],
                      message=f"시도 {attempt}/{ses.max_retries}"
                              + (f" (직전: {last_reason})" if last_reason else ""))

            window_end = time.time() + ses.confirm_timeout_s
            confirmed: List[str] = []
            hard: Optional[str] = None
            soft: Optional[str] = None
            while time.time() < window_end:
                if self._cancelled():
                    return self._set("cancelled", mode=mode)
                new_lines = [l for _, l in
                             self._manager.log_since(comp, seq0, 400)]
                counts = count_calibrates(new_lines)
                confirmed = [s for s in expected if counts.get(s, 0) >= 1]
                hard = find_hard_fail(new_lines)
                soft = find_soft_fail(new_lines)
                if hard:
                    break
                if len(confirmed) == len(expected):
                    break
                if soft and not counts:
                    break     # 전 팔 거부 — 윈도우 다 기다릴 필요 없음
                time.sleep(0.2)

            if hard:
                return self._set("failed", mode=mode, attempt=attempt,
                                 confirmed_arms=confirmed, message=hard)
            if len(confirmed) == len(expected):
                return self._set("active", mode=mode, attempt=attempt,
                                 confirmed_arms=confirmed,
                                 message="캘리브레이션 완료 — 조종 활성")
            last_reason = soft or (
                f"calibrate 미확인 (완료: {confirmed or '없음'})")
            time.sleep(ses.retry_delay_s)

        self._set("failed", mode=mode, attempt=ses.max_retries,
                  message=f"캘리브레이션 실패 ({ses.max_retries}회) — "
                          f"마지막 사유: {last_reason}")
