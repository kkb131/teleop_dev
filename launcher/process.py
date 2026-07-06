"""ManagedProcess — 컴포넌트 subprocess 의 생명주기 + 로그 관리.

- start_new_session=True 로 자체 process group 생성 → killpg 로 자식
  (ros2 launch 가 띄우는 노드들 포함) 까지 한 번에 신호 전달.
- 종료 escalation: SIGINT → (stop_grace_s) → SIGTERM → (3s) → SIGKILL.
- 로그: stdout+stderr 합쳐 pump thread 가 ring buffer (deque) 에 적재.
  각 줄에 단조 증가 seq 부여 — GUI/agent 가 log_since(seq) 커서로 tail.
- use_pty: termios 를 쓰는 프로세스 (run_xr_dual_teleop 등) 용 —
  send_input() 으로 키 전달 가능.
"""

from __future__ import annotations

import collections
import os
import pty
import signal
import subprocess
import threading
import time
from typing import Deque, List, Optional, Tuple

from launcher.config import ComponentSpec

LOG_RING_SIZE = 2000


class ManagedProcess:
    def __init__(self, spec: ComponentSpec, setups: dict):
        self.spec = spec
        self._setups = setups
        self._proc: Optional[subprocess.Popen] = None
        self._pty_master: Optional[int] = None
        self._log: Deque[Tuple[int, str]] = collections.deque(maxlen=LOG_RING_SIZE)
        self._log_seq = 0
        self._log_lock = threading.Lock()
        self._start_time = 0.0
        self._stop_requested = False

    # ── lifecycle ───────────────────────────────────────────────────────

    def start(self) -> None:
        if self.is_running():
            return
        argv = self.spec.build_argv(self._setups)
        env = {**os.environ, **self.spec.env}
        self._stop_requested = False

        if self.spec.use_pty:
            master, slave = pty.openpty()
            self._proc = subprocess.Popen(
                argv, cwd=self.spec.cwd, env=env,
                stdin=slave, stdout=slave, stderr=slave,
                start_new_session=True, close_fds=True,
            )
            os.close(slave)
            self._pty_master = master
            pump = threading.Thread(target=self._pump_pty, daemon=True,
                                    name=f"log-{self.spec.name}")
        else:
            self._proc = subprocess.Popen(
                argv, cwd=self.spec.cwd, env=env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True,
            )
            pump = threading.Thread(target=self._pump_pipe, daemon=True,
                                    name=f"log-{self.spec.name}")

        self._start_time = time.time()
        self._append_log(f"── started (pid {self._proc.pid}) ──")
        pump.start()

    def stop(self) -> None:
        """SIGINT → SIGTERM → SIGKILL escalation. blocking (최대 grace+3s+2s)."""
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        self._stop_requested = True
        pgid = self._pgid()
        self._signal_group(pgid, signal.SIGINT)
        if self._wait(proc, self.spec.stop_grace_s):
            self._append_log("── stopped (SIGINT) ──")
            return
        self._signal_group(pgid, signal.SIGTERM)
        if self._wait(proc, 3.0):
            self._append_log("── stopped (SIGTERM) ──")
            return
        self._signal_group(pgid, signal.SIGKILL)
        self._wait(proc, 2.0)
        self._append_log("── killed (SIGKILL) ──")

    def send_input(self, text: str) -> bool:
        """pty 컴포넌트에 키 입력 전달 (예: 'r' calibrate)."""
        if self._pty_master is None or not self.is_running():
            return False
        os.write(self._pty_master, text.encode("utf-8"))
        return True

    # ── state ───────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def status(self) -> dict:
        if self._proc is None:
            state, rc = "stopped", None
        elif self._proc.poll() is None:
            state, rc = "running", None
        else:
            rc = self._proc.returncode
            if self.spec.oneshot and rc == 0:
                state = "done"
            elif self._stop_requested:
                state = "stopped"
            else:
                state = "exited"     # 비정상 (요청 없이 종료)
        return {
            "name": self.spec.name,
            "group": self.spec.group,
            "state": state,
            "pid": self._proc.pid if self.is_running() else None,
            "returncode": rc,
            "uptime_s": round(time.time() - self._start_time, 1) if self.is_running() else None,
            "log_seq": self._log_seq,
            "use_pty": self.spec.use_pty,
            "depends_on": list(self.spec.depends_on),
            "oneshot": self.spec.oneshot,
        }

    def log_since(self, since: int = 0, max_lines: int = 200) -> List[Tuple[int, str]]:
        with self._log_lock:
            return [(s, l) for s, l in self._log if s > since][-max_lines:]

    # ── internals ───────────────────────────────────────────────────────

    def _pgid(self) -> Optional[int]:
        try:
            return os.getpgid(self._proc.pid)
        except (ProcessLookupError, PermissionError):
            return None

    @staticmethod
    def _signal_group(pgid: Optional[int], sig: int) -> None:
        if pgid is None:
            return
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    @staticmethod
    def _wait(proc: subprocess.Popen, timeout: float) -> bool:
        try:
            proc.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    def _append_log(self, line: str) -> None:
        with self._log_lock:
            self._log_seq += 1
            self._log.append((self._log_seq, line))

    def _pump_pipe(self) -> None:
        proc = self._proc
        for raw in iter(proc.stdout.readline, b""):
            self._append_log(raw.decode("utf-8", errors="replace").rstrip("\n"))
        proc.stdout.close()
        proc.wait()
        self._append_log(f"── exited (rc={proc.returncode}) ──")

    def _pump_pty(self) -> None:
        proc = self._proc
        master = self._pty_master
        buf = b""
        while True:
            try:
                chunk = os.read(master, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            # \r 덮어쓰기 status 줄도 로그로 남도록 \r 을 줄바꿈으로 취급
            while b"\n" in buf or b"\r" in buf:
                idx_n = buf.find(b"\n")
                idx_r = buf.find(b"\r")
                idx = min(i for i in (idx_n, idx_r) if i >= 0)
                line, buf = buf[:idx], buf[idx + 1:]
                if line.strip():
                    self._append_log(line.decode("utf-8", errors="replace"))
        try:
            os.close(master)
        except OSError:
            pass
        self._pty_master = None
        proc.wait()
        self._append_log(f"── exited (rc={proc.returncode}) ──")
