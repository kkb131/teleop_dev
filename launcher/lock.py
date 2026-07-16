"""런처 관리자 상호배제 lock (공용).

한 머신에서 같은 컴포넌트군을 관리하는 프로세스(웹 데몬 / bringup CLI)는
하나만 실행되어야 한다 — flock 으로 잠근다. robot/sender 는 서로 다른
lock 경로를 사용하므로 동시 실행 가능:
    robot : /tmp/teleop_robot_launcher.lock
    sender: /tmp/teleop_sender_launcher.lock
"""

from __future__ import annotations

import fcntl
import os
from typing import Optional


class LauncherLock:
    """fcntl.flock 기반 배타 lock. 프로세스 종료 시 자동 해제."""

    def __init__(self, path: str):
        self._path = path
        self._fd: Optional[int] = None

    def acquire(self) -> bool:
        """비차단 획득. 성공 True / 이미 다른 관리자가 잡고 있으면 False."""
        fd = os.open(self._path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            return False
        os.ftruncate(fd, 0)
        os.write(fd, f"{os.getpid()}\n".encode())
        self._fd = fd
        return True

    def holder_pid(self) -> Optional[int]:
        """lock 파일에 기록된 (마지막) 소유자 pid — 안내 메시지용."""
        try:
            with open(self._path) as f:
                return int(f.read().strip() or 0) or None
        except (OSError, ValueError):
            return None

    def keep_across_exec(self) -> None:
        """os.exec* 이후에도 lock fd 가 살아남도록 CLOEXEC 해제.

        bringup CLI 가 러너를 foreground exec 할 때 사용 — 러너가 도는 동안
        웹 데몬의 이중 관리를 계속 차단한다 (러너 종료 시 fd 닫히며 해제).
        """
        if self._fd is not None:
            os.set_inheritable(self._fd, True)

    def release(self) -> None:
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None


def acquire_or_exit_message(lock: LauncherLock,
                            health_url: str = "http://localhost:9876/api/health"
                            ) -> Optional[str]:
    """획득 실패 시 사용자 안내 문자열 반환 (성공 시 None)."""
    if lock.acquire():
        return None
    pid = lock.holder_pid()
    return (
        f"다른 런처 관리자(pid {pid})가 이 머신의 컴포넌트를 관리 중입니다.\n"
        f"  - 웹 데몬이 떠 있는지 확인: curl {health_url}\n"
        f"  - 웹/다른 bringup 을 먼저 종료한 뒤 다시 실행하세요."
    )
