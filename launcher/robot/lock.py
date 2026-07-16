"""bringup CLI ↔ 웹 데몬 상호배제 lock.

두 관리자가 같은 컴포넌트를 동시에 spawn/stop 하면 프로세스 소유권이
꼬이므로, 한 머신에서 launcher 관리자는 하나만 실행되도록 flock 으로 잠근다.
"""

from __future__ import annotations

import fcntl
import os
from typing import Optional

LOCK_PATH = "/tmp/teleop_robot_launcher.lock"


class LauncherLock:
    """fcntl.flock 기반 배타 lock. 프로세스 종료 시 자동 해제."""

    def __init__(self, path: str = LOCK_PATH):
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

    def release(self) -> None:
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None


def acquire_or_exit_message(lock: LauncherLock) -> Optional[str]:
    """획득 실패 시 사용자 안내 문자열 반환 (성공 시 None)."""
    if lock.acquire():
        return None
    pid = lock.holder_pid()
    return (
        f"다른 런처 관리자(pid {pid})가 이 머신의 컴포넌트를 관리 중입니다.\n"
        f"  - 웹 대시보드가 떠 있는지 확인: curl http://localhost:9876/api/health\n"
        f"  - 웹/다른 bringup 을 먼저 종료한 뒤 다시 실행하세요."
    )
