"""로봇 PC 런처 lock — 공용 구현(launcher/lock.py)의 robot 기본값 래퍼."""

from __future__ import annotations

from typing import Optional

from launcher.lock import LauncherLock as _LauncherLock
from launcher.lock import acquire_or_exit_message as _acquire_or_exit_message

LOCK_PATH = "/tmp/teleop_robot_launcher.lock"


class LauncherLock(_LauncherLock):
    def __init__(self, path: str = LOCK_PATH):
        super().__init__(path)


def acquire_or_exit_message(lock: LauncherLock) -> Optional[str]:
    return _acquire_or_exit_message(lock, "http://localhost:9876/api/health")
