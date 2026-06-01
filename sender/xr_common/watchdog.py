"""Watchdog + workspace clamp for XR sender side.

xr_teleop 의 [docs/remote_teleop_office_to_field_analysis.md §4.6](../../../../xr_teleop/docs/remote_teleop_office_to_field_analysis.md)
의 100ms heartbeat / workspace boundary 권고를 sender 측 layer 로 구현.

용도:
    조종 PC 의 사용자 손/머리 추적이 끊겼을 때 (헤드셋 연결 끊김, 손 시야 밖
    지속, Chrome 멈춤) robot 으로 stale target 송신 방지. robot PC 측 admittance
    의 4단계 safety_monitor 는 그대로 + sender 측에서 추가 보호 layer.

설계:
    1. BridgePoseStore 의 `last_msg_time` 을 watchdog 가 매 loop 마다 확인.
    2. age > timeout (default 200ms) 이면 stale 으로 판정 → sender 가 packet 송신 skip
       또는 estop=True 강제.
    3. workspace clamp 는 base_link 좌표계 의 위치 envelope 검사. xr_teleop
       Track C 에서 권장한 절차.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class WorkspaceLimits:
    """robot base_link 좌표계의 안전 workspace envelope.

    UR10e 의 reach 가 ~1.3m 이지만 책상 / 충돌 회피용 보수적 default.
    sender 측에서 target_pos 를 이 envelope 안으로 clamp.
    """
    # 좌우 대칭 envelope. 이전에 y_max=0.0 (robot 뒤로 안 가도록) 로 두었더니
    # init pose [2.24, ...] 의 EE y 가 양수 영역이라 target_pos[1] 이 매번 0 으로
    # clamp 되어 사용자 손의 y(좌우) 이동이 robot 으로 전달되지 못하는 버그 발생.
    # robot 의 작업 영역이 정면 한쪽에만 있다고 가정하기보다는 대칭 권장.
    x_min: float = -3.7
    x_max: float =  3.7
    y_min: float = -3.7
    y_max: float =  3.7
    z_min: float =  0.05      # 책상 평면 위
    z_max: float =  3.8

    def clamp(self, pos: np.ndarray) -> Tuple[np.ndarray, bool]:
        """pos 를 envelope 안으로 clamp. (clamped, was_clamped) 반환."""
        pos = np.asarray(pos, dtype=np.float64).copy()
        was_clamped = False
        if pos[0] < self.x_min: pos[0] = self.x_min; was_clamped = True
        if pos[0] > self.x_max: pos[0] = self.x_max; was_clamped = True
        if pos[1] < self.y_min: pos[1] = self.y_min; was_clamped = True
        if pos[1] > self.y_max: pos[1] = self.y_max; was_clamped = True
        if pos[2] < self.z_min: pos[2] = self.z_min; was_clamped = True
        if pos[2] > self.z_max: pos[2] = self.z_max; was_clamped = True
        return pos, was_clamped


class StoreWatchdog:
    """BridgePoseStore freshness watchdog.

    매 sender loop 마다 `check()` 호출. age > timeout_s 면 sender 가 packet 송신
    skip — robot PC 측 admittance 가 자체 timeout / safety 로 stationary 유지.

    Usage:
        wd = StoreWatchdog(store, timeout_s=0.2)
        ...
        if not wd.fresh():
            # sender 송신 skip
            continue
    """

    def __init__(self, store, timeout_s: float = 0.2):
        self.store = store
        self.timeout_s = float(timeout_s)
        self._first_msg_seen = False
        self._stale_count = 0

    def fresh(self) -> bool:
        stats = self.store.get_stats()
        last = stats["last_msg_time"]
        if last <= 0:
            # 아직 첫 메시지 없음 — fresh=False, 송신 skip
            return False
        if not self._first_msg_seen:
            self._first_msg_seen = True
        age = time.perf_counter() - last
        ok = age < self.timeout_s
        if not ok:
            self._stale_count += 1
        else:
            self._stale_count = 0
        return ok

    @property
    def stale_count(self) -> int:
        return self._stale_count

    def reset(self) -> None:
        self._stale_count = 0
