"""양손 제스처 → 키 명령 (opt-in, 기본 off).

헤드셋을 쓴 채 키보드 없이 조종을 시작/일시정지하기 위한 기능:
    양손 동시 pinch  (엄지-검지 <1cm)  를 hold_s 유지 → 'r' (캘리브레이션/시작)
    양손 동시 squeeze(엄지-중지 <7cm) 를 hold_s 유지 → 'p' (일시정지 토글)

⚠️ 오발동 방지 설계 (조종 중 파지 동작에서 pinch/squeeze 는 자연 발생):
    1. **양손 동시** — 한 손만으로는 절대 발화하지 않음
    2. **hold_s 연속 유지** (기본 1.5s) — 한 샘플이라도 조건이 깨지면 리셋
    3. **refractory_s 불응기** (기본 3.0s) — 연속 발화 차단
    4. **per-hand msg 신선도** — 트래킹 유실 시 pinch bool 이 마지막 값으로
       동결되므로, 손별 hand msg age < hand_fresh_s 필수
    5. **kp25 유효성** — 손 키포인트가 valid 해야 함
    6. pinch 우선 — pinch 성립 시 squeeze 로 해석하지 않음
       (pinch 자세에서는 엄지-중지 거리도 작아 squeeze 가 함께 성립하기 쉬움)

사용: scripts/run_xr_dual_teleop.py 가 xr_dual.yaml `gestures:` 섹션에 따라
스폰. emit 콜백으로 키를 arm sender key_queue 들에 broadcast 한다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from sender.hand.xr_remap import is_kp25_valid


@dataclass
class GestureConfig:
    enabled: bool = False
    hold_s: float = 1.5
    refractory_s: float = 3.0
    sample_hz: float = 20.0
    hand_fresh_s: float = 0.3     # 손별 hand msg 최대 age


# 제스처 → 키 매핑 (고정)
GESTURE_KEYS = {"pinch": "r", "squeeze": "p"}


class GestureCommander(threading.Thread):
    """BridgePoseStore 의 pinch/squeeze 를 20Hz 샘플링해 키 발행.

    step() 은 단위 테스트 가능하도록 분리 — 한 샘플 처리 후 발화한 키
    (또는 None) 를 반환한다.
    """

    def __init__(self, store, emit: Callable[[str], None],
                 cfg: Optional[GestureConfig] = None,
                 stop_event: Optional[threading.Event] = None,
                 now_fn: Callable[[], float] = time.perf_counter):
        super().__init__(daemon=True, name="gesture-commander")
        self._store = store
        self._emit = emit
        self._cfg = cfg or GestureConfig()
        self._stop = stop_event or threading.Event()
        self._now = now_fn

        self._hold_gesture: Optional[str] = None   # 현재 유지 중인 제스처
        self._hold_since: float = 0.0
        self._last_fire: float = -1e9

    # ── 판정 ────────────────────────────────────────────────────────────

    def _current_gesture(self) -> Optional[str]:
        """지금 이 순간 양손이 만드는 유효 제스처 — 없으면 None."""
        now = self._now()
        stats = self._store.get_stats()

        for side in ("left", "right"):
            # 손별 신선도 (동결 bool 차단)
            t = stats.get(f"last_{side}_hand_msg_time", 0.0)
            if t <= 0.0 or now - t > self._cfg.hand_fresh_s:
                return None
            # 키포인트 유효성
            kp = getattr(self._store, f"{side}_hand_positions")
            if not is_kp25_valid(kp):
                return None

        l_pinch = bool(self._store.left_hand_pinch)
        r_pinch = bool(self._store.right_hand_pinch)
        l_squeeze = bool(self._store.left_hand_squeeze)
        r_squeeze = bool(self._store.right_hand_squeeze)

        # pinch 우선 (pinch 시 squeeze 도 함께 성립하기 쉬움)
        if l_pinch and r_pinch:
            return "pinch"
        if l_squeeze and r_squeeze and not (l_pinch or r_pinch):
            return "squeeze"
        return None

    def step(self) -> Optional[str]:
        """한 샘플 처리. 발화 시 키 반환 (emit 호출 포함)."""
        now = self._now()
        gesture = self._current_gesture()

        if gesture is None or now - self._last_fire < self._cfg.refractory_s:
            # 조건 깨짐 / 불응기 → hold 리셋
            self._hold_gesture = None
            return None

        if gesture != self._hold_gesture:
            self._hold_gesture = gesture
            self._hold_since = now
            return None

        if now - self._hold_since >= self._cfg.hold_s:
            key = GESTURE_KEYS[gesture]
            self._last_fire = now
            self._hold_gesture = None
            action = "캘리브레이션" if key == "r" else "일시정지 토글"
            print(f"[gesture] 양손 {gesture} {self._cfg.hold_s:.1f}s hold → "
                  f"'{key}' ({action})", flush=True)
            self._emit(key)
            return key
        return None

    # ── thread ──────────────────────────────────────────────────────────

    def run(self) -> None:
        dt = 1.0 / max(self._cfg.sample_hz, 1.0)
        print(f"[gesture] 활성 — 양손 pinch {self._cfg.hold_s}s='r', "
              f"양손 squeeze {self._cfg.hold_s}s='p' "
              f"(불응기 {self._cfg.refractory_s}s)", flush=True)
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as e:
                print(f"[gesture] WARN: step 오류 — {e}", flush=True)
            self._stop.wait(dt)
