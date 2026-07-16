"""GestureCommander 단위 테스트 — FakeStore + 주입 시계 (하드웨어 불필요).

실행: python3 -m unittest sender.xr_common.tests.test_gesture_commands
"""

import unittest

import numpy as np

from sender.xr_common.gesture_commands import GestureCommander, GestureConfig


def _valid_kp25():
    """is_kp25_valid 를 통과하는 그럴듯한 25×3 keypoints."""
    kp = np.zeros((25, 3))
    kp[:, 0] = np.linspace(0.0, 0.15, 25)
    kp[:, 1] = np.linspace(0.0, 0.05, 25)
    kp[:, 2] = 0.4
    return kp


class FakeStore:
    """BridgePoseStore 의 제스처 관련 표면만 흉내."""

    def __init__(self):
        self.left_hand_pinch = False
        self.right_hand_pinch = False
        self.left_hand_squeeze = False
        self.right_hand_squeeze = False
        self.left_hand_positions = _valid_kp25()
        self.right_hand_positions = _valid_kp25()
        self.left_fresh_at = 0.0     # 마지막 hand msg 시각
        self.right_fresh_at = 0.0

    def get_stats(self):
        return {
            "last_left_hand_msg_time": self.left_fresh_at,
            "last_right_hand_msg_time": self.right_fresh_at,
        }


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TestGestureCommander(unittest.TestCase):
    def setUp(self):
        self.store = FakeStore()
        self.clock = Clock()
        self.emitted = []
        self.cmd = GestureCommander(
            self.store, emit=self.emitted.append,
            cfg=GestureConfig(enabled=True, hold_s=1.5, refractory_s=3.0,
                              hand_fresh_s=0.3),
            now_fn=self.clock)

    def _fresh(self):
        self.store.left_fresh_at = self.clock.t
        self.store.right_fresh_at = self.clock.t

    def _advance(self, dt, keep_fresh=True):
        """dt 를 0.05s 단위로 진행하며 step() 반복 — 발화 키 목록 반환."""
        fired = []
        steps = max(1, int(dt / 0.05))
        for _ in range(steps):
            self.clock.t += dt / steps
            if keep_fresh:
                self._fresh()
            k = self.cmd.step()
            if k:
                fired.append(k)
        return fired

    def test_both_pinch_hold_fires_r_once(self):
        self._fresh()
        self.store.left_hand_pinch = self.store.right_hand_pinch = True
        fired = self._advance(2.0)
        self.assertEqual(fired, ["r"])
        self.assertEqual(self.emitted, ["r"])
        # 계속 유지해도 불응기 동안 재발화 없음
        fired = self._advance(2.0)
        self.assertEqual(fired, [])
        # 불응기(3s) 경과 + 계속 유지 → hold 재축적 후 재발화 가능
        fired = self._advance(6.0)
        self.assertEqual(fired, ["r"])

    def test_single_hand_never_fires(self):
        self._fresh()
        self.store.right_hand_pinch = True     # 오른손만
        self.assertEqual(self._advance(5.0), [])

    def test_hold_reset_on_break(self):
        self._fresh()
        self.store.left_hand_pinch = self.store.right_hand_pinch = True
        self._advance(1.0)                     # hold 1.0s (< 1.5s)
        self.store.left_hand_pinch = False     # 잠깐 풀림
        self._advance(0.1)
        self.store.left_hand_pinch = True
        fired = self._advance(1.0)             # 리셋됐으므로 아직 미발화
        self.assertEqual(fired, [])
        fired = self._advance(0.7)
        self.assertEqual(fired, ["r"])

    def test_stale_hand_blocks(self):
        """트래킹 유실로 pinch bool 이 동결된 경우 — 발화 금지 (핵심 안전)."""
        self._fresh()
        self.store.left_hand_pinch = self.store.right_hand_pinch = True
        # 왼손 msg 가 갱신되지 않음 (동결) — keep_fresh 로 오른손만 갱신
        fired = []
        for _ in range(60):
            self.clock.t += 0.05
            self.store.right_fresh_at = self.clock.t   # 오른손만 fresh
            k = self.cmd.step()
            if k:
                fired.append(k)
        self.assertEqual(fired, [])

    def test_invalid_kp_blocks(self):
        self._fresh()
        self.store.left_hand_pinch = self.store.right_hand_pinch = True
        self.store.left_hand_positions = np.zeros((25, 3))   # invalid
        self.assertEqual(self._advance(3.0), [])

    def test_squeeze_fires_p(self):
        self._fresh()
        self.store.left_hand_squeeze = self.store.right_hand_squeeze = True
        fired = self._advance(2.0)
        self.assertEqual(fired, ["p"])

    def test_pinch_priority_over_squeeze(self):
        """pinch 성립 시 squeeze 동시 성립해도 'r' (pinch 우선)."""
        self._fresh()
        self.store.left_hand_pinch = self.store.right_hand_pinch = True
        self.store.left_hand_squeeze = self.store.right_hand_squeeze = True
        fired = self._advance(2.0)
        self.assertEqual(fired, ["r"])

    def test_mixed_gesture_blocked(self):
        """한 손 pinch + 다른 손 squeeze → 발화 없음."""
        self._fresh()
        self.store.left_hand_pinch = True
        self.store.right_hand_squeeze = True
        self.assertEqual(self._advance(3.0), [])


if __name__ == "__main__":
    unittest.main()
