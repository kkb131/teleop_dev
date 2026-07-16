"""RunnerControl + SessionRunner(자동 캘리 FSM) 통합 테스트 — mock 러너 사용.

실행: python3 -m unittest launcher.sender.tests.test_session
(pty 로 mock_runner 를 실제 spawn — 하드웨어/adb 불필요, ~15s)
"""

import time
import unittest
from pathlib import Path

from launcher.manager import LocalManager
from launcher.sender.actions import RunnerControl, SessionRunner
from launcher.sender.adb import AdbHelper
from launcher.sender.parts import load_sender_config

_DEMO = str(Path(__file__).parents[1] / "config" / "demo.yaml")


def _wait_session(session, until_states, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        st = session.state()
        if st["state"] in until_states:
            return st
        time.sleep(0.2)
    raise AssertionError(f"세션 타임아웃 — 마지막 상태: {session.state()}")


class TestSenderActions(unittest.TestCase):
    def setUp(self):
        self.cfg = load_sender_config(_DEMO)
        self.manager = LocalManager(self.cfg.launcher, groups=["operator"])
        self.runner = RunnerControl(self.cfg, self.manager)
        self.session = SessionRunner(self.cfg, self.manager, self.runner,
                                     AdbHelper(self.cfg.adb.binary))

    def tearDown(self):
        self.session.cancel()
        self.manager.stop_all()

    def test_runner_slot_and_keypad(self):
        r = self.runner.start("xr-dual")
        self.assertTrue(r["ok"])
        time.sleep(1.5)
        # 상호배타: 다른 모드 → 거부
        r2 = self.runner.start("xr-single-right")
        self.assertFalse(r2["ok"])
        self.assertEqual(r2["running_mode"], "xr-dual")
        # 같은 모드 → already_running
        r3 = self.runner.start("xr-dual")
        self.assertTrue(r3.get("already_running"))
        # 키패드: r → SYNC 는 fail-first 2 때문에 3회 필요. p 는 READY 에서 무시됨
        self.assertTrue(self.runner.send_key("r")["ok"])
        self.assertFalse(self.runner.send_key("z")["ok"])   # 허용 외 키
        # 정지
        r4 = self.runner.stop()
        self.assertEqual(r4["stopped"], "xr-dual")
        time.sleep(0.5)
        self.assertIsNone(self.runner.current_mode())
        # 러너 없음 상태에서 키 → 오류
        self.assertFalse(self.runner.send_key("r")["ok"])

    def test_session_retry_then_active(self):
        """dual mock: 'r' 2회 거부 → 3회차 성공 (재시도 경로)."""
        r = self.session.start(mode="xr-dual", skip_adb=True)
        self.assertTrue(r["ok"])
        # 이중 시작 거부
        r2 = self.session.start(mode="xr-dual", skip_adb=True)
        self.assertFalse(r2["ok"])
        st = _wait_session(self.session, {"active", "failed"})
        self.assertEqual(st["state"], "active", st)
        self.assertEqual(st["attempt"], 3)                    # fail-first 2
        self.assertEqual(sorted(st["confirmed_arms"]), ["left", "right"])
        # 러너는 계속 실행 중 (세션은 러너를 정지시키지 않음)
        self.assertEqual(self.runner.current_mode(), "xr-dual")

    def test_session_single_immediate(self):
        """single mock: 즉시 성공 + 무라벨 'single' 팔 확인."""
        r = self.session.start(mode="xr-single-right", skip_adb=True)
        self.assertTrue(r["ok"])
        st = _wait_session(self.session, {"active", "failed"})
        self.assertEqual(st["state"], "active", st)
        self.assertEqual(st["attempt"], 1)
        self.assertEqual(st["confirmed_arms"], ["single"])

    def test_session_hard_fail(self):
        """single-left mock: Pose query failed → 재시도 없이 hard-fail."""
        r = self.session.start(mode="xr-single-left", skip_adb=True)
        self.assertTrue(r["ok"])
        st = _wait_session(self.session, {"active", "failed"})
        self.assertEqual(st["state"], "failed", st)
        self.assertIn("로봇 PC 무응답", st["message"])
        self.assertEqual(st["attempt"], 1)                    # 재시도 안 함

    def test_session_adb_missing(self):
        """skip_adb=False + adb 미설치 바이너리 → adb 단계 실패."""
        r = self.session.start(mode="xr-dual", skip_adb=False)
        self.assertTrue(r["ok"])
        st = _wait_session(self.session, {"failed"}, timeout=10)
        self.assertIn("adb 미설치", st["message"])

    def test_session_cancel(self):
        r = self.session.start(mode="xr-dual", skip_adb=True)
        self.assertTrue(r["ok"])
        time.sleep(0.3)
        self.session.cancel()
        st = _wait_session(self.session, {"cancelled", "active", "failed"},
                           timeout=10)
        self.assertEqual(st["state"], "cancelled", st)


if __name__ == "__main__":
    unittest.main()
