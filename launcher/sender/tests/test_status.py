"""러너 로그 파서 단위 테스트 — 실측 확인된 포맷의 canned 라인 사용.

실행: python3 -m unittest launcher.sender.tests.test_status
"""

import unittest

from launcher.sender.status import (
    count_calibrates,
    find_hard_fail,
    find_soft_fail,
    parse_runner_log,
    parse_sender_cam_stats,
)

DUAL_LOG = [
    "[run_xr_dual_teleop] BridgePoseStore: http://localhost:8013/",
    "[BridgePoseStore] ws client connected: 127.0.0.1",
    "[Sender] Target: 192.168.0.10:9871 at 50 Hz",
    "[Sender] Initial pose received: pos=[0.500, -0.100, 0.400]",
    "[Sender] Initial pose received: pos=[0.500, 0.100, 0.400]",
    "[Sender] Running. Press Q or Ctrl+C to stop.",
    "[Sender:right] #    120  pos=[+0.500, -0.100, +0.400]  spd=READY x0.2",
    "[Sender:left] #    118  pos=[+0.500, +0.100, +0.400]  spd=READY x0.2",
    "[xr_dual:hand:right] #   240 TRACK skip=3 q[5/9/13/17]=[+0.10/+0.20/+0.30/+0.40]",
    "[xr_dual:hand:left] #   238 LOST  skip=12 q[5/9/13/17]=[+0.00/+0.00/+0.00/+0.00]",
    "[XRArmSender:right] calibrate (r)",
    "[XRArmSender:left] calibrate (r)",
    "[Sender:right] #    150  pos=[+0.510, -0.100, +0.400]  spd=SYNC x0.5",
    "[Sender:left] #    148  pos=[+0.500, +0.110, +0.400]  spd=SYNC x0.5",
    "[gesture] 양손 squeeze 1.5s hold → 'p' (pause toggle)",
    "[Sender:right] #    160  pos=[+0.510, -0.100, +0.400]  spd=PAUSE x0.5",
]

SINGLE_LOG = [
    "\r[Sender] #    999  pos=[+0.500, -0.100, +0.400]  spd=READY x0.2",
    "[run_xr_teleop:hand] #  1200 TRACK skip=0 ws_msg=5000 q[5/9/13/17]=[+0.1/+0.2/+0.3/+0.4]",
    "[XRArmSender] calibrate (r)",
]

WARN_LOG = [
    "[XRArmSender:right] WARN: 헤드셋 ws msg 없음 — 헤드셋 사이트 접속 + Enter VR/AR + 손 들이밀고 'r' 재시도",
]

HARDFAIL_LOG = [
    "[XRArmSender:right] calibrate (r)",
    "[Sender] WARNING: Pose query failed. Using default home pose.",
]

CAM_LOG = [
    "[sender.cam] vr.mode=head_locked  브라우저 뷰: http://localhost:8014/",
    "[sender.cam] head: 29.8fps 850KB/s lat=12ms  wrist: 15.0fps 420KB/s lat=25ms",
]


class TestParseRunnerLog(unittest.TestCase):
    def test_dual_full(self):
        st = parse_runner_log(DUAL_LOG)
        self.assertEqual(st["arms"]["right"]["state"], "PAUSE")   # 마지막 상태
        self.assertEqual(st["arms"]["left"]["state"], "SYNC")
        self.assertEqual(st["arms"]["right"]["calib_count"], 1)
        self.assertTrue(st["hands"]["right"]["tracking"])
        self.assertFalse(st["hands"]["left"]["tracking"])
        self.assertEqual(st["handshake"], {"received": 2, "failed": 0})
        self.assertEqual(st["headset_ws"], "connected")
        self.assertIn("squeeze", st["last_gesture"])
        self.assertTrue(st["running_seen"])

    def test_single_unlabeled(self):
        st = parse_runner_log(SINGLE_LOG)
        self.assertEqual(st["arms"]["single"]["state"], "READY")
        self.assertEqual(st["arms"]["single"]["calib_count"], 1)
        self.assertTrue(st["hands"]["single"]["tracking"])

    def test_warn(self):
        st = parse_runner_log(WARN_LOG)
        self.assertEqual(st["last_warn"], "헤드셋 ws msg 없음")

    def test_ws_disconnect_wins_when_later(self):
        lines = DUAL_LOG + ["[BridgePoseStore] ws disconnected (msgs=5120)"]
        self.assertEqual(parse_runner_log(lines)["headset_ws"], "disconnected")

    def test_empty(self):
        st = parse_runner_log([])
        self.assertEqual(st["arms"], {})
        self.assertEqual(st["headset_ws"], "unknown")


class TestFsmHelpers(unittest.TestCase):
    def test_count_calibrates(self):
        self.assertEqual(count_calibrates(DUAL_LOG), {"right": 1, "left": 1})
        self.assertEqual(count_calibrates(SINGLE_LOG), {"single": 1})
        self.assertEqual(count_calibrates(WARN_LOG), {})

    def test_soft_fail(self):
        self.assertEqual(find_soft_fail(WARN_LOG), "헤드셋 ws msg 없음")
        self.assertIsNone(find_soft_fail(DUAL_LOG[:5]))

    def test_hard_fail(self):
        self.assertIn("로봇 PC 무응답", find_hard_fail(HARDFAIL_LOG))
        self.assertIsNone(find_hard_fail(DUAL_LOG))


class TestSenderCamStats(unittest.TestCase):
    def test_parse(self):
        cams = parse_sender_cam_stats(CAM_LOG)
        self.assertEqual(len(cams), 2)
        self.assertEqual(cams[0], {"name": "head", "fps": 29.8, "kbps": 850,
                                   "lat_ms": 12})
        self.assertEqual(cams[1]["lat_ms"], 25)

    def test_ignores_non_stats(self):
        self.assertEqual(parse_sender_cam_stats([CAM_LOG[0]]), [])


if __name__ == "__main__":
    unittest.main()
