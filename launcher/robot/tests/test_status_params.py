"""로그 파서 + 파라미터 편집 단위 테스트 (하드웨어 불필요).

실행: python3 -m unittest launcher.robot.tests.test_status_params
"""

import unittest

from launcher.robot.status import parse_arm_status, parse_cam_stats, strip_ansi


# robot/arm/admittance/main.py _write_status 실제 출력 형식
# (첫 줄에 \x1b[8A 커서 이동, 각 줄 앞 \r, 72폭 ljust)
ARM_BLOCK = [
    "\x1b[8A\r  EE Pos : x= 0.5123  y=-0.1042  z= 0.4310 m" + " " * 20,
    "\r  EE RPY : R=  45.0  P= -30.5  Y=  12.3 deg" + " " * 26,
    "\r  Joints : [  10.5  -60.0   45.2   -5.1   20.0    0.5] deg",
    "\r  Vel    : 0.0123 m/s  |  Speed: 1.0x",
    "\r  Safety : OK  ",
    "\r  E-Stop : off (Space: trigger)",
    "\r  Admit  : ON [MEDIUM] | F: [1.2,0.5,-2.1]N | dx: 12.3mm",
    "\r  Input  : 45ms ago  |  rtde 125Hz",
]

ARM_BLOCK_ESTOP = [
    "\r  Safety : TIMEOUT  input 200ms 초과",
    "\r  E-Stop : !! ACTIVE !! (R: reset)",
]

CAM_LINES = [
    "[robot.cam] streaming: head 640x480@30 (serial=auto)",
    "[robot.cam] head: 29.8fps 850KB/s  wrist: 15.2fps 420KB/s fail=3",
]


class TestStripAnsi(unittest.TestCase):
    def test_removes_cursor_and_cr(self):
        self.assertEqual(strip_ansi("\x1b[8A\r  EE Pos : x"), "EE Pos : x")


class TestParseArmStatus(unittest.TestCase):
    def test_full_block(self):
        st = parse_arm_status(ARM_BLOCK)
        self.assertEqual(st["ee_pos"], [0.5123, -0.1042, 0.4310])
        self.assertEqual(st["ee_rpy_deg"], [45.0, -30.5, 12.3])
        self.assertEqual(st["safety"], "OK")
        self.assertFalse(st["estop_active"])
        self.assertIn("MEDIUM", st["admittance"])
        self.assertEqual(st["input_age_ms"], 45)

    def test_estop_latched(self):
        st = parse_arm_status(ARM_BLOCK + ARM_BLOCK_ESTOP)
        self.assertTrue(st["estop_active"])
        self.assertEqual(st["safety"], "TIMEOUT")

    def test_last_match_wins(self):
        lines = ARM_BLOCK + ["\r  EE Pos : x= 0.9000  y= 0.0000  z= 0.1000 m"]
        st = parse_arm_status(lines)
        self.assertEqual(st["ee_pos"], [0.9, 0.0, 0.1])

    def test_empty(self):
        self.assertEqual(parse_arm_status([]), {})
        self.assertEqual(parse_arm_status(["driver tick 3"]), {})


class TestParseCamStats(unittest.TestCase):
    def test_multi_cam_with_fail(self):
        cams = parse_cam_stats(CAM_LINES)
        self.assertEqual(len(cams), 2)
        self.assertEqual(cams[0], {"name": "head", "fps": 29.8, "kbps": 850, "fail": 0})
        self.assertEqual(cams[1], {"name": "wrist", "fps": 15.2, "kbps": 420, "fail": 3})

    def test_latest_line_wins(self):
        lines = CAM_LINES + ["[robot.cam] head: 10.0fps 100KB/s"]
        cams = parse_cam_stats(lines)
        self.assertEqual(len(cams), 1)
        self.assertEqual(cams[0]["fps"], 10.0)

    def test_ignores_non_stats(self):
        self.assertEqual(parse_cam_stats(["[robot.cam] streaming: head 640x480@30"]), [])
        self.assertEqual(parse_cam_stats([]), [])


if __name__ == "__main__":
    unittest.main()
