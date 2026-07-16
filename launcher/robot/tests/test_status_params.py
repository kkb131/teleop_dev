"""로그 파서 + 파라미터 편집 단위 테스트 (하드웨어 불필요).

실행: python3 -m unittest launcher.robot.tests.test_status_params
"""

import shutil
import tempfile
import unittest
from pathlib import Path

import yaml

from launcher.robot.params import get_params, set_params
from launcher.robot.status import parse_arm_status, parse_cam_stats, strip_ansi

_TELEOP_DIR = Path(__file__).parents[3]


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


class TestParams(unittest.TestCase):
    """left.yaml 사본으로 round-trip 편집 — 주석 보존 + all-or-nothing 검증."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.arm_yaml = Path(self.tmp) / "left.yaml"
        shutil.copy(_TELEOP_DIR / "robot/arm/admittance/config/left.yaml",
                    self.arm_yaml)
        self.cam_yaml = Path(self.tmp) / "cam.yaml"
        shutil.copy(_TELEOP_DIR / "robot/cam/config/default.yaml", self.cam_yaml)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_get_arm_params(self):
        p = get_params(str(self.arm_yaml), "arm")
        self.assertEqual(p["values"]["robot.ip"], "192.168.0.3")
        self.assertEqual(p["values"]["input.unified_port"], 9875)
        self.assertEqual(p["values"]["initial_pose.enabled"], False)
        self.assertTrue(p["restart_required"])

    def test_set_and_preserve_comments(self):
        before = self.arm_yaml.read_text()
        comment_lines = [l for l in before.splitlines() if l.strip().startswith("#")]
        self.assertGreater(len(comment_lines), 5, "left.yaml 주석 전제")

        r = set_params(str(self.arm_yaml), "arm", {
            "safety.max_joint_vel": 0.5,
            "robot.ip": "192.168.0.99",
            "safety.workspace.x": [-1.0, 1.0],
        })
        self.assertTrue(r.ok, r.errors)
        self.assertEqual(sorted(r.saved),
                         ["robot.ip", "safety.max_joint_vel", "safety.workspace.x"])

        after = self.arm_yaml.read_text()
        data = yaml.safe_load(after)
        self.assertEqual(data["safety"]["max_joint_vel"], 0.5)
        self.assertEqual(data["robot"]["ip"], "192.168.0.99")
        self.assertEqual(data["safety"]["workspace"]["x"], [-1.0, 1.0])
        if r.comments_preserved:   # ruamel 설치 환경
            for cl in comment_lines:
                self.assertIn(cl.strip().lstrip("# ").split()[0] if cl.strip("# ") else "",
                              after)
            # 대표 안전 주석 문구 보존 확인
            self.assertIn("첫 가동", after)

    def test_all_or_nothing(self):
        before = self.arm_yaml.read_text()
        r = set_params(str(self.arm_yaml), "arm", {
            "safety.max_joint_vel": 0.4,        # valid
            "safety.max_ee_velocity": 99.0,     # 범위 밖 → 전체 거부
        })
        self.assertFalse(r.ok)
        self.assertIn("safety.max_ee_velocity", r.errors)
        self.assertEqual(self.arm_yaml.read_text(), before, "부분 저장 금지")

    def test_whitelist_reject(self):
        r = set_params(str(self.arm_yaml), "arm", {"ik.damping": 1.0})
        self.assertFalse(r.ok)
        self.assertIn("ik.damping", r.errors)

    def test_enum_and_bool(self):
        r = set_params(str(self.arm_yaml), "arm", {
            "admittance.default_preset": "MEDIUM",
            "initial_pose.enabled": "true",
        })
        self.assertTrue(r.ok, r.errors)
        data = yaml.safe_load(self.arm_yaml.read_text())
        self.assertEqual(data["admittance"]["default_preset"], "MEDIUM")
        self.assertIs(data["initial_pose"]["enabled"], True)
        r2 = set_params(str(self.arm_yaml), "arm",
                        {"admittance.default_preset": "JELLY"})
        self.assertFalse(r2.ok)

    def test_cam_params(self):
        p = get_params(str(self.cam_yaml), "cam")
        self.assertEqual(p["values"]["stream.port"], 9873)
        self.assertEqual(p["values"]["cameras"][0]["name"], "head")

        r = set_params(str(self.cam_yaml), "cam", {
            "stream.jpeg_quality": 90,
            "cameras": [
                {"name": "head", "serial": "", "width": 640, "height": 480, "fps": 30},
                {"name": "wrist", "serial": "S123", "width": 424, "height": 240, "fps": 15},
            ],
        })
        self.assertTrue(r.ok, r.errors)
        data = yaml.safe_load(self.cam_yaml.read_text())
        self.assertEqual(data["stream"]["jpeg_quality"], 90)
        self.assertEqual(len(data["cameras"]), 2)
        self.assertEqual(data["cameras"][1]["serial"], "S123")

        # 이름 중복 거부
        r2 = set_params(str(self.cam_yaml), "cam", {
            "cameras": [{"name": "a"}, {"name": "a"}]})
        self.assertFalse(r2.ok)


if __name__ == "__main__":
    unittest.main()
