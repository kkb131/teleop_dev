"""adb 출력 파서 + config/스키마 단위 테스트 (adb/하드웨어 불필요).

실행: python3 -m unittest launcher.sender.tests.test_adb
"""

import shutil
import tempfile
import unittest
from pathlib import Path

import yaml

from launcher.params_engine import get_params, set_params
from launcher.sender.adb import AdbHelper, parse_devices, parse_reverse_list
from launcher.sender.params_schema import XR_DUAL_SCHEMA
from launcher.sender.parts import load_sender_config

_TELEOP_DIR = Path(__file__).parents[3]

DEVICES_OUT = """\
* daemon not running; starting now at tcp:5037
* daemon started successfully
List of devices attached
R3CX70ABCDE\tdevice
emulator-5554\tunauthorized
0123456789\toffline
"""

REVERSE_OUT_NEW = """\
R3CX70ABCDE tcp:8013 tcp:8013
R3CX70ABCDE tcp:8014 tcp:8014
"""

REVERSE_OUT_OLD = "(reverse) UsbFfs tcp:8013 tcp:8013\n"


class TestAdbParsers(unittest.TestCase):
    def test_parse_devices(self):
        devs = parse_devices(DEVICES_OUT)
        self.assertEqual(len(devs), 3)
        self.assertEqual(devs[0], {"serial": "R3CX70ABCDE", "state": "device"})
        self.assertEqual(devs[1]["state"], "unauthorized")

    def test_parse_devices_empty(self):
        self.assertEqual(parse_devices("List of devices attached\n"), [])
        self.assertEqual(parse_devices(""), [])

    def test_parse_reverse_list(self):
        revs = parse_reverse_list(REVERSE_OUT_NEW)
        self.assertEqual(len(revs), 2)
        self.assertEqual(revs[0], {"serial": "R3CX70ABCDE",
                                   "remote": "tcp:8013", "local": "tcp:8013"})
        revs_old = parse_reverse_list(REVERSE_OUT_OLD)
        self.assertEqual(revs_old[0]["remote"], "tcp:8013")

    def test_missing_binary(self):
        h = AdbHelper(binary="definitely-not-adb-xyz")
        self.assertFalse(h.available())
        r = h._run("devices")
        self.assertFalse(r["ok"])
        self.assertIn("미설치", r["output"])
        self.assertEqual(h.devices(), [])


class TestSenderConfig(unittest.TestCase):
    def test_load_default(self):
        cfg = load_sender_config(str(_TELEOP_DIR / "launcher/sender/config/sender.yaml"))
        self.assertEqual([r.name for r in cfg.runners],
                         ["xr-dual", "xr-single-right", "xr-single-left"])
        self.assertEqual(cfg.adb.reverse_ports, [8013, 8014, 9877])
        self.assertEqual(cfg.session.countdown_s, 4.0)
        self.assertEqual(cfg.launcher.web.port, 9877)
        self.assertEqual(cfg.runner("xr-dual").component, "xr-dual-runner")
        # 러너 컴포넌트는 use_pty 강제
        self.assertTrue(cfg.launcher.get("xr-dual-runner").use_pty)
        # params 대상 해석
        self.assertIn("xr-dual", cfg.params)
        self.assertTrue(cfg.params["xr-dual"]["file"].endswith("xr_dual.yaml"))


class TestXrDualParams(unittest.TestCase):
    """xr_dual.yaml 사본 round-trip — floats3 + 주석 보존."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.yaml = Path(self.tmp) / "xr_dual.yaml"
        shutil.copy(_TELEOP_DIR / "scripts/config/xr_dual.yaml", self.yaml)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_get(self):
        p = get_params(str(self.yaml), XR_DUAL_SCHEMA)
        self.assertEqual(p["values"]["arms.right.port"], 9871)
        self.assertEqual(p["values"]["arms.left.remap_rpy_deg"], [90.0, 0.0, 180.0])
        self.assertEqual(p["values"]["hands.left.port"], 9874)

    def test_set_floats3_and_comments(self):
        before = self.yaml.read_text()
        self.assertIn("#", before)
        r = set_params(str(self.yaml), XR_DUAL_SCHEMA, {
            "arms.left.scale": 0.4,
            "arms.left.remap_rpy_deg": [90, 0, -90],
            "arms.left.workspace.x": [-0.6, 0.6],
        })
        self.assertTrue(r.ok, r.errors)
        data = yaml.safe_load(self.yaml.read_text())
        self.assertEqual(data["arms"]["left"]["scale"], 0.4)
        self.assertEqual(data["arms"]["left"]["remap_rpy_deg"], [90.0, 0.0, -90.0])
        if r.comments_preserved:
            self.assertIn("포트 배치", self.yaml.read_text())   # 상단 주석 보존

    def test_floats3_validation(self):
        r = set_params(str(self.yaml), XR_DUAL_SCHEMA,
                       {"arms.left.remap_rpy_deg": [90, 0]})
        self.assertFalse(r.ok)
        self.assertIn("arms.left.remap_rpy_deg", r.errors)

    def test_whitelist(self):
        r = set_params(str(self.yaml), XR_DUAL_SCHEMA,
                       {"arms.left.remap_matrix": [[1, 0, 0]]})
        self.assertFalse(r.ok)


if __name__ == "__main__":
    unittest.main()
