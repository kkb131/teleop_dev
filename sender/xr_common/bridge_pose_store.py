"""BridgePoseStore — Galaxy XR / Quest 3 → Python aiohttp ws bridge.

조종 PC 에서 헤드셋 Chrome 의 webxr_to_pose.html 이 보내는 head / hand pose
JSON 메시지를 multiprocessing.Array 에 저장. arm sender + hand sender 둘 다
같은 인스턴스를 참조해 single source of truth 로 동작.

설계 요점
---------
1. **Singleton**: 한 프로세스 안에서는 한 인스턴스만 (port 충돌 방지). thread-safe.
2. **aiohttp 자체 HTTP/WS server** — vuer 의존성 없음. plain HTTP (localhost
   secure context 예외) 라 cert 불필요.
3. **shared array layout**: TeleVuer 와 동일 — head_pose (16), arm_pose (16),
   hand_positions (75 = 25×3). 외부 코드 (retargeter, sender) 의 reshape 가
   기존 패턴 그대로 동작.
4. **카메라 영상 미지원**: 현재 로봇 PC 가 카메라 publish 안 함. config.yaml 의
   webrtc.enabled=false 가 default 라 webxr_to_pose 가 plane 안 그림 / WebRTC
   connect 시도 안 함. 추후 WebRTC 기능 추가 시 config 만 변경하면 됨.

Cross-reference:
    xr_teleop/scripts/bridge_pose_store.py 가 원본. teleop_dev 이식 시:
      * vuer-specific kwargs (display_fps, display_mode, zmq, webrtc,
        webrtc_url, cert_file, key_file) 는 받기만 하고 noop — 외부 코드
        호환 명목, 미사용
      * config.yaml 위치 src/teleop_dev/sender/xr_common/config.yaml
      * HTML asset 위치 src/teleop_dev/sender/xr_common/assets/webxr_to_pose.html
      * 영상 path 는 모두 noop (render_to_xr → pass)
"""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from multiprocessing import Array, Value
from pathlib import Path
from typing import Literal, Optional

import numpy as np

try:
    from aiohttp import web, WSMsgType
except ImportError as e:
    raise ImportError(
        "aiohttp 필요. 'conda env update -f environment.yaml' 또는 'pip install aiohttp'"
    ) from e

try:
    import yaml
except ImportError as e:
    raise ImportError("PyYAML 필요. 'pip install pyyaml'") from e


# ── config 로딩 ──────────────────────────────────────────────────────────
_CONFIG_PATH = Path(os.environ.get(
    "XR_BRIDGE_CONFIG",
    str(Path(__file__).resolve().parent / "config.yaml"),
))

_DEFAULT_CONFIG = {
    "ws": {"port": 8013},
    "webrtc": {
        "enabled": False,   # 카메라 publisher 없음 — 기본 비활성
        "host": "localhost",
        "ports": {"head": 60001, "left_wrist": 60002, "right_wrist": 60003},
    },
    "render": {
        "plane_distance_m": 1.0,
        "plane_width_m": 1.6,
        "plane_height_m": 0.9,
    },
}


def load_config() -> dict:
    """yaml 로딩 + missing key 는 default 로 채움. 파일 없으면 default 그대로."""
    if not _CONFIG_PATH.exists():
        print(f"[bridge_pose_store] config {_CONFIG_PATH} 없음 — default 사용")
        return _DEFAULT_CONFIG.copy()
    try:
        with _CONFIG_PATH.open() as f:
            loaded = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"[bridge_pose_store] config 로딩 실패: {e} — default 사용")
        return _DEFAULT_CONFIG.copy()
    merged = {}
    for k, dv in _DEFAULT_CONFIG.items():
        v = loaded.get(k, dv)
        if isinstance(dv, dict) and isinstance(v, dict):
            merged[k] = {**dv, **v}
            if k == "webrtc" and "ports" in v and isinstance(v["ports"], dict):
                merged[k]["ports"] = {**dv["ports"], **v["ports"]}
        else:
            merged[k] = v
    return merged


CONFIG = load_config()

# WebXR 25-joint 이름 순서. webxr_to_pose.html 의 JOINT_NAMES 와 1:1 일치.
JOINT_NAMES = [
    'wrist',
    'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
    'index-finger-metacarpal', 'index-finger-phalanx-proximal',
    'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip',
    'middle-finger-metacarpal', 'middle-finger-phalanx-proximal',
    'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip',
    'ring-finger-metacarpal', 'ring-finger-phalanx-proximal',
    'ring-finger-phalanx-intermediate', 'ring-finger-phalanx-distal', 'ring-finger-tip',
    'pinky-finger-metacarpal', 'pinky-finger-phalanx-proximal',
    'pinky-finger-phalanx-intermediate', 'pinky-finger-phalanx-distal', 'pinky-finger-tip',
]
THUMB_TIP_IDX = 4
INDEX_TIP_IDX = 9
MIDDLE_TIP_IDX = 14

# pinch / squeeze threshold (xr_teleop 검증값 그대로)
PINCH_THRESHOLD = 0.01
SQUEEZE_THRESHOLD = 0.07


class BridgePoseStore:
    """Galaxy XR / Quest 3 ws bridge — single source of truth for XR poses.

    Singleton 패턴: 같은 process 에서 두 번째 호출 시 기존 인스턴스 반환.
    """

    _instance: Optional["BridgePoseStore"] = None
    _instance_lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
            return cls._instance

    def __init__(
        self,
        use_hand_tracking: bool = True,
        binocular: bool = False,
        img_shape: Optional[tuple] = None,
        display_fps: float = 30.0,
        display_mode: Literal["immersive", "pass-through", "ego"] = "immersive",
        zmq: bool = False,
        webrtc: bool = False,
        webrtc_url: Optional[str] = None,
        cert_file: Optional[str] = None,
        key_file: Optional[str] = None,
        port: Optional[int] = None,
    ):
        """
        Parameters
        ----------
        use_hand_tracking : bool
            True 면 25-joint hand position + pinch/squeeze 채워짐. False 면
            controller 모드 (트리거 / thumbstick 등).
        binocular / img_shape / display_fps / display_mode / zmq / webrtc /
        webrtc_url / cert_file / key_file :
            vuer-style 호환용. 받지만 사용하지 않음 — BridgePoseStore 는 자체
            HTTP/WS server 라 vuer 자산 불필요. 향후 WebRTC 통합 시 webrtc /
            webrtc_url 만 다시 사용 예정.
        port : Optional[int]
            ws server port. 우선순위: 인자 > env XR_BRIDGE_PORT > config.yaml ws.port > 8013.
        """
        if getattr(self, "_initialized", False):
            return
        self._initialized = True

        self.use_hand_tracking = use_hand_tracking
        self.binocular = binocular
        self.display_fps = display_fps
        self.display_mode = display_mode
        self.zmq = zmq
        self.webrtc = webrtc
        self.webrtc_url = webrtc_url

        if img_shape is None:
            img_shape = (480, 640)
        self.img_shape = (img_shape[0], img_shape[1], 3)
        self.img_height = self.img_shape[0]
        self.img_width = self.img_shape[1] // 2 if binocular else self.img_shape[1]
        self.aspect_ratio = self.img_width / self.img_height

        # ── shared variables — TeleVuer 와 100% 동일 layout ─────────────
        # 4×4 SE(3) matrix: column-major flattened (16 floats).
        self.head_pose_shared = Array('d', 16, lock=True)
        self.left_arm_pose_shared = Array('d', 16, lock=True)
        self.right_arm_pose_shared = Array('d', 16, lock=True)

        if self.use_hand_tracking:
            self.left_hand_position_shared = Array('d', 75, lock=True)
            self.right_hand_position_shared = Array('d', 75, lock=True)
            self.left_hand_orientation_shared = Array('d', 25 * 9, lock=True)
            self.right_hand_orientation_shared = Array('d', 25 * 9, lock=True)
            self.left_hand_pinch_shared = Value('b', False, lock=True)
            self.left_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.left_hand_squeeze_shared = Value('b', False, lock=True)
            self.left_hand_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.right_hand_pinch_shared = Value('b', False, lock=True)
            self.right_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.right_hand_squeeze_shared = Value('b', False, lock=True)
            self.right_hand_squeezeValue_shared = Value('d', 0.0, lock=True)
        else:
            self.left_ctrl_trigger_shared = Value('b', False, lock=True)
            self.left_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.left_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.left_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.left_ctrl_aButton_shared = Value('b', False, lock=True)
            self.left_ctrl_bButton_shared = Value('b', False, lock=True)
            self.right_ctrl_trigger_shared = Value('b', False, lock=True)
            self.right_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.right_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.right_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.right_ctrl_aButton_shared = Value('b', False, lock=True)
            self.right_ctrl_bButton_shared = Value('b', False, lock=True)

        # ws message stats (debug)
        self._msg_count = 0
        self._head_msg_count = 0
        self._hand_msg_count = 0
        self._last_msg_time = 0.0
        self._stats_lock = threading.Lock()

        # ── start ws server in background thread ────────────────────────
        # 우선순위: 인자 > env > config.yaml > 8013
        if port is not None:
            self._port = int(port)
        else:
            self._port = int(os.environ.get("XR_BRIDGE_PORT", str(CONFIG["ws"]["port"])))
        self._server_ready = threading.Event()
        self._server_thread = threading.Thread(
            target=self._server_thread_main, name="BridgePoseStoreWS", daemon=True
        )
        self._server_thread.start()
        if not self._server_ready.wait(timeout=5.0):
            print(f"[BridgePoseStore] WARN: server start timeout (port={self._port})")
        else:
            print(f"[BridgePoseStore] ready: http://localhost:{self._port}/  "
                  f"(WS: /pose)")
            print(f"[BridgePoseStore] adb reverse tcp:{self._port} tcp:{self._port} 필요")
            if not CONFIG.get("webrtc", {}).get("enabled", False):
                print("[BridgePoseStore] webrtc.enabled=false — 영상 plane 비활성 "
                      "(현재 로봇 PC 가 카메라 publisher 미제공. 추후 WebRTC 구현 예정)")

    # ── ws server (aiohttp on thread) ──────────────────────────────────
    def _server_thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve())
        except Exception as e:
            print(f"[BridgePoseStore] server thread error: {e}")

    async def _serve(self) -> None:
        app = web.Application()
        app.router.add_get('/', self._index_handler)
        app.router.add_get('/pose', self._ws_handler)
        app.router.add_get('/config', self._config_handler)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self._port)
        await site.start()
        self._server_ready.set()
        while True:
            await asyncio.sleep(3600)

    async def _index_handler(self, _request: web.Request) -> web.FileResponse:
        p = Path(__file__).resolve().parent / "assets" / "webxr_to_pose.html"
        if not p.exists():
            return web.Response(status=404, text=f"missing {p}")
        return web.FileResponse(p)

    async def _config_handler(self, _request: web.Request) -> web.Response:
        return web.json_response(CONFIG)

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        print(f"[BridgePoseStore] ws client connected: {request.remote}", flush=True)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        self._handle_message(json.loads(msg.data))
                    except Exception:
                        # 매 frame 송신되므로 1회 실패는 silent skip
                        pass
                elif msg.type == WSMsgType.ERROR:
                    print(f"[BridgePoseStore] ws error: {ws.exception()}", flush=True)
                    break
        finally:
            print(f"[BridgePoseStore] ws disconnected (msgs={self._msg_count})",
                  flush=True)
        return ws

    def _handle_message(self, payload: dict) -> None:
        t = payload.get("type")
        if t == "head":
            self._update_head(payload["matrix"])
            with self._stats_lock:
                self._head_msg_count += 1
        elif t == "hand":
            self._update_hand(
                payload["handedness"],
                payload["wrist"],
                payload["positions"],
                payload.get("orientations"),
            )
            with self._stats_lock:
                self._hand_msg_count += 1
        with self._stats_lock:
            self._msg_count += 1
            self._last_msg_time = time.perf_counter()

    def _update_head(self, matrix: list) -> None:
        with self.head_pose_shared.get_lock():
            self.head_pose_shared[:] = matrix

    def _update_hand(self, handedness: str, wrist: list, positions: list,
                     orientations: Optional[list]) -> None:
        if not self.use_hand_tracking:
            return
        if handedness == "left":
            arm_shared = self.left_arm_pose_shared
            pos_shared = self.left_hand_position_shared
            ori_shared = self.left_hand_orientation_shared
            pinch_shared = self.left_hand_pinch_shared
            pinchValue_shared = self.left_hand_pinchValue_shared
            squeeze_shared = self.left_hand_squeeze_shared
            squeezeValue_shared = self.left_hand_squeezeValue_shared
        elif handedness == "right":
            arm_shared = self.right_arm_pose_shared
            pos_shared = self.right_hand_position_shared
            ori_shared = self.right_hand_orientation_shared
            pinch_shared = self.right_hand_pinch_shared
            pinchValue_shared = self.right_hand_pinchValue_shared
            squeeze_shared = self.right_hand_squeeze_shared
            squeezeValue_shared = self.right_hand_squeezeValue_shared
        else:
            return

        with arm_shared.get_lock():
            arm_shared[:] = wrist

        flat_pos = np.array(positions, dtype=np.float64).reshape(-1)
        with pos_shared.get_lock():
            pos_shared[:] = flat_pos

        if orientations is not None:
            flat_ori = np.array(orientations, dtype=np.float64).reshape(-1)
            with ori_shared.get_lock():
                ori_shared[:] = flat_ori

        pos25 = flat_pos.reshape(25, 3)
        pinch_d = float(np.linalg.norm(pos25[THUMB_TIP_IDX] - pos25[INDEX_TIP_IDX]))
        squeeze_d = float(np.linalg.norm(pos25[THUMB_TIP_IDX] - pos25[MIDDLE_TIP_IDX]))
        with pinch_shared.get_lock():
            pinch_shared.value = pinch_d < PINCH_THRESHOLD
        with pinchValue_shared.get_lock():
            pinchValue_shared.value = pinch_d
        with squeeze_shared.get_lock():
            squeeze_shared.value = squeeze_d < SQUEEZE_THRESHOLD
        with squeezeValue_shared.get_lock():
            squeezeValue_shared.value = squeeze_d

    # ── properties — TeleVuer 와 100% 동일 시그니처 ─────────────────────
    @property
    def head_pose(self) -> np.ndarray:
        with self.head_pose_shared.get_lock():
            return np.array(self.head_pose_shared[:]).reshape(4, 4, order='F')

    @property
    def left_arm_pose(self) -> np.ndarray:
        with self.left_arm_pose_shared.get_lock():
            return np.array(self.left_arm_pose_shared[:]).reshape(4, 4, order='F')

    @property
    def right_arm_pose(self) -> np.ndarray:
        with self.right_arm_pose_shared.get_lock():
            return np.array(self.right_arm_pose_shared[:]).reshape(4, 4, order='F')

    @property
    def left_hand_positions(self) -> np.ndarray:
        with self.left_hand_position_shared.get_lock():
            return np.array(self.left_hand_position_shared[:]).reshape(25, 3)

    @property
    def right_hand_positions(self) -> np.ndarray:
        with self.right_hand_position_shared.get_lock():
            return np.array(self.right_hand_position_shared[:]).reshape(25, 3)

    @property
    def left_hand_orientations(self) -> np.ndarray:
        with self.left_hand_orientation_shared.get_lock():
            return np.array(self.left_hand_orientation_shared[:]).reshape(25, 9).reshape(
                25, 3, 3, order='F')

    @property
    def right_hand_orientations(self) -> np.ndarray:
        with self.right_hand_orientation_shared.get_lock():
            return np.array(self.right_hand_orientation_shared[:]).reshape(25, 9).reshape(
                25, 3, 3, order='F')

    @property
    def left_hand_pinch(self) -> bool:
        with self.left_hand_pinch_shared.get_lock():
            return bool(self.left_hand_pinch_shared.value)

    @property
    def left_hand_pinchValue(self) -> float:
        with self.left_hand_pinchValue_shared.get_lock():
            return float(self.left_hand_pinchValue_shared.value)

    @property
    def left_hand_squeeze(self) -> bool:
        with self.left_hand_squeeze_shared.get_lock():
            return bool(self.left_hand_squeeze_shared.value)

    @property
    def left_hand_squeezeValue(self) -> float:
        with self.left_hand_squeezeValue_shared.get_lock():
            return float(self.left_hand_squeezeValue_shared.value)

    @property
    def right_hand_pinch(self) -> bool:
        with self.right_hand_pinch_shared.get_lock():
            return bool(self.right_hand_pinch_shared.value)

    @property
    def right_hand_pinchValue(self) -> float:
        with self.right_hand_pinchValue_shared.get_lock():
            return float(self.right_hand_pinchValue_shared.value)

    @property
    def right_hand_squeeze(self) -> bool:
        with self.right_hand_squeeze_shared.get_lock():
            return bool(self.right_hand_squeeze_shared.value)

    @property
    def right_hand_squeezeValue(self) -> float:
        with self.right_hand_squeezeValue_shared.get_lock():
            return float(self.right_hand_squeezeValue_shared.value)

    # ── methods ────────────────────────────────────────────────────────
    def render_to_xr(self, image) -> None:
        """noop. 영상 통합은 향후 WebRTC 추가 시 활성."""
        return

    def clear_state(self) -> None:
        """Zero shared pose arrays + reset msg stats.

        Use case: sender 프로세스 재시작 직후 호출. 새 process 에서는 shared
        array 가 자연스럽게 zeros 로 할당되지만 (singleton 도 process-local),
        같은 process 내에서 다회 init / 헤드셋 페이지 reload 사이 stale data 차단을
        위해 명시적 reset 제공. 헤드셋 ws 재연결 후 첫 frame 이 들어오기 전까지의
        time window 에서 stale pose 가 origin 으로 잡히는 사고 방지.
        """
        with self.head_pose_shared.get_lock():
            for i in range(16):
                self.head_pose_shared[i] = 0.0
        with self.left_arm_pose_shared.get_lock():
            for i in range(16):
                self.left_arm_pose_shared[i] = 0.0
        with self.right_arm_pose_shared.get_lock():
            for i in range(16):
                self.right_arm_pose_shared[i] = 0.0
        if self.use_hand_tracking:
            with self.left_hand_position_shared.get_lock():
                for i in range(75):
                    self.left_hand_position_shared[i] = 0.0
            with self.right_hand_position_shared.get_lock():
                for i in range(75):
                    self.right_hand_position_shared[i] = 0.0
        with self._stats_lock:
            self._msg_count = 0
            self._head_msg_count = 0
            self._hand_msg_count = 0
            self._last_msg_time = 0.0

    def close(self) -> None:
        """daemon thread 라 main 종료 시 자동 cleanup."""
        return

    def get_stats(self) -> dict:
        with self._stats_lock:
            return {
                "msg_count": self._msg_count,
                "head_msg_count": self._head_msg_count,
                "hand_msg_count": self._hand_msg_count,
                "last_msg_time": self._last_msg_time,
                "port": self._port,
            }

    @property
    def port(self) -> int:
        return self._port


# ── self-test ──────────────────────────────────────────────────────────
def _selftest() -> int:
    """python -m sender.xr_common.bridge_pose_store --selftest

    Smoke test: HTTP+WS server 부팅 + index page 응답 + /config endpoint 응답.
    헤드셋 없이도 로컬 curl 으로 검증 가능.
    """
    import urllib.request
    import urllib.error

    store = BridgePoseStore(use_hand_tracking=True)
    port = store.port

    # 1) /config endpoint
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/config", timeout=2.0) as r:
            cfg = json.loads(r.read().decode())
        print(f"[selftest] /config OK: ws.port={cfg['ws']['port']} webrtc.enabled={cfg['webrtc']['enabled']}")
    except Exception as e:
        print(f"[selftest] /config FAIL: {e}")
        return 1

    # 2) index page (HTML)
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2.0) as r:
            body = r.read().decode("utf-8", errors="replace")
        if "WebXR" in body and "wsSend" in body:
            print(f"[selftest] / (HTML) OK: {len(body)} bytes, WebXR/wsSend 발견")
        else:
            print(f"[selftest] / (HTML) FAIL: HTML payload 비정상")
            return 2
    except Exception as e:
        print(f"[selftest] / FAIL: {e}")
        return 2

    # 3) shared array 초기값 확인
    head = store.head_pose
    assert head.shape == (4, 4), f"head_pose shape: {head.shape}"
    rh = store.right_hand_positions
    assert rh.shape == (25, 3), f"right_hand_positions shape: {rh.shape}"
    print(f"[selftest] shared arrays OK: head=(4,4) right_hand=(25,3)")

    print(f"[selftest] PASS — server is running at http://localhost:{port}/")
    print(f"[selftest] 헤드셋 Chrome 에서 http://localhost:{port}/ 접속해 Enter VR/AR 시험")
    return 0


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        rc = _selftest()
        sys.exit(rc)
    # default: launch + wait
    store = BridgePoseStore(use_hand_tracking=True)
    print(f"[bridge_pose_store] running on http://localhost:{store.port}/  (Ctrl+C 종료)")
    try:
        while True:
            time.sleep(2.0)
            stats = store.get_stats()
            print(f"[bridge_pose_store] msgs={stats['msg_count']} "
                  f"head={stats['head_msg_count']} hand={stats['hand_msg_count']}")
    except KeyboardInterrupt:
        print("\n[bridge_pose_store] stopped.")
