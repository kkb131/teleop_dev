"""조종 PC cam HTTP/WS 서버 (aiohttp, tcp 8014).

BridgePoseStore(8013) 와 같은 background-thread + 자체 event loop 패턴.
프로세스가 다르므로 (sender.cam.main ↔ run_xr_teleop) 라우트 통합 대신
별도 포트 + /config 디스커버리로 연결한다.

라우트:
    GET  /               — assets/cam_view.html (브라우저 그리드 뷰)
    GET  /ws             — WS binary push (12-byte 헤더 + JPEG, latest-only)
    GET  /config         — 카메라 이름/VR 레이아웃 JSON.
                           ⚠ Access-Control-Allow-Origin: * 필수 —
                           webxr_to_pose.html (origin 8013) 이 cross-origin fetch.
    GET  /snapshot/{name} — 최신 JPEG 1장 (디버그/검증용)
    POST /recenter       — 전 WS 클라이언트에 {"type":"recenter"} 텍스트 브로드캐스트
                           (world_locked anchor 재캡처 트리거)

헤드셋 USB 모드: adb reverse tcp:8014 tcp:8014 필요 (8013 과 별개).
"""

import asyncio
import json
import threading
from pathlib import Path
from typing import Set

from aiohttp import web

from protocol.cam_protocol import pack_ws_frame
from sender.cam.config import SenderCamConfig
from sender.cam.receiver import CamReceiver

_CORS = {"Access-Control-Allow-Origin": "*"}
_ASSETS = Path(__file__).resolve().parent / "assets"


class CamHttpServer:
    """aiohttp 서버 — background thread 에서 자체 loop 로 실행."""

    def __init__(self, cfg: SenderCamConfig, receiver: CamReceiver):
        self._cfg = cfg
        self._receiver = receiver
        self._cameras = list(cfg.zmq.cameras)
        self._port = cfg.http.port
        self._ws_period = 1.0 / max(cfg.http.ws_max_fps, 1)

        self._loop: asyncio.AbstractEventLoop | None = None
        self._clients: Set[web.WebSocketResponse] = set()
        self._server_ready = threading.Event()
        self._thread = threading.Thread(
            target=self._thread_main, name="CamHttpServer", daemon=True)

    def start(self) -> None:
        self._thread.start()
        if not self._server_ready.wait(timeout=5.0):
            print(f"[CamHttpServer] WARN: server start timeout (port={self._port})")
            return
        print(f"[CamHttpServer] ready: http://localhost:{self._port}/  "
              f"(WS: /ws, config: /config)")
        print(f"[CamHttpServer] 헤드셋 USB 모드: adb reverse tcp:{self._port} "
              f"tcp:{self._port} 필요")

    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._serve())
        except Exception as e:
            print(f"[CamHttpServer] server thread error: {e}")

    async def _serve(self) -> None:
        app = web.Application()
        app.router.add_get("/", self._index_handler)
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/config", self._config_handler)
        app.router.add_get("/snapshot/{name}", self._snapshot_handler)
        app.router.add_post("/recenter", self._recenter_handler)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self._port)
        await site.start()
        self._server_ready.set()
        while True:
            await asyncio.sleep(3600)

    # ── handlers ────────────────────────────────────────────────────────
    async def _index_handler(self, _request: web.Request) -> web.FileResponse:
        p = _ASSETS / "cam_view.html"
        if not p.exists():
            return web.Response(status=404, text=f"missing {p}")
        return web.FileResponse(p)

    async def _config_handler(self, _request: web.Request) -> web.Response:
        vr = self._cfg.vr
        payload = {
            "cameras": self._cameras,
            "ws_max_fps": self._cfg.http.ws_max_fps,
            "vr": {
                "mode": vr.mode,
                "plane_distance_m": vr.plane_distance_m,
                "plane_width_m": vr.plane_width_m,
                "plane_height_m": vr.plane_height_m,
                "plane_height_offset_m": vr.plane_height_offset_m,
                "yaw_deg": vr.yaw_deg,
            },
        }
        return web.json_response(payload, headers=_CORS)

    async def _snapshot_handler(self, request: web.Request) -> web.Response:
        name = request.match_info["name"]
        slot = self._receiver.slots.get(name)
        if slot is None:
            return web.Response(status=404, text=f"unknown camera: {name}",
                                headers=_CORS)
        _seq, _ts, jpeg = slot.get()
        if jpeg is None:
            return web.Response(status=404, text=f"no frame yet: {name}",
                                headers=_CORS)
        return web.Response(body=jpeg, content_type="image/jpeg", headers=_CORS)

    async def _recenter_handler(self, _request: web.Request) -> web.Response:
        msg = json.dumps({"type": "recenter"})
        n = 0
        for ws in list(self._clients):
            try:
                await ws.send_str(msg)
                n += 1
            except Exception:
                pass
        print(f"[CamHttpServer] recenter broadcast → {n} client(s)")
        return web.json_response({"sent": n}, headers=_CORS)

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._clients.add(ws)
        print(f"[CamHttpServer] ws client connected: {request.remote} "
              f"(total={len(self._clients)})")

        # 클라이언트별 push task — seq 가 전진한 카메라만 latest-only 송신.
        # 느린 클라이언트는 send_bytes await 로 자연 페이싱 (중간 프레임 skip).
        last_seq = {name: -1 for name in self._cameras}
        push_task = asyncio.create_task(self._push_loop(ws, last_seq))
        try:
            async for _msg in ws:
                pass  # 클라이언트→서버 메시지는 현재 미사용
        finally:
            push_task.cancel()
            self._clients.discard(ws)
            print(f"[CamHttpServer] ws disconnected "
                  f"(total={len(self._clients)})")
        return ws

    async def _push_loop(self, ws: web.WebSocketResponse,
                         last_seq: dict) -> None:
        try:
            while not ws.closed:
                for idx, name in enumerate(self._cameras):
                    seq, ts, jpeg = self._receiver.slots[name].get()
                    if jpeg is None or seq == last_seq[name]:
                        continue
                    last_seq[name] = seq
                    await ws.send_bytes(pack_ws_frame(idx, seq, ts, jpeg))
                await asyncio.sleep(self._ws_period)
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        except Exception as e:
            print(f"[CamHttpServer] push loop error: {e}")

    @property
    def port(self) -> int:
        return self._port
