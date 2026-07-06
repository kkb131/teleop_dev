"""Camera streaming protocol — Robot PC → Operator PC → browser/headset.

Robot PC 의 D405 컬러 프레임(JPEG)을 두 구간으로 나눠 전달한다:

  1. ZMQ PUB-SUB (Robot PC → Operator PC, TCP 9873)
     multipart 3-frame: [topic, header JSON, JPEG bytes]
       Frame 0: topic       — 카메라 이름 utf-8 (예: b"head"). SUB prefix filter 용.
       Frame 1: header JSON — {"v":1,"name":"head","seq":123,"ts":...,
                               "w":640,"h":480,"fps":30,"q":80}
       Frame 2: jpeg        — cv2.imencode 출력 bytes.

  2. WebSocket binary (Operator PC → browser / WebXR headset, TCP 8014)
     카메라 1~3개를 WS 연결 1개로 다중화. 고정 12-byte 헤더 + JPEG:
       byte 0     : u8  protocol version (1)
       byte 1     : u8  cam index (0..2 — /config 의 cameras 리스트 인덱스)
       bytes 2-3  : u16 LE seq (wrapping)
       bytes 4-11 : f64 LE capture timestamp (epoch s)
       bytes 12+  : JPEG
     JS 측은 DataView 로 파싱 (webxr_to_pose.html / cam_view.html 참조).

Dependencies: stdlib only (no numpy, no cv2, no zmq) — protocol 패키지 규칙.
"""

import json
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

CAM_PROTOCOL_VERSION = 1
DEFAULT_CAM_ZMQ_PORT = 9873    # 987x 시리즈 (arm 9871, hand 9872 에 이어)
DEFAULT_CAM_HTTP_PORT = 8014   # 801x 시리즈 (BridgePoseStore 8013 에 이어)

# WS binary header: version(u8) + cam_index(u8) + seq(u16 LE) + ts(f64 LE)
_WS_HEADER_FMT = "<BBHd"
WS_HEADER_SIZE = struct.calcsize(_WS_HEADER_FMT)  # 12


@dataclass
class CamFrame:
    """ZMQ multipart 를 unpack 한 카메라 1프레임."""
    name: str            # 카메라 이름 (= ZMQ topic)
    seq: int             # 프레임 시퀀스 (카메라별 단조 증가)
    ts: float            # capture timestamp (epoch s, robot PC 시계)
    width: int
    height: int
    fps: int             # 설정 fps (실측 아님)
    quality: int         # JPEG quality 1-100
    jpeg: bytes


# ---------------------------------------------------------------------------
# ZMQ multipart (Robot PC → Operator PC)
# ---------------------------------------------------------------------------

def pack_frame(
    name: str,
    seq: int,
    ts: float,
    width: int,
    height: int,
    fps: int,
    quality: int,
    jpeg: bytes,
) -> List[bytes]:
    """multipart 3-frame [topic, header JSON, jpeg] 생성. zmq send_multipart 용."""
    header = {
        "v": CAM_PROTOCOL_VERSION,
        "name": name,
        "seq": int(seq),
        "ts": float(ts),
        "w": int(width),
        "h": int(height),
        "fps": int(fps),
        "q": int(quality),
    }
    return [
        name.encode("utf-8"),
        json.dumps(header, separators=(",", ":")).encode("utf-8"),
        jpeg,
    ]


def unpack_frame(parts: List[bytes]) -> Optional[CamFrame]:
    """recv_multipart 결과를 CamFrame 으로. 파싱 실패 시 None."""
    if len(parts) != 3:
        return None
    try:
        d = json.loads(parts[1])
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    if d.get("v", 0) != CAM_PROTOCOL_VERSION:
        return None
    try:
        name = str(d["name"])
        if parts[0].decode("utf-8") != name:
            return None
        return CamFrame(
            name=name,
            seq=int(d["seq"]),
            ts=float(d["ts"]),
            width=int(d["w"]),
            height=int(d["h"]),
            fps=int(d.get("fps", 0)),
            quality=int(d.get("q", 0)),
            jpeg=parts[2],
        )
    except (KeyError, TypeError, ValueError, UnicodeDecodeError):
        return None


# ---------------------------------------------------------------------------
# WS binary (Operator PC → browser / headset)
# ---------------------------------------------------------------------------

def pack_ws_frame(cam_index: int, seq: int, ts: float, jpeg: bytes) -> bytes:
    """12-byte 헤더 + JPEG. aiohttp ws.send_bytes 용."""
    return struct.pack(
        _WS_HEADER_FMT,
        CAM_PROTOCOL_VERSION,
        cam_index & 0xFF,
        seq & 0xFFFF,
        float(ts),
    ) + jpeg


def unpack_ws_frame(data: bytes) -> Optional[Tuple[int, int, float, bytes]]:
    """WS binary → (cam_index, seq, ts, jpeg). 파싱 실패 시 None."""
    if len(data) < WS_HEADER_SIZE:
        return None
    version, cam_index, seq, ts = struct.unpack_from(_WS_HEADER_FMT, data)
    if version != CAM_PROTOCOL_VERSION:
        return None
    return cam_index, seq, ts, data[WS_HEADER_SIZE:]
