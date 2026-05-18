# Phase A · Unit A2 — sender/xr_common (bridge_pose_store + HTML)

## 목적

조종 PC 에서 헤드셋 Chrome 의 webxr_to_pose.html 이 보내는 head / hand pose JSON 을 받아 multiprocessing.Array 에 저장. arm sender + hand sender 둘 다 같은 인스턴스 참조.

## 출처

- 원본: [xr_teleop/scripts/bridge_pose_store.py](../../../../xr_teleop/scripts/bridge_pose_store.py) (550 줄)
- 원본 HTML: [xr_teleop/assets/webxr_to_pose.html](../../../../xr_teleop/assets/webxr_to_pose.html) (513 줄)
- 검증 출처: [xr_teleop/docs/galaxy_xr_ws_bridge_guide.md](../../../../xr_teleop/docs/galaxy_xr_ws_bridge_guide.md) — Galaxy XR Chrome 에서 vuer 0.0.60 client React 가 immersive 진입 후 publish freeze 되는 문제 우회

## 신규 파일

```
src/teleop_dev/sender/xr_common/
├── __init__.py                       # re-export: BridgePoseStore, JOINT_NAMES, load_config
├── bridge_pose_store.py              # aiohttp ws server + Singleton
├── config.yaml                       # ws.port=8013, webrtc.enabled=false (default)
└── assets/
    └── webxr_to_pose.html            # WebXR client (browser 측)
```

## teleop_dev 이식 시 변경점 (vs xr_teleop 원본)

| 항목 | xr_teleop | teleop_dev |
|---|---|---|
| 모듈 경로 | `scripts/bridge_pose_store.py` | `sender/xr_common/bridge_pose_store.py` |
| config 위치 | `scripts/config.yaml` | `sender/xr_common/config.yaml` |
| HTML asset | `assets/webxr_to_pose.html` | `sender/xr_common/assets/webxr_to_pose.html` |
| `webrtc.enabled` default | `true` | **`false`** (현재 teleop_dev 로봇 PC 가 카메라 publisher 미제공) |
| `vuer/televuer` import | 없음 (Singleton + raw aiohttp) | 동일 — 의존성 없음 |
| `cert/key` 인자 | 받고 무시 | 받고 무시 (호환 명목) |
| import root | `setup/bridge_pose_store import BridgePoseStore` (xr_teleop 측 패턴) | `from sender.xr_common import BridgePoseStore` |

## 영상 처리 (현재 비활성)

- `config.yaml` 의 `webrtc.enabled: false` 가 default.
- HTML 측 `_applyConfig()` 가 `CONFIG.webrtc.enabled === false` 면 `WEBRTC_OFF=true` 강제.
- `onFrame()` 에서 `WEBRTC_OFF` 이면 `_uploadVideoFrame()` 과 `_drawVideoPlane()` 을 호출 안 함 → background 만 clear.
- `_connectWebRTC()` 도 `WEBRTC_OFF` 일 때 즉시 early return — cert 신뢰 안내 메시지 안 뜸.
- 추후 로봇 PC 가 카메라 publisher (WebRTC) 추가 시:
  1. `sender/xr_common/config.yaml` 의 `webrtc.enabled` → `true`
  2. `webrtc.host` 를 카메라 publisher host 로 변경
  3. 헤드셋 Chrome 에서 `https://<host>:60001`, `60003` 1회씩 cert 신뢰
  - HTML / Python 측 코드 변경 불필요

## API 호환성 (TeleVuer ↔ BridgePoseStore)

property 시그니처 100% 동일:

- `head_pose` (4, 4) 4×4 SE(3) `local-floor` 좌표계
- `left_arm_pose / right_arm_pose` (4, 4) wrist SE(3)
- `left_hand_positions / right_hand_positions` (25, 3) WebXR 25-joint xyz
- `left_hand_orientations / right_hand_orientations` (25, 3, 3) per-joint rotation matrix
- `left_hand_pinch / right_hand_pinch` bool — thumb-tip ↔ index-tip < 0.01m
- `left_hand_squeeze / right_hand_squeeze` bool — thumb-tip ↔ middle-tip < 0.07m

## 검증

### 단위 1: 모듈 import + selftest (헤드셋 불요)

```bash
cd /workspaces/tamp_ws/src/teleop_dev

# import 확인 + JOINT_NAMES 길이
python3 -c "from sender.xr_common import BridgePoseStore, JOINT_NAMES; print('joints:', len(JOINT_NAMES))"
# → joints: 25

# 자체 selftest: HTTP+WS server 부팅 + /config endpoint + / (HTML) endpoint 응답 확인
python3 -m sender.xr_common.bridge_pose_store --selftest
# 기대 출력:
#   [BridgePoseStore] ready: http://localhost:8013/  (WS: /pose)
#   [selftest] /config OK: ws.port=8013 webrtc.enabled=False
#   [selftest] / (HTML) OK: 18187 bytes, WebXR/wsSend 발견
#   [selftest] shared arrays OK: head=(4,4) right_hand=(25,3)
#   [selftest] PASS
```

### 단위 2: HTTP / curl 직접 확인 (헤드셋 불요)

```bash
# 1터미널
python3 -m sender.xr_common.bridge_pose_store

# 2터미널
curl -s http://localhost:8013/config | python3 -m json.tool
# → {"ws": {"port": 8013}, "webrtc": {"enabled": false, ...}, "render": {...}}

curl -s http://localhost:8013/ | head -20
# → <!doctype html> ... <title>WebXR → pose ws bridge (teleop_dev)</title> ...
```

### 수동 검증 (헤드셋 필요)

[A2_test_guide.md](A2_test_guide.md) 에 별도 정리.

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `ImportError: aiohttp` | env 미갱신 | `conda env update -f environment.yaml --prune` 또는 `pip install 'aiohttp>=3.8'` |
| port 8013 already in use | 이전 selftest 가 daemon thread 로 남음 | `pkill -f bridge_pose_store` 또는 `--port 8014` 옵션 사용 |
| HTML 페이지 안 뜸 | `assets/webxr_to_pose.html` 경로 문제 | bridge_pose_store.py:`_index_handler` 가 가리키는 절대 경로 확인. 보통 패키지 install 안 됐을 때 발생 |
| `[selftest] / FAIL` | webxr_to_pose.html 파일 누락 | `git status` 로 파일 존재 확인. HTML 다시 commit |

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/xr_common/__init__.py`
- `src/teleop_dev/sender/xr_common/bridge_pose_store.py`
- `src/teleop_dev/sender/xr_common/config.yaml`
- `src/teleop_dev/sender/xr_common/assets/webxr_to_pose.html`

수정 없음.
