# sender/cam — 카메라 수신 + 브라우저/VR 서빙 (조종 PC)

로봇 PC `robot/cam` 의 ZMQ 스트림(tcp 9873)을 구독해 두 곳으로 서빙한다:

1. **브라우저 그리드 뷰** — `http://localhost:8014/`
2. **VR (기존 WebXR 텔레옵 세션)** — `webxr_to_pose.html` 이 이 서버의
   `/config` 와 `/ws` 에 접속해 카메라 plane 을 렌더

```
ZMQ SUB :9873 ─→ CamReceiver (카메라별 LatestSlot, 최신 프레임만 유지)
                  └→ CamHttpServer :8014
                      ├ GET  /               브라우저 그리드 뷰 (cam_view.html)
                      ├ GET  /ws             WS binary push (12B 헤더 + JPEG)
                      ├ GET  /config         카메라 목록 + VR 레이아웃 (CORS 허용)
                      ├ GET  /snapshot/{name} 최신 JPEG 1장 (디버그)
                      └ POST /recenter       world_locked anchor 재정렬 브로드캐스트
```

## 요구사항

```bash
pip install pyzmq aiohttp pyyaml
# 또는: conda env update -f environment.yaml
```

## 빠른 시작

```bash
cd src/teleop_dev

# 로봇 PC 에서 robot.cam.main 이 실행 중이라는 전제
python3 -m sender.cam.main --robot-ip 192.168.0.2

# 브라우저에서 열기
#   http://localhost:8014/   ← 그리드 뷰 + fps/지연 오버레이 + Recenter 버튼
```

정상 동작 시 2초마다 통계 출력:

```
[CamReceiver] SUB connected: tcp://192.168.0.2:9873 cameras=['head']
[CamHttpServer] ready: http://localhost:8014/  (WS: /ws, config: /config)
[sender.cam] head: 30.0fps 45KB/s lat=18ms
```

`lat` 는 (조종 PC 수신 시각 − 로봇 PC capture ts) — 두 PC 시계 차가 포함되므로
NTP 동기 상태에서만 의미 있다.

## 설정 (config.yaml)

```yaml
zmq:
  robot_ip: 192.168.0.2     # 로봇 PC IP
  port: 9873                # robot/cam 의 stream.port 와 일치
  cameras: [head]           # ⚠ robot 측 cameras[].name 과 이름 일치 필요
                            #   순서 = cam index = vr.yaw_deg 인덱스

http:
  port: 8014
  ws_max_fps: 30            # WS 클라이언트당 push 상한

vr:
  mode: head_locked         # head_locked | world_locked
  plane_distance_m: 1.0     # 시점(anchor)으로부터 plane 거리
  plane_width_m: 1.06       # 640x480 → 4:3
  plane_height_m: 0.8
  plane_height_offset_m: 0.0
  yaw_deg: [0, -40, 40]     # 카메라별 좌우 배치 각도 (+=왼쪽)
```

### VR 파라미터 의미

| 키 | 설명 |
|---|---|
| `mode: head_locked` | 화면이 머리를 따라옴 (항상 정면) |
| `mode: world_locked` | 세션 시작 시점 위치에 화면 **고정** — 머리를 돌려도 그 자리에 있음 |
| `plane_distance_m` | 화면까지 거리 (m). 크면 멀리, 작으면 가까이 |
| `plane_width_m` / `plane_height_m` | 화면 크기 (m). 카메라 종횡비에 맞출 것 |
| `yaw_deg` | 화면 개수만큼의 배치 각도. `[0, -40, 40]` = 1번 중앙, 2번 오른쪽 40°, 3번 왼쪽 40° |

표시되는 **화면 개수 = `zmq.cameras` 리스트 길이** (1~3개).

## CLI 옵션

| 옵션 | 설명 |
|---|---|
| `--config PATH` | YAML config 경로 (기본: `sender/cam/config.yaml`) |
| `--robot-ip IP` | 로봇 PC IP |
| `--port N` | ZMQ SUB 포트 (기본 9873) |
| `--http-port N` | HTTP/WS 포트 (기본 8014) |
| `--cameras N1 [N2 N3]` | 구독 카메라 이름 목록 |
| `--mode head_locked\|world_locked` | VR 고정 모드 |
| `--selftest` | 인라인 PUB → 수신/config/snapshot/ws 전체 경로 검증 |

### 예시

```bash
# 3캠 + 화면 고정 모드
python3 -m sender.cam.main --robot-ip 192.168.0.2 \
    --cameras head left_wrist right_wrist --mode world_locked

# 로컬 테스트 (로봇 PC 대신 같은 PC 에서 합성 카메라)
python3 -m robot.cam.main --synthetic 2                             # 터미널 1
python3 -m sender.cam.main --robot-ip 127.0.0.1 --cameras syn0 syn1 # 터미널 2
```

## VR 에서 보기 (기존 텔레옵 세션 통합)

헤드셋은 immersive WebXR 세션을 1개만 실행할 수 있으므로 영상은 포즈 텔레옵
페이지(`http://localhost:8013/`) 세션 안에 렌더된다.

1. `sender/xr_common/config.yaml` 에서 `cam.enabled: true`
2. `python3 -m sender.cam.main --robot-ip <ROBOT_IP>` 실행 (텔레옵과 별도 프로세스)
3. Galaxy XR USB 모드는 reverse tunnel **2개** 필요:
   ```bash
   adb reverse tcp:8013 tcp:8013   # pose bridge (기존)
   adb reverse tcp:8014 tcp:8014   # cam 서버 (추가)
   ```
4. 헤드셋 Chrome → `http://localhost:8013/` → 진단 pane 에
   `cam config loaded: N cams` 확인 → Enter VR

URL 쿼리로 임시 override 가능 (설정 파일 수정 없이):
`?cam_off=1` `?cam_host=` `?cam_port=` `?cam_mode=world_locked`
`?plane_distance=1.5` `?plane_width=1.6` `?plane_height=0.9`

### world_locked 재정렬 (Recenter)

anchor 는 세션의 첫 유효 head pose 에서 yaw+위치만 캡처된다 (중력 정렬 —
화면이 기울지 않음). 화면 위치를 다시 잡으려면:

- 브라우저 뷰(`:8014`)의 **Recenter VR** 버튼, 또는
- `curl -X POST http://localhost:8014/recenter`

→ 헤드셋의 다음 프레임에서 현재 머리 위치 기준으로 anchor 재캡처.
세션을 나갔다 다시 들어가도 재정렬된다.

## 검증

```bash
python3 -m sender.cam.main --selftest    # PASS 확인 (로봇 PC 불필요)
```

## 트러블슈팅

| 증상 | 확인 |
|---|---|
| 수신 0fps | 로봇 PC 에서 robot.cam.main 실행 중인지, 방화벽 9873, 카메라 이름 일치 (`--cameras` vs robot 측 name) |
| 브라우저 뷰 검은 화면 | `http://localhost:8014/snapshot/<name>` 으로 프레임 도달 확인 (404 = 미수신) |
| 헤드셋 "cam config fetch FAIL" | `adb reverse tcp:8014 tcp:8014` 누락, sender.cam 미실행 |
| VR 에 plane 안 보임 | `xr_common/config.yaml` `cam.enabled: true` 여부, 8013 진단 pane 의 cam 로그 |
| 지연 값이 비정상 (음수/수 초) | 두 PC 시계 차 — NTP 동기 |

전체 파이프라인 (로봇 PC / 프로토콜 포함): `docs/cam_streaming_guide.md`
