# D405 카메라 스트리밍 가이드 (robot/cam + sender/cam)

로봇 PC 의 RealSense D405 컬러 영상(1~3대)을 조종 PC 로 전송하고,
브라우저(HTTP)와 VR(기존 WebXR 텔레옵 세션)에서 스트리밍하는 기능.

## 구조

```
로봇 PC                          조종 PC                        헤드셋 / 브라우저
─────────                        ─────────                      ─────────────────
D405 ×N ─→ robot.cam.main        sender.cam.main
           (JPEG 인코딩)          ├ CamReceiver (ZMQ SUB)
           ZMQ PUB :9873 ──TCP──→ ├ CamHttpServer :8014
                                  │   GET /          ────────→ 브라우저 그리드 뷰
                                  │   GET /ws (WS-JPEG) ─────→ webxr_to_pose.html
                                  │   GET /config     ────────→   (VR plane 렌더)
                                  │   GET /snapshot/{name}
                                  │   POST /recenter
                                  └ BridgePoseStore :8013 (pose, 기존)
```

- **전송 포맷**: `protocol/cam_protocol.py`
  - ZMQ multipart `[topic, header JSON, JPEG]` (topic = 카메라 이름)
  - WS binary: 12-byte 헤더(version u8, cam index u8, seq u16, ts f64) + JPEG
- **latest-only**: PUB `SNDHWM=1` + SUB `RCVHWM=1` + drain. 느린 쪽은 프레임을
  건너뛰지 큐잉하지 않는다 (CONFLATE 은 multipart/다중 topic 과 비호환이라 미사용).
- **WebRTC 미사용 이유**: Galaxy XR USB 모드는 `adb reverse`(TCP 전용) —
  UDP 기반 WebRTC 가 통과 불가. WS-JPEG 는 TCP 라 USB/WiFi 모두 동작.

## 로봇 PC

```bash
# 연결된 D405 시리얼 확인
python3 -m robot.cam.list_cameras

# 기본 config (robot/cam/config/default.yaml) 로 실행
python3 -m robot.cam.main

# 시리얼 직접 지정 (이름 cam0, cam1 자동 부여)
python3 -m robot.cam.main --serials 218622270123 218622270456

# 해상도/품질 override
python3 -m robot.cam.main --width 848 --height 480 --fps 30 --jpeg-quality 70
```

`robot/cam/config/default.yaml` 에서 카메라 개수/이름/시리얼/해상도/fps/JPEG 품질 설정.
**카메라 name = ZMQ topic** — 조종 PC 의 `sender/cam/config.yaml` `zmq.cameras` 와
이름·순서가 일치해야 한다.

Docker: `docker/robot/run_container.sh` 가 `/dev/bus/usb` 를 마운트하므로 D405 접근 가능.
D405 3대 동시 사용 시 USB 대역폭 주의 — 컬러 전용이라 640×480@30 ×3 은 무난,
그 이상 해상도는 허브 분산 권장.

## 조종 PC

```bash
python3 -m sender.cam.main --robot-ip <ROBOT_IP>

# 카메라 목록/고정 모드 override
python3 -m sender.cam.main --robot-ip <ROBOT_IP> \
    --cameras head left_wrist --mode world_locked
```

- 브라우저 뷰: `http://localhost:8014/` (그리드 + fps/latency 오버레이 + Recenter 버튼)
- 스냅샷: `http://localhost:8014/snapshot/head` (최신 JPEG 1장, 디버그)

`sender/cam/config.yaml` 의 `vr` 섹션이 VR 표시 파라미터:

| 키 | 의미 |
|---|---|
| `mode` | `head_locked`(화면이 머리를 따라옴) / `world_locked`(공간에 고정) |
| `plane_distance_m` | 시점으로부터 plane 거리 |
| `plane_width_m` / `plane_height_m` | plane 크기 (640×480 → 1.06×0.8 이 4:3) |
| `plane_height_offset_m` | 수직 오프셋 |
| `yaw_deg` | 카메라별 좌우 배치 각도 (cameras 순서와 1:1, `[0,-40,40]` = 중앙/좌/우) |

## VR (기존 텔레옵 세션 통합)

헤드셋은 immersive WebXR 세션을 1개만 실행할 수 있으므로, 영상은
`webxr_to_pose.html` (BridgePoseStore :8013 이 서빙) 세션 안에 plane 으로 렌더된다.

1. `sender/xr_common/config.yaml` 에서 `cam.enabled: true` 설정
2. 조종 PC: `python3 -m sender.cam.main --robot-ip <ROBOT_IP>` 실행 (8013 텔레옵과 별도 프로세스)
3. USB 모드(Galaxy XR)는 reverse tunnel 2개 필요:
   ```bash
   adb reverse tcp:8013 tcp:8013   # pose bridge (기존)
   adb reverse tcp:8014 tcp:8014   # cam 서버 (추가)
   ```
4. 헤드셋 Chrome 에서 `http://localhost:8013/` → Enter VR

URL 쿼리 override: `?cam_off=1`(영상 비활성), `?cam_host=` `?cam_port=`,
`?cam_mode=head_locked|world_locked`, `?plane_distance=` `?plane_width=` `?plane_height=`

### world_locked (화면 고정) 동작

- 세션의 첫 유효 head pose 에서 yaw+위치만 뽑아 anchor 캡처 (중력 정렬 — plane 이 기울지 않음)
- 이후 머리를 움직여도 plane 은 그 자리에 고정
- 재정렬: 브라우저 뷰(8014)의 **Recenter VR** 버튼 또는
  `curl -X POST http://localhost:8014/recenter` → 다음 프레임에서 anchor 재캡처.
  세션을 나갔다 다시 들어가도 재정렬된다.

## 테스트 (D405·헤드셋 없이)

```bash
# 1) 셀프테스트
python3 -m robot.cam.main --selftest       # 합성 2캠 → 인라인 SUB 검증
python3 -m sender.cam.main --selftest      # 인라인 PUB → slot/config/snapshot/ws 검증

# 2) 로컬 E2E — 터미널 2개
python3 -m robot.cam.main --synthetic 2                          # T1
python3 -m sender.cam.main --robot-ip 127.0.0.1 --cameras syn0 syn1   # T2
# 브라우저에서 http://localhost:8014/ — 컬러바 2개 + 시계 갱신 확인

# 3) WebXR 페이지 (헤드셋 불필요, 데스크톱 Chrome)
# sender/xr_common/config.yaml cam.enabled: true 후
python3 -m sender.xr_common.bridge_pose_store
# http://localhost:8013/ — 진단 pane 에 "cam config loaded" + "(no session) cam.recv=.../s"
```

합성 카메라 프레임의 벽시계 텍스트로 대략적인 glass-to-glass 지연을 육안 측정할 수 있다.

## 트러블슈팅

| 증상 | 확인 |
|---|---|
| 조종 PC 수신 0fps | 로봇 PC 방화벽 9873, `--robot-ip`, 카메라 이름 일치 여부 |
| 헤드셋에서 "cam config fetch FAIL" | `adb reverse tcp:8014 tcp:8014` 누락, sender.cam 미실행 |
| VR 에 plane 안 보임 | `xr_common/config.yaml` `cam.enabled: true` 여부, 진단 pane 의 cam 로그 |
| latency 값이 이상함 | 로봇/조종 PC 시계 차 포함 (NTP 동기 필요) |
| D405 open 실패 | `python3 -m robot.cam.list_cameras`, USB3 케이블/허브, 다른 프로세스 점유 |
