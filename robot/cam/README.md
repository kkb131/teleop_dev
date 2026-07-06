# robot/cam — D405 컬러 스트리밍 (로봇 PC)

RealSense D405 1~3대의 컬러 영상을 JPEG 인코딩해 ZMQ PUB(tcp **9873**)로 송신한다.
조종 PC 의 `python3 -m sender.cam.main` 이 이 스트림을 구독한다.

```
D405 ×N ─→ CaptureWorker (카메라당 1 스레드, capture → cv2.imencode)
            └→ CamZmqPublisher (ZMQ PUB :9873, topic = 카메라 이름)
```

## 요구사항

```bash
pip install pyzmq opencv-python-headless pyrealsense2 pyyaml
```

Docker 사용 시 `docker/robot/Dockerfile` Layer 3 에 이미 포함.
컨테이너에서 D405 접근은 `docker/robot/run_container.sh` 가 `/dev/bus/usb` 를
마운트하므로 기본 동작한다 (`--no-devices` 옵션 사용 시 불가).

## 빠른 시작

```bash
cd src/teleop_dev

# 1) 연결된 D405 시리얼 확인
python3 -m robot.cam.list_cameras
#   [list_cameras] 2개 장치:
#     serial=218622270123  (Intel RealSense D405)
#     serial=218622270456  (Intel RealSense D405)

# 2) config/default.yaml 의 cameras[].serial 에 시리얼 기입 후 실행
python3 -m robot.cam.main
```

정상 동작 시 2초마다 통계 출력:

```
[robot.cam] PUB bound: tcp://0.0.0.0:9873 (quality=80)
[robot.cam] streaming: head 640x480@30 (serial=218622270123)
[robot.cam] head: 30.0fps 45KB/s
```

## 설정 (config/default.yaml)

```yaml
stream:
  bind_host: 0.0.0.0
  port: 9873           # operator 측 sender/cam/config.yaml zmq.port 와 일치
  jpeg_quality: 80     # 1-100. 640x480 기준 80 ≈ 프레임당 30-60KB

cameras:               # 1~3개
  - name: head         # ⚠ ZMQ topic — operator 측 zmq.cameras 와 이름·순서 일치 필요
    serial: ""          # "" = 첫 번째 연결 장치
    width: 640
    height: 480
    fps: 30
```

다른 파일을 쓰려면 `--config <path>`.

## CLI 옵션

| 옵션 | 설명 |
|---|---|
| `--config PATH` | YAML config 경로 (기본: `robot/cam/config/default.yaml`) |
| `--port N` | ZMQ PUB 포트 override (기본 9873) |
| `--jpeg-quality N` | JPEG 품질 1-100 |
| `--width N` `--height N` `--fps N` | 모든 카메라에 일괄 적용 |
| `--serials S1 [S2 S3]` | 시리얼 직접 지정 — config 의 cameras 를 대체하고 이름은 `cam0..N` 자동 부여 |
| `--synthetic N` | 합성 카메라 N대 (D405 불필요, 이름 `syn0..N`) |
| `--list-cameras` | 연결 장치 나열 후 종료 |
| `--selftest` | 합성 2캠 + 프로세스 내 SUB 검증 (exit 0/1) |

### 예시

```bash
# 시리얼 직접 지정 (이름 cam0, cam1)
python3 -m robot.cam.main --serials 218622270123 218622270456

# 대역폭 절약 (해상도/품질 낮춤)
python3 -m robot.cam.main --width 424 --height 240 --jpeg-quality 60

# D405 없이 합성 2캠 (조종 PC 연동 테스트)
python3 -m robot.cam.main --synthetic 2
```

## 검증

```bash
python3 -m robot.cam.main --selftest          # PASS 확인
python3 -m robot.cam.main --synthetic 2       # 통계 로그에 2캠 × 30fps 확인
```

합성 카메라 프레임에는 이름/카운터/벽시계가 그려져 있어 수신 측에서
스트림 구분과 육안 지연 측정이 가능하다.

## 동작 특성

- **latest-only**: PUB `SNDHWM=1` — 느린 구독자에겐 최신 프레임만 전달 (지연 누적 없음)
- **wire format**: `protocol/cam_protocol.py` 의 multipart `[topic, header JSON, JPEG]`
- 카메라 name 이 그대로 ZMQ topic 이므로 **조종 PC 설정과 이름이 다르면 수신 0fps**
- fps 페이싱: 실제 D405 는 `wait_for_frames` 가 페이싱, 합성 카메라는 monotonic 고정 주기

## 트러블슈팅

| 증상 | 확인 |
|---|---|
| `list_cameras` 에 장치 없음 | USB3 케이블/포트, 다른 프로세스(realsense-viewer 등) 점유, 컨테이너 `--no-devices` 여부 |
| 카메라 open 실패 / fail 카운트 증가 | 시리얼 오타, 해상도/fps 조합이 D405 미지원 (640×480@30 권장) |
| 3대 동시 사용 시 프레임 드랍 | USB 대역폭 한계 — 해상도 낮추거나 허브 분산 (컬러 전용이라 640×480@30 ×3 은 무난) |
| 조종 PC 에서 수신 0fps | 방화벽 9873, 카메라 이름 불일치, `--robot-ip` 확인 |

전체 파이프라인 (조종 PC / VR 포함): `docs/cam_streaming_guide.md`
