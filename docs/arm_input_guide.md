# 팔 원격조종 입력장치 가이드

조종 PC에서 로봇 PC로 팔(UR10e) 제어 명령을 전송하는 방법.
3가지 입력장치(Vive Tracker, 키보드, 조이스틱)를 지원한다.

---

## 전체 흐름

```
조종 PC                               로봇 PC
┌─────────────────┐    UDP 9871    ┌──────────────────────┐
│ vive_sender.py  │───────────────→│ UnifiedNetworkInput  │
│ keyboard_sender │                │   ↓                  │
│ joystick_sender │                │ admittance/main.py   │
│                 │                │ impedance/main.py    │
└─────────────────┘                └──────────────────────┘
```

1. 조종 PC: 입력장치 데이터를 읽고 → 로봇 base_link 좌표계의 절대 포즈로 변환
2. UDP 9871: `TeleopPosePacket` (JSON) 전송
3. 로봇 PC: `UnifiedNetworkInput`이 수신 → admittance 또는 impedance 제어 루프에 전달

**초기 핸드셰이크**: sender 시작 시 `query_pose` 전송 → 로봇 PC가 현재 TCP 포즈 응답 → sender가 이 포즈부터 상대 이동 시작.

---

## 1. Vive Tracker (실전 원격조종)

### 사전 요건

- HTC Vive Tracker 3.0 + Base Station
- SteamVR 실행 중 (트래커 녹색 표시)
- Python 패키지:
  ```bash
  pip install openvr numpy pynput pyyaml
  ```

### 캘리브레이션 (최초 1회)

SteamVR 좌표계를 로봇 base_link 좌표계에 맞추는 과정.

```bash
cd /workspaces/tamp_ws/src/teleop_dev

# 1. 연결된 트래커 확인
python3 -m teleop_dev.operator.arm.vive_sender --list-trackers

# 2. 3점 캘리브레이션 실행
python3 -m teleop_dev.operator.arm.calibrate --output my_calibration.json
```

**3점 캘리브레이션 순서**:
1. 트래커를 로봇 base_link **원점**에 놓고 Enter
2. 트래커를 로봇 **+X 방향**으로 이동시키고 Enter
3. 트래커를 로봇 **+Y 방향** (XY 평면)으로 이동시키고 Enter
4. `my_calibration.json` 저장됨 (회전 행렬 R + 평행이동 t)

> 캘리브레이션 없이도 동작하지만, SteamVR Y-up → UR Z-up 기본 변환만 적용됨.

### 실행

```bash
# 기본 실행
python3 -m teleop_dev.operator.arm.vive_sender \
  --target-ip 192.168.0.10 \
  --calibration my_calibration.json

# YAML 설정 파일 사용
python3 -m teleop_dev.operator.arm.vive_sender \
  --config operator/arm/config/default.yaml \
  --target-ip 192.168.0.10

# 특정 트래커 지정
python3 -m teleop_dev.operator.arm.vive_sender \
  --target-ip 192.168.0.10 \
  --tracker-serial LHR-XXXXXXXX
```

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--port` | 9871 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 (Hz) |
| `--calibration` | null | 캘리브레이션 JSON 경로 |
| `--tracker-serial` | 자동 | Vive Tracker 시리얼 번호 |
| `--config` | default.yaml | YAML 설정 파일 |
| `--list-trackers` | - | 감지된 트래커 목록 출력 후 종료 |

### 키보드 제어 (sender 실행 중)

| 키 | 동작 |
|----|------|
| Space | E-Stop (긴급 정지) |
| R | 리셋 (현재 로봇 TCP 포즈로 초기화) |
| Q / Esc | 종료 |
| + / = | 속도 증가 |
| - | 속도 감소 |

### 매핑 모드

- **상대 매핑** (기본): 트래커의 **이동량(delta)**을 로봇에 적용. 손을 내려놓고 다시 들어도 연속 동작.
- 캘리브레이션 변환이 delta에 적용되므로 로봇 좌표계 기준으로 정확히 이동.

### 설정 파일 (`operator/arm/config/default.yaml`)

```yaml
network:
  target_ip: "192.168.0.10"
  port: 9871
  hz: 50

trackers:
  - name: "right_hand"
    serial: null              # null = 첫 번째 트래커 자동 선택
    role: "teleop"

calibration:
  file: null                  # 캘리브레이션 JSON 경로
  samples_per_point: 50       # 캘리브레이션 시 포인트당 평균 샘플 수

mapping_mode: "relative"
```

---

## 2. 키보드 (개발/테스트용)

### 사전 요건

- Linux (termios 필요)
- Python 패키지:
  ```bash
  pip install numpy
  ```

### 실행

```bash
python3 -m teleop_dev.operator.arm.keyboard_sender \
  --target-ip 192.168.0.10

# 스텝 크기 조절
python3 -m teleop_dev.operator.arm.keyboard_sender \
  --target-ip 192.168.0.10 \
  --cart-step 0.01 \
  --rot-step 0.1
```

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--port` | 9871 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 |
| `--cart-step` | 0.005 | 이동 스텝 (m/tick) |
| `--rot-step` | 0.05 | 회전 스텝 (rad/tick) |

### 키 매핑

**이동 (Cartesian)**:

| 키 | 방향 | 키 | 방향 |
|----|------|----|------|
| W | +Y (전진) | S | -Y (후진) |
| A | -X (좌) | D | +X (우) |
| Q | +Z (상) | E | -Z (하) |

**회전**:

| 키 | 방향 | 키 | 방향 |
|----|------|----|------|
| U | +Roll | O | -Roll |
| I | +Pitch | K | -Pitch |
| J | +Yaw | L | -Yaw |

**제어**:

| 키 | 동작 |
|----|------|
| Space | E-Stop (긴급 정지) |
| R | 리셋 |
| X / Esc | 종료 |
| + / = | 속도 증가 |
| - | 속도 감소 |
| T | 어드미턴스 토글 |
| Z | F/T 센서 영점 |

**속도 배율**: `[0.1, 0.2, 0.3, 0.5, 0.8, 1.0]x` (기본: 0.3x). `cart-step`과 `rot-step`에 곱해짐.

---

## 3. 조이스틱/게임패드

### 사전 요건

- USB 게임패드 연결 (Xbox, Logitech 등)
- Python 패키지:
  ```bash
  pip install pygame
  ```

### 실행

```bash
python3 -m teleop_dev.operator.arm.joystick_sender \
  --target-ip 192.168.0.10

# 포트 변경
python3 -m teleop_dev.operator.arm.joystick_sender \
  --target-ip 192.168.0.10 \
  --port 9870
```

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--port` | 9870 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 |

### 전송 데이터

Raw pygame 데이터를 JSON으로 전송:
```json
{
  "axes": [-0.12, 0.85, ...],
  "buttons": [false, true, ...],
  "hat": [0, 1]
}
```

- 축/버튼 수는 연결된 게임패드에 따라 다름
- **Deadzone/스케일링 없음** — 로봇 PC의 `input_handler.py`에서 처리
- 시작 시 자동으로 첫 번째 게임패드 감지

---

## 로봇 PC 수신 설정

로봇 PC에서는 `--input unified` 옵션으로 UDP 수신 모드를 활성화한다.

```bash
# 어드미턴스 모드로 수신
python3 -m teleop_dev.robot.arm.admittance.main \
  --mode rtde \
  --input unified \
  --robot-ip 192.168.0.2

# 임피던스 모드로 수신
python3 -m teleop_dev.robot.arm.impedance.main \
  --mode rtde \
  --input unified \
  --robot-ip 192.168.0.2
```

`UnifiedNetworkInput`이 처리하는 내용:
- UDP 9871 대기 → `teleop_pose` 패킷 파싱
- `query_pose` 요청 시 현재 TCP 포즈 응답 (초기 핸드셰이크)
- 쿼터니언 변환: `wxyz` (프로토콜) → `xyzw` (Pinocchio)
- `ButtonState` → `TeleopCommand` 매핑

---

## 모니터링 (디버깅)

조종 PC에서 전송되는 패킷을 실시간으로 확인:

```bash
python3 -m teleop_dev.operator.arm.monitor --port 9871
```

포즈, 버튼 상태, 전송 빈도 등을 실시간 대시보드로 표시.

---

## 트러블슈팅

### sender 시작 시 "No response to query_pose"

- 로봇 PC에서 수신 프로그램이 실행 중인지 확인 (`--input unified`)
- 방화벽에서 UDP 9871 허용 확인
- `--target-ip`가 정확한지 확인
- 타임아웃 시 기본 홈 포즈 `[0.0, -0.4, 0.4]`에서 시작

### Vive 트래커가 감지되지 않음

- SteamVR 실행 여부 확인 (트래커 아이콘 녹색)
- `--list-trackers`로 감지 확인
- USB 동글이 연결되어 있고, 트래커가 페어링되어 있는지 확인

### 키보드 입력이 먹히지 않음

- 터미널이 포커스 상태인지 확인 (termios는 현재 터미널만 읽음)
- macOS/Windows에서는 termios 미지원 → Linux 사용

### 조이스틱이 감지되지 않음

- `pygame.joystick.get_count()`가 0인지 확인
- USB 연결 재확인, `lsusb`로 디바이스 표시 확인
- 일부 게임패드는 `xboxdrv` 또는 `xpad` 드라이버 필요

### 로봇이 움직이지 않음

- 로봇 PC에서 `--mode rtde`로 실제 로봇 연결 확인
- `--mode sim`으로 먼저 시뮬레이션 테스트
- E-Stop 상태 확인 (Space 키로 해제 후 R 키로 리셋)

---

## 빠른 시작 요약

| 단계 | 조종 PC | 로봇 PC |
|------|---------|---------|
| 1 | - | `python3 -m teleop_dev.robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2` |
| 2 | `python3 -m teleop_dev.operator.arm.keyboard_sender --target-ip <로봇IP>` | (자동 수신) |
| 3 | 키보드 W/A/S/D로 이동 확인 | 로봇 움직임 확인 |

Vive 사용 시 2번을 `vive_sender`로 교체. 조이스틱 사용 시 `joystick_sender`로 교체.
