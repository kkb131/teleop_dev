# Vive Tracker 3.0 Teleop Setup Guide

UR10e 원격조종을 위한 Vive Tracker 3.0 설정 가이드.

> **통합 프로토콜 가이드**: 키보드/조이스틱/Vive를 통합한 원격조작 전체 가이드는
> [`docs/unified_teleop_guide.md`](../docs/unified_teleop_guide.md) 를 참조하세요.

## 구성

```
[조종 PC (RTX 5090)]          UDP (port 9871)          [로봇 PC (AGX Orin)]
 SteamVR + Vive Tracker  ─────────────────────────>  standalone teleop
 vive_sender.py             통합 프로토콜               --input unified
 + 키보드 (E-Stop 등)    (absolute pose, base_link)
```

## 하드웨어 준비물

- HTC Vive Tracker 3.0 x1
- SteamVR Base Station 2.0 x2
- Vive Tracker USB Dongle x1 (트래커 페어링용)
- 조종 PC: Ubuntu 22.04 + NVIDIA GPU (SteamVR 구동용)

## 1. 조종 PC 설치

### 1.1 Steam + SteamVR 설치

```bash
# Steam 설치 (Ubuntu)
sudo dpkg --add-architecture i386
sudo apt update
sudo apt install steam

# Steam 실행 후 SteamVR 설치
# Steam → Library → SteamVR → Install
```

### 1.2 SteamVR 헤드셋 없이 트래커만 사용 (Null Driver)

헤드셋 없이 트래커만 사용하려면 SteamVR null driver 설정이 필요:

```bash
# SteamVR config 경로 (Linux)
mkdir -p ~/.config/openvr

# steamvr.vrsettings 편집 (또는 생성)
# 경로: ~/.steam/steam/config/steamvr.vrsettings
```

`steamvr.vrsettings`에 다음 추가:
```json
{
   "steamvr": {
      "requireHmd": false,
      "forcedDriver": "null",
      "activateMultipleDrivers": true
   }
}
```

또는 SteamVR 설정에서:
- SteamVR → Settings → Developer → Enable "Do not require HMD"

### 1.3 Python 패키지 설치

```bash
pip install openvr numpy pynput
```

### 1.4 Base Station 배치

- 두 Base Station을 대각선 방향으로 배치 (2~5m 간격)
- 각 스테이션을 높은 위치 (2m+)에 설치, 아래를 향하도록
- 전원 연결 (AC 어댑터)
- LED가 녹색이면 정상 작동

### 1.5 Vive Tracker 페어링

1. SteamVR 실행
2. Vive Tracker USB Dongle을 PC에 연결
3. 트래커 전원 버튼 길게 누르기 (LED 파란색 점멸)
4. SteamVR → Devices → Pair Controller
5. 페어링 완료 시 SteamVR에서 트래커 아이콘 표시 (녹색 = 트래킹 중)

## 2. 캘리브레이션

SteamVR 좌표계와 로봇 좌표계를 정렬합니다.

### 2.1 캘리브레이션 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m vive.calibrate --output vive/calibration.json
```

3개 포인트를 순서대로 측정:
1. **로봇 base_link 원점** — 트래커를 로봇 베이스 중심에 배치
2. **로봇 +X 방향** — 원점에서 오른쪽으로 30cm+ 이동
3. **로봇 +Y 방향** — 원점에서 앞쪽으로 30cm+ 이동

### 2.2 캘리브레이션 검증

```bash
python3 -m vive.calibrate --verify vive/calibration.json
```

실시간으로 트래커 위치가 로봇 좌표계로 변환되어 출력됩니다.
로봇 base_link 원점에서 (0, 0, 0) 근처가 나오면 정상.

## 3. 실행

### 3.1 트래커 연결 확인

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# 트래커 목록 확인
python3 -m vive.vive_tracker --list

# 실시간 포즈 확인
python3 -m vive.vive_tracker --hz 50
```

### 3.2 조종 PC에서 sender 실행

```bash
cd ~/tamp_ws/src/tamp_dev
conda activate tamp_sender

# 기본 (첫 번째 트래커 사용, 50Hz, 통합 프로토콜)
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP>

# 캘리브레이션 적용
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP> --calibration vive/calibration.json

# 특정 트래커 지정
python3 -m vive.vive_sender --target-ip 192.168.0.10 --tracker-serial LHR-XXXXXXXX
```

**키보드 단축키 (조종 PC에서):**
| 키 | 기능 |
|---|---|
| Space | E-Stop |
| R | Reset (로봇 포즈 재동기화) |
| Q / Esc | Quit |
| + / = | 속도 증가 |
| - | 속도 감소 |

### 3.3 로봇 PC에서 teleop 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Admittance 모드 (통합 프로토콜)
python3 -m standalone.teleop_admittance.main \
    --mode rtde --input unified --robot-ip 192.168.0.2

# Impedance 모드
python3 -m standalone.teleop_impedance.main \
    --mode rtde --input unified --robot-ip 192.168.0.2

# Sim 모드 (mock hardware 테스트)
python3 -m standalone.teleop_admittance.main \
    --mode sim --input unified
```

> **레거시 모드**: 기존 `--input vive`도 동작하지만, 통합 프로토콜(`--input unified`)을 권장합니다.
> `--input unified`는 Vive뿐 아니라 키보드/조이스틱 sender도 동일하게 수신합니다.

## 4. 트러블슈팅

### SteamVR가 트래커를 인식하지 못함
- USB 동글이 연결되어 있는지 확인
- 트래커 충전 상태 확인 (LED 빨간색 = 배터리 부족)
- Base Station이 켜져 있고 녹색 LED인지 확인

### 트래킹이 자주 끊김
- Base Station 사이에 장애물이 없는지 확인
- 반사가 심한 표면(거울, 유리) 제거
- Base Station 간격이 너무 넓지 않은지 확인 (5m 이내)

### "No Vive Tracker found" 에러
- SteamVR이 실행 중인지 확인
- null driver 설정이 적용되었는지 확인
- `python3 -m vive.vive_tracker --list`로 디바이스 확인

### UDP 패킷이 수신되지 않음
- 방화벽 확인: `sudo ufw allow 9871/udp`
- 네트워크 연결 확인: `ping <ROBOT_PC_IP>`
- 수신 확인: `nc -lu 9871` (로봇 PC에서)

## 5. 단계별 테스트 가이드

클로드 코드 없는 환경에서 독립적으로 디버깅할 수 있도록 6단계 테스트를 제공합니다.

```
Step 1: OpenVR 연결       → test_step1_openvr      (SteamVR 필요)
Step 2: 트래커 포즈 읽기   → test_step2_pose        (SteamVR + 트래커 필요)
Step 3: UDP 통신           → test_step3_udp          (Vive 불필요!)
Step 4: 캘리브레이션 수학   → test_step4_calibration  (Vive 불필요!)
Step 5: Velocity 계산      → test_step5_velocity     (Vive 불필요!)
Step 6: 실전 E2E           → test_step6_e2e_sender   (전체 시스템)
```

> **Step 3~5는 Vive 없이도 실행 가능** → 로봇 PC에서 사전 검증에 활용하세요.

모든 테스트는 `src/tamp_dev/` 디렉토리에서 실행합니다:
```bash
cd /workspaces/tamp_ws/src/tamp_dev
```

### Step 1: OpenVR 연결 테스트

SteamVR 연결 + 디바이스 목록 확인.

```bash
python3 -m vive.tests.test_step1_openvr
```
```
# 예상 출력:
# [PASS] OpenVR initialized
# [PASS] Found 1 tracker(s):
#   index=3  serial=LHR-xxx  [TRACKING]
# Results: 3/3 passed
```

**실패 시 확인:**
- SteamVR이 실행 중인가?
- null driver 설정이 적용되었는가? (섹션 1.2 참고)
- `pip install openvr` 완료했는가?

### Step 2: 트래커 포즈 읽기

100프레임 읽어서 위치 범위, 쿼터니언 정규화, 주파수 검증.

```bash
python3 -m vive.tests.test_step2_pose --duration 3
```
```
# 예상 출력:
# [PASS] Read 150 valid poses (100.0%)
# [PASS] Position range reasonable
# [PASS] Quaternion norm: min=0.9999, max=1.0001
# [PASS] Frequency: 49.8 Hz
```

**실패 시 확인:**
- 트래커 전원이 켜져 있는가?
- Base Station이 트래커를 볼 수 있는가? (장애물, 반사면 확인)

### Step 3: UDP 통신 테스트

Mock 데이터로 UDP 송수신 검증. **Vive 불필요.**

```bash
# 자동 모드 (loopback 테스트, 터미널 1개)
python3 -m vive.tests.test_step3_udp --port 9873

# 수동 모드 (두 터미널)
# Terminal 1 - 수신
python3 -m vive.tests.test_step3_udp --mode recv --port 9873
# Terminal 2 - 송신
python3 -m vive.tests.test_step3_udp --mode send --target-ip 127.0.0.1 --port 9873
```
```
# 예상 출력:
# [PASS] Received 50 packets in 1.0s
# [PASS] Packet format valid
# [PASS] Timestamps monotonically increasing
```

**실패 시 확인:**
- 방화벽: `sudo ufw allow 9873/udp`
- 포트 충돌: 다른 프로세스가 같은 포트를 사용하고 있지 않은지

### Step 4: 캘리브레이션 수학 검증

알려진 입력으로 R, t 계산 → 결과 검증. **Vive 불필요.**

```bash
python3 -m vive.tests.test_step4_calibration
```
```
# 예상 출력:
# [PASS] Identity calibration
# [PASS] Translation-only calibration
# [PASS] 90-degree Z rotation calibration
# [PASS] Quaternion transform
# [PASS] Round-trip: calibrate then transform
# [PASS] Quaternion multiply correctness
# Results: 6/6 passed
```

### Step 5: Velocity 계산 검증

Mock UDP 패킷으로 알려진 궤적 전송 → ViveNetworkInput의 velocity 출력 검증. **Vive 불필요.**

```bash
python3 -m vive.tests.test_step5_velocity --port 9872
```
```
# 예상 출력:
# [PASS] Stationary tracker → velocity ≈ 0
# [PASS] Linear motion X (0.1 m/s) → vx ≈ 0.1
# [PASS] Tracking lost → velocity = 0
# [PASS] E-stop button → estop=True
# Results: 4/4 passed
```

**실패 시 확인:**
- `standalone.core.input_handler`가 import 가능한지 (`cd /workspaces/tamp_ws/src/tamp_dev`)
- `pip install pin-pink proxsuite numpy` 설치 완료 여부

### Step 6: 실전 E2E 테스트

실제 vive_sender.py로부터 패킷을 수신하여 품질 리포트 생성. **전체 시스템 필요.**

```bash
# 조종 PC
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP> --port 9871

# 로봇 PC (또는 같은 PC의 다른 터미널)
python3 -m vive.tests.test_step6_e2e_sender --port 9871 --duration 5
```
```
# 예상 출력:
# [PASS] Received 248 packets in 5.0s (49.6 Hz)
# [PASS] Tracking rate: 248/248 (100.0%)
# [INFO] Position std dev: X: 0.30mm, Y: 0.25mm, Z: 0.28mm
# [PASS] Latency: mean=2.1ms, max=5.3ms
# [PASS] Interval: mean=20.0ms, σ=1.2ms
```

### 디버깅 순서 권장

문제 발생 시 아래 순서로 테스트하여 어느 단계에서 실패하는지 확인:

1. **Step 3** (UDP) → 네트워크 문제 격리
2. **Step 4** (캘리브레이션 수학) → 좌표 변환 로직 확인
3. **Step 5** (Velocity) → ViveNetworkInput 로직 확인
4. **Step 1** (OpenVR) → SteamVR 연결 확인
5. **Step 2** (Pose) → 트래커 품질 확인
6. **Step 6** (E2E) → 전체 시스템 통합 확인

Step 3~5를 먼저 통과시킨 후 SteamVR 관련 테스트(1, 2, 6)로 넘어가면 효율적입니다.

## 6. 좌표계 참고

```
SteamVR (Y-up, 오른손 좌표계)     UR10e (Z-up)
    Y (up)                           Z (up)
    |                                |
    |                                |
    +--- X (right)                   +--- X (right)
   /                                /
  Z (toward user)                  Y (forward)
```

캘리브레이션 없이 사용 시 기본 매핑:
- SteamVR X → Robot X
- SteamVR Y → Robot Z
- SteamVR Z → Robot -Y
