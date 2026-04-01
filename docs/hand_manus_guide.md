# Manus 글러브 → Tesollo DG5F 손 원격조종 가이드

Manus Quantum Metaglove로 손 동작을 캡처하여 Tesollo DG5F 로봇 핸드를 원격 제어하는 방법.

---

## 전체 흐름

### Mode A: Raw 전송 (retarget은 로봇 PC에서)

```
조종 PC (Operator)                          로봇 PC (AGX Orin)
┌──────────────────────┐                   ┌──────────────────────────────┐
│ Manus Glove          │                   │ dg5f_driver (pid_all_ctrl)   │
│   ↓                  │                   │   ↓                          │
│ ManusReader (SDK)    │                   │ ManusReceiver (UDP)          │
│   ↓                  │   UDP 9872        │   ↓                          │
│ manus_sender.py  ────┼──────────────────→│ ManusToD5FRetarget + EMA     │
│                      │                   │   ↓                          │
│ [pynput 키보드]      │                   │ DG5FROS2Client (ROS2 topic)  │
│                      │                   │   ↓ MultiDOFCommand          │
│                      │                   │ Tesollo DG5F Hand            │
└──────────────────────┘                   └──────────────────────────────┘
```

### Mode B: Vector retarget (retarget은 조종 PC에서)

```
조종 PC (Operator)                          로봇 PC (AGX Orin)
┌────────────────────────────┐             ┌──────────────────────────────┐
│ Manus Glove                │             │ dg5f_driver (pid_all_ctrl)   │
│   ↓                        │             │   ↓                          │
│ ManusReader (SDK)          │             │ ManusReceiver (UDP)          │
│   ↓                        │  UDP 9872   │   ↓ (retarget 스킵)         │
│ VectorRetarget (최적화)    │             │ DG5FROS2Client               │
│   ↓ DG5F angles            │             │   ↓ MultiDOFCommand          │
│ manus_sender.py ───────────┼────────────→│ Tesollo DG5F Hand            │
│  (retargeted=true)         │             │                              │
└────────────────────────────┘             └──────────────────────────────┘
```

> **자동 감지**: packet에 `"retargeted": true`가 있으면 receiver가 retarget을 스킵.

---

## 1. 사전 요건

### 조종 PC (Operator)

- Ubuntu 22.04 (또는 Windows WSL2)
- Manus Quantum Metaglove + USB 동글
- Python 패키지:
  ```bash
  pip install numpy pyyaml pynput
  ```
- 시스템 패키지 (SDK 빌드용):
  ```bash
  sudo apt install build-essential libusb-1.0-0-dev zlib1g-dev libudev-dev libncurses5-dev pkg-config
  ```

### 로봇 PC (Robot)

- NVIDIA Jetson AGX Orin (arm64) 또는 x86_64 PC
- ROS 2 Humble
- Tesollo DG5F 핸드 (이더넷 연결, 기본 IP: `169.254.186.72`)
- dg5f_ros2 드라이버 스택:
  ```bash
  # 소스 클론
  cd ~/ws/src
  git clone https://github.com/tesollodelto/dg5f_ros2.git
  git clone https://github.com/kkb131/dg_hardware.git  # arm64 stub 포함
  git clone https://github.com/tesollodelto/dg_tcp_comm.git

  # 빌드
  cd ~/ws
  rosdep install --from-paths src/dg5f_ros2/dg5f_driver --ignore-src -r -y
  colcon build --symlink-install --packages-select delto_tcp_comm delto_hardware dg5f_driver dg5f_description
  ```

> **ARM64 참고**: `dg_hardware`의 `libdelto_gripper_helper.so`는 x86_64 전용 바이너리.
> arm64에서는 리버스 엔지니어링된 stub이 자동으로 빌드됨. 상세: `dg_hardware/README.md` 참조.

---

## 2. Manus SDK 설치 및 빌드

### 2.1 SDK 다운로드

Manus SDK Linux 버전을 [공식 문서](https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/)에서 다운로드.
`sender/hand/sdk/SDKClient_Linux/` 디렉토리에 배치.

### 2.2 SDK 빌드

```bash
cd teleop_dev/sender/hand/sdk
./build.sh
```

빌드 성공 시 `SDKClient_Linux/SDKClient_Linux.out` 생성.

### 2.3 SDK 모드 선택

| 모드 | 설명 | 필요 소프트웨어 |
|------|------|----------------|
| **Integrated** | 글러브에서 직접 데이터 수신 (별도 소프트웨어 불필요) | libManusSDK_Integrated.so |
| **Remote** | Manus Core 소프트웨어 경유 | Manus Core (Windows) + libManusSDK.so |

기본: **Integrated 모드** (독립 실행, 별도 PC 불필요).

### 2.4 USB 동글 확인

```bash
lsusb | grep -i manus
# 또는
ls /dev/hidraw*
```

동글이 감지되지 않으면 USB 연결 확인.

---

## 3. 캘리브레이션 (최초 사용 시)

사용자마다 손 크기가 다르므로, 관절 각도 범위를 캘리브레이션한다.

```bash
python3 -m sender.hand.calibrate \
  --hand right \
  --output calibration_right.json \
  --samples 100
```

### 캘리브레이션 순서

1. 프로그램 실행 → "손을 완전히 펴세요" 안내
2. Enter → 100샘플 수집 (최소 각도 기록)
3. "주먹을 쥐세요" 안내
4. Enter → 100샘플 수집 (최대 각도 기록)
5. `calibration_right.json` 저장

### 캘리브레이션 파일 형식

```json
{
  "hand": "right",
  "timestamp": "2024-03-24 14:30:00",
  "joints": {
    "Thumb_MCP_Spread": {"index": 0, "min": -0.5, "max": 0.7, "range": 1.2},
    "Thumb_MCP_Stretch": {"index": 1, "min": 0.0, "max": 1.4, "range": 1.4},
    ...
  }
}
```

---

## 4. Sender 실행 (조종 PC)

### 기본 실행 (Mode A: raw 전송)

```bash
python3 -m sender.hand.manus_sender \
  --target-ip 192.168.0.10 \
  --hand right
```

### Vector retarget 실행 (Mode B: 조종 PC에서 retarget)

```bash
python3 -m sender.hand.manus_sender \
  --target-ip 192.168.0.10 \
  --hand right \
  --retarget vector \
  --calibrate
```

`--calibrate`: 시작 시 3초간 손을 편 상태로 baseline 캘리브레이션.

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--port` | 9872 | UDP 포트 |
| `--hz` | 60 | 전송 주파수 (Hz) |
| `--hand` | right | `left`, `right`, 또는 `both` |
| `--config` | default.yaml | YAML 설정 파일 경로 |
| `--sdk-path` | (자동) | SDKClient_Linux.out 경로 |
| `--retarget` | none | `none` (raw) 또는 `vector` (벡터 최적화) |
| `--calibrate` | false | 시작 시 open-hand baseline 캘리브레이션 |
| `--urdf` | (자동) | DG5F URDF 경로 (vector 모드용) |

### 키보드 제어 (sender 실행 중)

| 키 | 동작 |
|----|------|
| Space | E-Stop (긴급 정지) |
| R | 리셋 |
| Q / Esc | 종료 |
| + / = | 속도 증가 |
| - | 속도 감소 |

### 설정 파일 (`sender/hand/config/default.yaml`)

```yaml
network:
  target_ip: "192.168.0.10"
  port: 9872
  hz: 60

hand:
  side: "right"
  dongle_id: 0

joint_mapping:
  num_joints: 20
  calibration_file: null    # calibration_right.json 경로

sdk:
  bin_path: "sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out"
```

---

## 5. 시각화 (디버깅용)

하드웨어 연결 전에 글러브 데이터를 시각적으로 확인:

```bash
# SDK 직접 연결 (글러브 필요)
python3 -m sender.hand.hand_visualizer --sdk

# UDP 수신 모드 (sender가 이미 실행 중일 때)
python3 -m sender.hand.hand_visualizer --udp --port 9872

# Mock 데이터 (하드웨어 없이 테스트)
python3 -m sender.hand.hand_visualizer
```

pygame 기반 2D 스켈레톤 + 관절 각도 바 차트를 표시.

---

## 6. DG5F 드라이버 실행 (로봇 PC)

### 6.1 드라이버 시작 (Terminal 1)

PID_ALL 컨트롤러 모드로 실행 (권장):

```bash
source ~/ws/install/setup.bash
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72
```

Left hand의 경우:
```bash
ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py delto_ip:=169.254.186.73
```

### 6.2 드라이버 확인

```bash
# 컨트롤러 상태 (rj_dg_pospid가 active인지 확인)
ros2 control list_controllers -c /dg5f_right/controller_manager

# joint_states 확인
ros2 topic echo /dg5f_right/joint_states --once

# 토픽 목록
ros2 topic list | grep dg5f
```

### 6.3 ROS2 토픽

| 토픽 | 메시지 타입 | 용도 |
|------|------------|------|
| `/dg5f_right/rj_dg_pospid/reference` | `control_msgs/MultiDOFCommand` | 관절 위치 명령 (PID) |
| `/dg5f_right/joint_states` | `sensor_msgs/JointState` | 관절 피드백 (위치/속도/토크) |

### 6.4 PID 튜닝

설정 파일: `dg5f_driver/config/dg5f_right_pid_all_controller.yaml`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `p` | 1.5 | 비례 게인 (높이면 빠르지만 진동 위험) |
| `i` | 0.0 | 적분 게인 (정상상태 오차 제거) |
| `d` | 0.0 | 미분 게인 (진동 감쇠) |
| `i_clamp_max/min` | 0.0 | 적분 windup 제한 |

튜닝 테스트:
```bash
python3 -m robot.hand.tests.test_tuning --hand right
python3 -m robot.hand.tests.test_tuning --hand right --test oscillation
```

> yaml 수정 후 드라이버 재시작 필요.

---

## 7. Receiver 실행 (로봇 PC, Terminal 2)

### 기본 실행 (ROS2 모드)

```bash
source ~/ws/install/setup.bash
python3 -m robot.hand.receiver --hand right
```

### Dry-run (하드웨어 없이)

```bash
python3 -m robot.hand.receiver --dry-run
```

UDP 수신 + retarget 결과만 출력, ROS2 발행 안 함.

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--hand` | right | `left` 또는 `right` |
| `--port` | 9872 | UDP 수신 포트 |
| `--hz` | 60 | 제어 루프 주파수 |
| `--motion-time` | 50 | 관절 이동 시간 (ms) |
| `--config` | default.yaml | YAML 설정 파일 |
| `--dry-run` | false | 하드웨어 없이 출력만 |

---

## 8. 관절 매핑

### Manus 글러브 (20관절)

5개 손가락 x 4관절:

| 인덱스 | 관절 | 설명 |
|--------|------|------|
| 0 | MCP_Spread | 벌림 (좌우) |
| 1 | MCP_Stretch | 굽힘 (첫째 마디) |
| 2 | PIP_Stretch | 굽힘 (둘째 마디) |
| 3 | DIP_Stretch | 굽힘 (셋째 마디) |

순서: Thumb(0-3) → Index(4-7) → Middle(8-11) → Ring(12-15) → Pinky(16-19)

### Tesollo DG5F (20모터)

동일한 5x4 구조. `ManusToD5FRetarget`이 Manus 각도를 DG5F 모터 각도로 변환.

```python
from robot.hand.retarget import ManusToD5FRetarget

retarget = ManusToD5FRetarget(hand_side="right")
dg5f_angles = retarget.retarget(manus_angles)  # ndarray[20] → ndarray[20]
```

### DG5F 관절 이름

| 손가락 | Joint 1 (Spread) | Joint 2 (MCP) | Joint 3 (PIP) | Joint 4 (DIP) |
|--------|-------------------|---------------|----------------|----------------|
| Thumb | rj_dg_1_1 | rj_dg_1_2 | rj_dg_1_3 | rj_dg_1_4 |
| Index | rj_dg_2_1 | rj_dg_2_2 | rj_dg_2_3 | rj_dg_2_4 |
| Middle | rj_dg_3_1 | rj_dg_3_2 | rj_dg_3_3 | rj_dg_3_4 |
| Ring | rj_dg_4_1 | rj_dg_4_2 | rj_dg_4_3 | rj_dg_4_4 |
| Pinky | rj_dg_5_1 | rj_dg_5_2 | rj_dg_5_3 | rj_dg_5_4 |

Left hand: `rj_` → `lj_`

---

## 9. UDP 패킷 포맷

`manus_sender` → `receiver` JSON over UDP 9872:

```json
{
  "type": "manus",
  "hand": "right",
  "joint_angles": [20 floats],
  "finger_spread": [5 floats],
  "wrist_pos": [x, y, z],
  "wrist_quat": [w, x, y, z],
  "tracking": true,
  "retargeted": false,
  "buttons": {
    "estop": false,
    "reset": false,
    "quit": false,
    "speed_up": false,
    "speed_down": false
  },
  "timestamp": 1711180800.123
}
```

- `joint_angles`: 라디안, 순서는 Thumb→Index→Middle→Ring→Pinky (각 4관절)
- `wrist_quat`: wxyz 순서
- `tracking`: false이면 글러브 추적 불가 상태
- `retargeted`: true이면 sender에서 이미 DG5F 각도로 변환됨 (receiver가 retarget 스킵)

### 특수 패킷

| type | 용도 |
|------|------|
| `"manus"` | 핸드 데이터 (위 형식) |
| `"reload_config"` | receiver에 config 재로드 트리거 (calibrate_retarget에서 전송) |

---

## 10. DG5F 직접 제어 (Python API)

### ROS2 PID API (권장)

```python
import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand

rclpy.init()
node = Node("dg5f_test")

RIGHT_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

pub = node.create_publisher(MultiDOFCommand, "/dg5f_right/rj_dg_pospid/reference", 10)

msg = MultiDOFCommand()
msg.dof_names = RIGHT_JOINTS
msg.values = [0.0] * 20           # 손 펴기 (0도)
pub.publish(msg)

node.destroy_node()
rclpy.shutdown()
```

### 튜닝 테스트

```bash
# dg5f_driver (pid_all) 실행 중인 상태에서:
python3 -m robot.hand.tests.test_tuning --hand right
python3 -m robot.hand.tests.test_tuning --hand right --test zero
```

---

## 11. 핸드 하드웨어 사양

| 항목 | 값 |
|------|-----|
| 모델 | Tesollo DG 5F M |
| DOF | 20 (5 fingers x 4 joints) |
| 통신 | Modbus TCP (dg5f_driver가 내부 처리) |
| IP | 169.254.186.72 (right), 169.254.186.73 (left) |
| Port | 502 |
| Slave ID | 1 |
| 토크 상수 | 3.27 mNm/A |
| 기어비 | 386.4:1 |

---

## 12. 빠른 시작 요약

### Mode A: Raw 전송 (로봇 PC에서 retarget)

| 단계 | 조종 PC | 로봇 PC |
|------|---------|---------|
| 1. SDK 빌드 | `cd sender/hand/sdk && ./build.sh` | - |
| 2. 드라이버 시작 | - | `ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72` |
| 3. Receiver 시작 | - | `python3 -m robot.hand.receiver --hand right` |
| 4. Sender 시작 | `python3 -m sender.hand.manus_sender --target-ip <로봇IP>` | (자동 수신) |
| 5. 동작 확인 | 손을 움직여서 확인 | DG5F 핸드 동작 확인 |

### Mode B: Vector retarget (조종 PC에서 retarget)

| 단계 | 조종 PC | 로봇 PC |
|------|---------|---------|
| 1. SDK 빌드 | `cd sender/hand/sdk && ./build.sh` | - |
| 2. 드라이버 시작 | - | `ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72` |
| 3. Receiver 시작 | - | `python3 -m robot.hand.receiver --hand right` |
| 4. Sender 시작 | `python3 -m sender.hand.manus_sender --target-ip <로봇IP> --retarget vector --calibrate` | (자동 수신, retarget 스킵) |
| 5. 동작 확인 | 시작 3초: 손 펴기 (캘리), 이후 자유 동작 | DG5F 핸드 동작 확인 |

---

## 13. 트러블슈팅

### SDK 바이너리를 찾을 수 없음

```
FileNotFoundError: SDKClient_Linux.out not found
```
→ `--sdk-path` 옵션으로 정확한 경로 지정, 또는 `./build.sh` 재실행

### 첫 데이터 수신 타임아웃 (10초)

```
Timeout waiting for first hand data
```
→ USB 동글 연결 확인 (`lsusb`), 글러브 전원 ON 확인, SDK 바이너리 로그 확인

### 글러브 추적 끊김

sender 로그에 `[LOST]` 표시 → 글러브-동글 거리 확인, USB 간섭 제거

### Robot PC에서 UDP 데이터 미수신

→ 방화벽 UDP 9872 허용 확인:
```bash
sudo ufw allow 9872/udp
```
→ `--target-ip`가 로봇 PC IP와 일치하는지 확인

### DG5F 드라이버 시작 실패

```
[ERROR] Failed to connect to DG5F
```
→ 이더넷 케이블 연결 확인
→ 핸드 IP 확인: `ping 169.254.186.72`
→ 핸드 전원 확인 (부팅 후 10-15초 대기)

### DG5F 모터가 움직이지 않음

→ 컨트롤러 상태 확인:
```bash
ros2 control list_controllers -c /dg5f_right/controller_manager
```
→ `rj_dg_pospid`가 `active` 상태인지 확인
→ joint_states에서 position 변화 확인:
```bash
ros2 topic echo /dg5f_right/joint_states --once
```
→ `--dry-run`으로 retarget 출력값 확인

### ARM64 빌드 에러 (libdelto_gripper_helper.so)

```
error adding symbols: file in wrong format
```
→ `kkb131/dg_hardware` 레포 사용 (arm64 stub 포함)
→ 상세: `dg_hardware/README.md`의 ARM64 Build Notes 참조
