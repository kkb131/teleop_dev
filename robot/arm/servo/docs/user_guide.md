# Servo 실시간 제어 사용자 가이드

## 개요

UR10e 로봇의 실시간(real-time) 텔레오퍼레이션을 위한 가이드입니다.
모든 스크립트는 **두 가지 백엔드 모드**를 지원합니다:

| 모드 | 설명 | ROS2 필요 | 용도 |
|------|------|-----------|------|
| `--mode rtde` | ur_rtde 라이브러리로 직접 통신 | 아니오 (joystick만 /joy용) | 실제 로봇 |
| `--mode sim` | SimBackend (ROS2 토픽: `/joint_states`, `/joint_command`) | 예 | Isaac Sim / mock hardware |

### 제어 스크립트

| 스크립트 | 제어 공간 | 입력 | F/T 지원 |
|----------|-----------|------|----------|
| `keyboard_forward.py` | Joint 공간 | 키보드 | - |
| `keyboard_cartesian.py` | Cartesian (Pinocchio DLS IK) | 키보드 | - |
| `joystick_cartesian.py` | Cartesian (Pinocchio DLS IK) | Xbox 조이스틱 | - |
| `keyboard_servo_admittance.py` | Cartesian + F/T Admittance | 키보드 | RTDE만 |

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [파일 구조](#2-파일-구조)
3. [Joint-Space 키보드 제어](#3-joint-space-키보드-제어)
4. [Cartesian 키보드 제어](#4-cartesian-키보드-제어)
5. [Cartesian Xbox 조이스틱 제어](#5-cartesian-xbox-조이스틱-제어)
6. [Admittance 키보드 제어](#6-admittance-키보드-제어)
7. [백엔드 아키텍처](#7-백엔드-아키텍처)
8. [설정 파라미터](#8-설정-파라미터)
9. [트러블슈팅](#9-트러블슈팅)

---

## 1. 사전 요구사항

### 1.1 공통 (Pinocchio)

```bash
sudo apt install -y ros-humble-pinocchio
```

### 1.2 RTDE 모드 (실제 로봇)

```bash
pip install ur-rtde
```

- ROS2, ControllerSwitcher 불필요
- 로봇 IP로 직접 연결 (기본: `192.168.0.2`)

### 1.3 Sim 모드 (Isaac Sim / mock hardware)

Isaac Sim 또는 mock hardware가 `/joint_states`, `/joint_command` 토픽을 발행/구독해야 합니다.

**mock hardware 예시**:
```bash
# 별도 터미널에서 UR Driver 실행
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
```

### 1.4 Xbox 조이스틱 (선택)

```bash
sudo apt install -y ros-humble-joy

# 별도 터미널에서 joy_node 실행
ros2 run joy joy_node
```

---

## 2. 파일 구조

```
robot/arm/servo/
├── keyboard_forward.py            # Joint-space 키보드 제어
├── keyboard_cartesian.py          # Cartesian 키보드 제어 (Pinocchio DLS)
├── joystick_cartesian.py          # Cartesian Xbox 제어 (Pinocchio DLS)
├── keyboard_servo_admittance.py   # Cartesian + F/T Admittance
├── controller_utils.py            # ControllerSwitcher (레거시, sim 전용 선택사항)
├── pinocchio_utils.py             # PinocchioIK (FK/Jacobian/DLS)
└── docs/
    └── user_guide.md              # 이 문서

robot/
├── config.py                      # 공유 설정 (JOINT_NAMES, DEFAULT_MODE, IP 등)
├── core/robot_backend.py          # RobotBackend ABC + create_backend() 팩토리
├── core/ur_robot.py               # RTDEBackend (실제 로봇)
└── core/sim_robot.py              # SimBackend (Isaac Sim / ROS2 토픽)
```

---

## 3. Joint-Space 키보드 제어

개별 joint를 직접 제어합니다.

### 3.1 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드 (기본)
python3 -m robot.arm.servo.keyboard_forward --mode sim

# RTDE 모드 (실제 로봇)
python3 -m robot.arm.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2
```

### 3.2 키 매핑

| 키 | 동작 |
|----|------|
| `1` ~ `6` | Joint 1~6 선택 |
| `w` / `↑` | 선택된 joint 위치 증가 (+step) |
| `s` / `↓` | 선택된 joint 위치 감소 (-step) |
| `+` / `=` | step 크기 증가 |
| `-` | step 크기 감소 |
| `h` | Home 위치로 이동 |
| `p` | 현재 joint 상태 출력 |
| `Space` | 현재 위치 유지 (정지) |
| `q` / `Esc` | 종료 |

**step 크기**: 0.001, 0.005, **0.01** (기본), 0.02, 0.05, 0.1 rad

### 3.3 동작 원리

```
키보드 입력 → target_positions 업데이트 → backend.send_joint_command()
                                              ↓
                                 RTDE: servoJ() → 로봇
                                 Sim:  /joint_command 토픽 → Isaac Sim
```

---

## 4. Cartesian 키보드 제어

Pinocchio의 Jacobian + Damped Least Squares (DLS)를 사용하여 end-effector를 Cartesian 공간에서 제어합니다.

### 4.1 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드
python3 -m robot.arm.servo.keyboard_cartesian --mode sim

# RTDE 모드
python3 -m robot.arm.servo.keyboard_cartesian --mode rtde --robot-ip 192.168.0.2
```

### 4.2 키 매핑

**이동 (Translation)**:

| 키 | 방향 | 설명 |
|----|------|------|
| `w` / `s` | X | 전진 / 후진 |
| `a` / `d` | Y | 좌 / 우 |
| `q` / `e` | Z | 상승 / 하강 |

**회전 (Rotation)**:

| 키 | 축 | 설명 |
|----|-----|------|
| `u` / `o` | RX | Roll +/- |
| `i` / `k` | RY | Pitch +/- |
| `j` / `l` | RZ | Yaw +/- |

**제어**:

| 키 | 동작 |
|----|------|
| `+` / `=` | 속도 증가 |
| `-` | 속도 감소 |
| `f` | 프레임 전환 (base_link ↔ tool0) |
| `p` | 현재 EE pose 출력 (FK) |
| `Space` | 정지 |
| `x` / `Esc` | 종료 |

**속도 스케일**: 0.1, 0.2, **0.3** (기본), 0.5, 0.8, 1.0

### 4.3 동작 원리

```
키보드 입력 → twist (6D) 생성
                ↓
Pinocchio: Jacobian 계산 → DLS inverse → joint delta
                ↓
q_new = clamp(q + dq) → backend.send_joint_command() → 로봇/시뮬레이터
```

- DLS: `dq = J^T @ inv(J @ J^T + λ²I) @ twist * dt`
- λ = 0.05 (damping factor) — singularity 근처에서 joint velocity 자동 제한
- 50Hz 루프

### 4.4 프레임 설명

| 프레임 | 설명 |
|--------|------|
| `base_link` (기본) | 로봇 베이스 기준. X=전방, Y=좌, Z=상 (직관적) |
| `tool0` | 엔드이펙터 기준. 로봇 자세에 따라 축이 변함 |

`f` 키로 전환 가능. 일반적으로 `base_link`가 더 직관적입니다.

---

## 5. Cartesian Xbox 조이스틱 제어

Xbox 컨트롤러로 Cartesian 제어합니다 (Pinocchio DLS).
**두 모드 모두 ROS2 필요** (`/joy` 토픽 구독용).

### 5.1 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# joy_node 먼저 실행 (별도 터미널)
ros2 run joy joy_node

# Sim 모드
python3 -m robot.arm.servo.joystick_cartesian --mode sim

# RTDE 모드 (로봇 통신은 RTDE, 조이스틱만 ROS2)
python3 -m robot.arm.servo.joystick_cartesian --mode rtde --robot-ip 192.168.0.2
```

### 5.2 Xbox 매핑

**이동 (Stick / Trigger)**:

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 X | Y 이동 (좌/우) |
| 왼쪽 스틱 Y | X 이동 (전진/후진) |
| 오른쪽 스틱 X | Yaw (RZ) 회전 |
| 오른쪽 스틱 Y | Pitch (RY) 회전 |
| LT (왼쪽 트리거) | Z 하강 |
| RT (오른쪽 트리거) | Z 상승 |

**버튼**:

| 버튼 | 동작 |
|------|------|
| LB (왼쪽 범퍼) | Roll (RX) - |
| RB (오른쪽 범퍼) | Roll (RX) + |
| A | 속도 감소 |
| B | 속도 증가 |
| X | 프레임 전환 (base_link ↔ tool0) |
| Y | EE pose 출력 (FK) |
| Start | 종료 |

- Deadzone: 0.1 (미세 떨림 무시)
- 아날로그 스틱은 비례 제어 (기울기에 따라 속도 변화)

---

## 6. Admittance 키보드 제어

Cartesian 키보드 텔레오퍼레이션에 F/T 센서 기반 어드미턴스 제어를 결합합니다.

### 6.1 모드별 동작

| 기능 | RTDE 모드 | Sim 모드 |
|------|-----------|----------|
| 키보드 Cartesian 제어 | O | O |
| F/T 센서 데이터 | `RTDEFTSource` (ur_rtde 직접) | `NullFTSource` (항상 0) |
| Admittance 제어 | **활성** | **비활성** (경고 출력) |

### 6.2 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드 (admittance 비활성, keyboard Cartesian만 동작)
python3 -m robot.arm.servo.keyboard_servo_admittance --mode sim

# RTDE 모드 (admittance 활성, F/T 센서 실시간 읽기)
python3 -m robot.arm.servo.keyboard_servo_admittance --mode rtde --robot-ip 192.168.0.2
```

### 6.3 키 매핑

Cartesian 키보드 매핑(섹션 4.2)과 동일 + 아래 Admittance 전용 키:

| 키 | 동작 |
|----|------|
| `z` | F/T 센서 제로 (바이어스 보정) |
| `t` | Admittance ON/OFF 토글 |
| `1` | Stiff 프리셋 |
| `2` | Medium 프리셋 (기본) |
| `3` | Soft 프리셋 |

### 6.4 Admittance 프리셋 (MDK)

| 프리셋 | M (질량) | D (감쇠) | K (강성) | 특성 |
|--------|----------|----------|----------|------|
| STIFF | 10 / 1 | 200 / 20 | 500 / 50 | 단단함, 외력에 적게 반응 |
| MEDIUM | 5 / 0.5 | 100 / 10 | 200 / 20 | 중간 (기본) |
| SOFT | 2 / 0.2 | 40 / 4 | 50 / 5 | 부드러움, 외력에 크게 반응 |

값은 [force / torque] 순서. 안정성 조건: D >= 2*sqrt(M*K)

### 6.5 동작 원리

```
키보드 → twist → Pinocchio DLS → dq_teleop ──┐
                                                ├→ target_q → backend → 로봇
F/T 센서 → deadzone → Admittance MDK → J_pinv ─┘
```

**안전 제한**:
- Dead-zone: 3.0 N (force), 0.3 Nm (torque)
- Saturation: 100 N / 10 Nm 초과 시 admittance 비활성
- Max displacement: 5 cm (translation), ~8.6° (rotation)

### 6.6 F/T 소스 추상화

```python
# RTDE 모드: ur_rtde에서 직접 읽기
class RTDEFTSource:
    def get_wrench(self) -> np.ndarray  # getActualTCPForce()
    def zero_sensor(self)               # 소프트웨어 바이어스 보정

# Sim 모드: F/T 센서 없음
class NullFTSource:
    def get_wrench(self) -> np.ndarray  # 항상 np.zeros(6)
    def zero_sensor(self)               # no-op
```

---

## 7. 백엔드 아키텍처

모든 servo 스크립트는 `RobotBackend` ABC를 사용합니다:

```
robot/core/robot_backend.py
├── RobotBackend (ABC)
│   ├── connect() / disconnect()
│   ├── get_joint_positions() → List[float]
│   ├── get_joint_velocities() → List[float]
│   └── send_joint_command(positions)
│
├── RTDEBackend (ur_robot.py)
│   ├── ur_rtde servoJ() (125Hz)
│   ├── get_tcp_pose() — 추가 메서드
│   └── get_tcp_force() — F/T 데이터 (admittance용)
│
└── SimBackend (sim_robot.py)
    ├── Subscribe: /joint_states
    ├── Publish: /joint_command
    └── 내부 ROS2 스피너 (백그라운드 스레드)
```

### 7.1 팩토리 사용법

```python
from robot.core.robot_backend import create_backend

# RTDE 모드
backend = create_backend("rtde", robot_ip="192.168.0.2")

# Sim 모드
backend = create_backend("sim")

with backend:
    q = backend.get_joint_positions()
    backend.send_joint_command(q)
```

### 7.2 모드별 ROS2 의존성

| 스크립트 | RTDE 모드 ROS2 | Sim 모드 ROS2 |
|----------|----------------|---------------|
| keyboard_forward | 불필요 | SimBackend 내부 |
| keyboard_cartesian | 불필요 | SimBackend 내부 |
| joystick_cartesian | `/joy` 구독용 최소 노드 | SimBackend + `/joy` 구독 |
| keyboard_servo_admittance | 불필요 | SimBackend 내부 |

---

## 8. 설정 파라미터

### 8.1 config.py (공유 설정)

| 상수 | 값 | 설명 |
|------|-----|------|
| `DEFAULT_MODE` | `"sim"` | 기본 백엔드 모드 |
| `DEFAULT_ROBOT_IP` | `"192.168.0.2"` | RTDE 기본 IP |
| `SERVO_RATE_HZ` | `50` | 서보 루프 주기 (Hz) |
| `RTDE_FREQUENCY` | `125` | RTDE servoJ 주기 (Hz) |
| `SERVOJ_DT` | `0.008` | servoJ dt (1/125s) |
| `SERVOJ_LOOKAHEAD` | `0.1` | servoJ lookahead (s) |
| `SERVOJ_GAIN` | `300` | servoJ gain |
| `JOINT_NAMES` | `[shoulder_pan, ...]` | UR10e 6 joints |
| `URDF_PATH` | `.docker/assets/ur10e.urdf` | Pinocchio URDF |

### 8.2 PinocchioIK (pinocchio_utils.py)

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `ee_frame` | `tool0` | EE 프레임 이름 |
| `damping` | `0.05` | DLS damping factor (λ) |
| `local` | `False` | `False`=base_link, `True`=tool0 |

---

## 9. 트러블슈팅

### "No joint states received" (모든 스크립트)

**원인**: 백엔드가 joint 데이터를 받지 못함.

**해결**:
- **RTDE 모드**: 로봇 IP 확인, 로봇 전원 및 네트워크 연결 확인
- **Sim 모드**: Isaac Sim 또는 mock hardware가 실행 중인지 확인
  ```bash
  ros2 topic echo /joint_states --once
  ```

### Sim 모드에서 로봇이 움직이지 않음

**확인 사항**:
1. `/joint_command` 토픽에 메시지가 도착하는지 확인:
   ```bash
   ros2 topic echo /joint_command
   ```
2. Isaac Sim / mock hardware가 `/joint_command`를 구독하고 있는지 확인
3. step 크기 또는 speed가 너무 작지 않은지 (`+`/`-` 키로 조절)

### RTDE 연결 실패

```
RuntimeError: ur_rtde: ...
```

**확인 사항**:
- 로봇 IP 핑 확인: `ping 192.168.0.2`
- 로봇이 Remote Control 모드인지 확인 (Teach Pendant)
- 다른 RTDE 클라이언트가 연결되어 있지 않은지 확인

### Singularity 근처에서 느린 동작

**원인**: DLS damping이 singularity 근처에서 자동으로 joint velocity를 제한합니다.

**해결**: 이것은 정상 동작입니다. MoveIt Servo처럼 emergency stop이 발생하지 않고 자연스럽게 감속됩니다.

**UR 로봇의 주요 singularity**:
- **Wrist singularity**: `wrist_2_joint ≈ 0` (joint 4, 6 축이 정렬)
- **Shoulder singularity**: wrist center가 joint 1 축을 통과
- **Elbow singularity**: 팔꿈치가 완전히 펴짐

### Joystick이 감지되지 않음

```bash
# 디바이스 확인
ls /dev/input/js*

# joy_node 실행 확인
ros2 topic echo /joy
```

### Admittance가 sim 모드에서 동작하지 않음

정상입니다. Sim 모드에서는 F/T 센서가 없으므로 `NullFTSource`가 사용되며 admittance 제어가 자동으로 비활성됩니다. 키보드 Cartesian 제어는 정상 동작합니다.
