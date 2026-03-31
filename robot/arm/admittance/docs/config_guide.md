# Teleop Admittance 설정 가이드

설정 항목 전체 레퍼런스, 변경 방법, 시나리오별 튜닝 가이드입니다.

---

## 목차

1. [설정 구조 개요](#1-설정-구조-개요)
2. [전체 설정 항목 상세](#2-전체-설정-항목-상세)
3. [설정 변경 방법](#3-설정-변경-방법)
4. [시나리오별 튜닝 가이드](#4-시나리오별-튜닝-가이드)

---

## 1. 설정 구조 개요

### 1.1 설정 파일

기본 설정 파일: `robot/arm/admittance/config/default.yaml`

YAML 파일은 7개 섹션으로 구성되며, 각 섹션은 Python dataclass에 1:1 매핑됩니다.

```
TeleopConfig (teleop_config.py)
├── robot:       RobotConfig        — 로봇 연결
├── control:     ControlConfig      — 제어 주파수
├── input:       InputConfig        — 입력 장치 설정
├── filter:      FilterConfig       — 스무딩 필터
├── ik:          IKConfig           — IK 솔버 파라미터
├── safety:      SafetyConfig       — 안전 시스템
│   └── workspace: WorkspaceConfig  — 작업 공간 범위
└── admittance:  AdmittanceConfig   — 어드미턴스 제어
```

### 1.2 우선순위

설정 값은 다음 우선순위로 결정됩니다:

```
CLI 인자 (--mode, --input, --robot-ip)  ← 최우선
    ↓
YAML 파일 (--config 또는 default.yaml)
    ↓
dataclass 기본값 (teleop_config.py)     ← 최하위
```

> **참고**: CLI 인자는 `robot.mode`, `input.type`, `robot.ip` 3개만 오버라이드 가능합니다. 그 외 항목은 YAML 파일에서 변경해야 합니다.

---

## 2. 전체 설정 항목 상세

### 2.1 `robot` — 로봇 연결

| 항목 | 타입 | 기본값 | CLI 오버라이드 | 설명 |
|------|------|--------|---------------|------|
| `mode` | str | `"sim"` | `--mode {sim\|rtde}` | 백엔드 모드 |
| `ip` | str | `"192.168.0.2"` | `--robot-ip IP` | 로봇 IP (rtde 모드) |

**`mode` 선택 기준**:

| 모드 | 백엔드 | 주파수 | F/T 센서 | 용도 |
|------|--------|--------|----------|------|
| `sim` | ROS2 topic (`/joint_states`, `/forward_position_controller`) | 50 Hz | NullFTSource (항상 0) | 시뮬레이션, 알고리즘 테스트 |
| `rtde` | ur_rtde (`servoJ`, `getActualTCPForce`) | 125 Hz | RTDEFTSource (실제 센서) | 실제 로봇 제어 |

> **주의**: `sim` 모드에서는 Terminal 1 (UR mock hardware 드라이버)이 먼저 실행 중이어야 합니다.

### 2.2 `control` — 제어 주파수

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `frequency_sim` | int | `50` | sim 모드 제어 루프 Hz |
| `frequency_rtde` | int | `125` | rtde 모드 제어 루프 Hz |

모드에 따라 자동 선택됩니다 (`TeleopConfig.frequency` property).

- `frequency_sim`: 50Hz면 충분. 올려도 mock hardware 응답 속도에 의해 제한됨
- `frequency_rtde`: **125Hz 권장**. UR의 servoJ 기본 주파수와 일치. 낮추면 움직임이 끊김, 높이면 제어 루프가 주파수를 못 맞출 수 있음

> **주의**: rtde 주파수를 125Hz 이하로 낮추면 UR 컨트롤러가 protective stop을 발생시킬 수 있습니다.

### 2.3 `input` — 입력 장치

| 항목 | 타입 | 기본값 | CLI 오버라이드 | 설명 |
|------|------|--------|---------------|------|
| `type` | str | `"keyboard"` | `--input {keyboard\|xbox\|network}` | 입력 장치 |
| `cartesian_step` | float | `0.01` | — | 키보드 1회 입력당 병진 이동량 (m) |
| `rotation_step` | float | `0.05` | — | 키보드 1회 입력당 회전량 (rad) |
| `xbox_linear_scale` | float | `0.03` | — | Xbox 스틱 → 병진 속도 스케일 |
| `xbox_angular_scale` | float | `0.08` | — | Xbox 스틱 → 회전 속도 스케일 |
| `network_port` | int | `9870` | — | 네트워크 조이스틱 UDP 포트 |
| `network_linear_scale` | float | `0.005` | — | 네트워크 조이스틱 병진 스케일 |
| `network_angular_scale` | float | `0.015` | — | 네트워크 조이스틱 회전 스케일 |

**입력 감도 조절**:

- **키보드**: `cartesian_step`과 `rotation_step`이 기본 이동량. 런타임에 `+`/`-` 키로 0.5x~8.0x 스케일 조절 가능
- **Xbox**: `xbox_linear_scale` / `xbox_angular_scale`이 스틱 최대 기울기 시 속도. 런타임에 D-pad 상/하로 스케일 조절
- **네트워크**: `network_linear_scale` / `network_angular_scale`은 Xbox보다 낮게 설정 (연속 입력이므로)

> **팁**: 키보드 `cartesian_step`을 0.005로 줄이면 더 정밀한 제어가 가능합니다. 대신 큰 이동이 필요하면 `+` 키로 스케일을 올리세요.

### 2.4 `filter` — 스무딩 필터

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `alpha_position` | float | `0.85` | 위치 EMA alpha (0~1) |
| `alpha_orientation` | float | `0.85` | 오리엔테이션 slerp alpha (0~1) |

**alpha 값의 의미**:

```
alpha = 1.0 → 필터 없음 (즉시 반응, 노이즈 그대로)
alpha = 0.85 → 새 입력 85% + 이전 값 15% (기본값, 빠른 응답 + 약간의 스무딩)
alpha = 0.5 → 50/50 (상당한 스무딩, 눈에 띄는 지연)
alpha = 0.3 → 30/70 (매우 부드럽지만 반응이 느림)
```

- **위치 필터**: 선형 EMA (Exponential Moving Average)
- **오리엔테이션 필터**: 쿼터니언 Slerp (gimbal lock 없음)

> **팁**: 로봇이 "떨리는" 느낌이면 alpha를 0.7~0.8로 낮추세요. 반응이 느린 느낌이면 0.9~1.0으로 올리세요.

### 2.5 `ik` — IK 솔버 파라미터

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `position_cost` | float | `1.0` | EE 위치 추적 가중치 |
| `orientation_cost` | float | `0.5` | EE 방향 추적 가중치 |
| `posture_cost` | float | `1e-3` | 관절 중립 자세 유지 가중치 |
| `damping` | float | `1e-12` | QP 솔버 댐핑 (특이점 방지) |

**가중치 비율의 의미**:

Pink IK는 QP (Quadratic Programming) 문제를 풀어 여러 태스크를 동시에 만족시킵니다. 가중치가 높을수록 해당 태스크가 우선됩니다.

| 설정 | 효과 |
|------|------|
| `position_cost > orientation_cost` (기본) | 위치가 우선, 방향은 약간 타협 가능 |
| `position_cost = orientation_cost` | 위치와 방향을 동등하게 추적 |
| `orientation_cost > position_cost` | 방향 유지가 우선 (예: 용접 자세 유지) |
| `posture_cost` 높이기 (0.01~0.1) | 관절이 중립 자세에 가깝게 유지됨 (작업 범위 제한) |
| `posture_cost` 낮추기 (1e-5) | 관절 자유도를 최대한 활용 (특이점 근처 주의) |

> **주의**: `damping`은 일반적으로 변경할 필요 없습니다. 너무 높이면 IK 응답이 느려집니다.

### 2.6 `safety` — 안전 시스템

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `packet_timeout_ms` | int | `200` | 입력 없음 타임아웃 (ms) |
| `max_joint_vel` | float | `0.5` | 최대 관절 속도 (rad/s) |
| `max_ee_velocity` | float | `0.1` | 최대 EE 속도 (m/s) |

**`safety.workspace`** — 작업 공간 범위 (base_link 기준, 미터):

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `x` | [float, float] | `[-0.8, 0.8]` | X축 범위 (좌/우) |
| `y` | [float, float] | `[-0.8, 0.8]` | Y축 범위 (앞/뒤) |
| `z` | [float, float] | `[0.05, 1.2]` | Z축 범위 (아래/위) |

**각 항목의 동작**:

- `packet_timeout_ms`: 이 시간 동안 입력이 없으면 현재 위치에서 정지 (hold). 입력 재개 시 자동 복구. 어드미턴스 활성화 중이면 타임아웃 없음 (외력이 입력 역할)
- `max_joint_vel`: 이 값을 초과하면 비례적으로 속도 스케일 다운. 안전 마진으로 작용
- `max_ee_velocity`: EE 속도 제한
- `workspace`: EE 위치가 범위를 벗어나면 범위 경계로 클램핑. 범위 밖으로 나갈 수 없음

> **팁**: `z` 하한을 `0.05`로 설정한 이유는 테이블 충돌 방지입니다. 테이블 높이에 맞게 조절하세요.

### 2.7 `admittance` — 어드미턴스 제어

| 항목 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `enabled_by_default` | bool | `false` | 시작 시 어드미턴스 활성화 여부 |
| `default_preset` | str | `"MEDIUM"` | 기본 프리셋 (STIFF/MEDIUM/SOFT/FREE) |
| `max_displacement_trans` | float | `0.15` | 최대 병진 변위 (m) |
| `max_displacement_rot` | float | `0.30` | 최대 회전 변위 (rad, ~17°) |
| `force_deadzone` | [6 floats] | `[3,3,3, 0.3,0.3,0.3]` | 힘/토크 데드존 [N,N,N, Nm,Nm,Nm] |
| `force_saturation` | float | `100.0` | 힘 포화 한계 (N, 초과 시 상태 리셋) |
| `torque_saturation` | float | `10.0` | 토크 포화 한계 (Nm, 초과 시 상태 리셋) |

**각 항목의 효과**:

- `enabled_by_default`: `true`면 시작 즉시 F/T 반응. `false`면 `T` 키로 수동 활성화. Xbox 모드에서는 항상 `true` 권장
- `default_preset`: 시작 시 M/D/K 파라미터 세트. 런타임에 `1`/`2`/`3`/`4` 키로 변경 가능
- `max_displacement_trans/rot`: 어드미턴스로 인한 EE 변위의 절대 상한. 이 값을 초과하면 변위가 클램핑되고 속도가 감쇠됨
- `force_deadzone`: 이 값 이하의 센서 노이즈는 무시. 센서가 시끄러우면 높이고, 민감한 반응이 필요하면 낮춤
- `force_saturation` / `torque_saturation`: 충돌 감지 역할. 초과 시 변위와 속도를 즉시 0으로 리셋

> **주의**: `max_displacement_trans`를 너무 크게 설정하면 어드미턴스가 workspace 경계를 넘어서 IK 실패가 발생할 수 있습니다. workspace 범위와 함께 고려하세요.

---

## 3. 설정 변경 방법

### 방법 1: default.yaml 직접 편집

가장 간단한 방법. 파일을 수정하면 다음 실행부터 적용됩니다.

```bash
# 편집
vi robot/arm/admittance/config/default.yaml

# 실행 (수정된 default.yaml 자동 로드)
python3 -m robot.arm.admittance.main --mode sim
```

> **장점**: 간편함
> **단점**: 기본 설정이 바뀌므로 원래 값을 기억해야 복원 가능

### 방법 2: 커스텀 YAML 파일

default.yaml을 복사하여 용도별 설정 파일을 만들고, `--config`로 지정합니다.

```bash
# 복사
cp robot/arm/admittance/config/default.yaml \
   robot/arm/admittance/config/hand_guiding.yaml

# 편집
vi robot/arm/admittance/config/hand_guiding.yaml

# 실행
python3 -m robot.arm.admittance.main --config robot/arm/admittance/config/hand_guiding.yaml
```

> **장점**: 용도별 프로파일 관리, default.yaml 보존
> **단점**: 파일 경로 지정 필요

**부분 YAML도 가능합니다.** YAML에 없는 섹션은 dataclass 기본값이 사용됩니다:

```yaml
# minimal_rtde.yaml — robot 섹션만 오버라이드
robot:
  mode: "rtde"
  ip: "192.168.0.100"
```

### 방법 3: CLI 인자 오버라이드

YAML 파일 없이 실행 시 변경 가능한 항목:

```bash
# mode + input + robot-ip 오버라이드
python3 -m robot.arm.admittance.main \
    --mode rtde \
    --input xbox \
    --robot-ip 192.168.0.100

# 커스텀 YAML + CLI 오버라이드 (CLI가 우선)
python3 -m robot.arm.admittance.main \
    --config my_config.yaml \
    --mode sim    # YAML에 rtde로 되어있어도 sim으로 오버라이드
```

| CLI 인자 | 오버라이드 대상 | 설명 |
|----------|---------------|------|
| `--mode {sim\|rtde}` | `robot.mode` | 백엔드 모드 |
| `--input {keyboard\|xbox\|network}` | `input.type` | 입력 장치 |
| `--robot-ip IP` | `robot.ip` | 로봇 IP |
| `--config PATH` | 전체 YAML | 커스텀 설정 파일 경로 |
| `--log` | — | CSV 로깅 활성화 (YAML에 없는 항목) |

---

## 4. 시나리오별 튜닝 가이드

### 4.1 실제 로봇 연결

sim에서 테스트 완료 후 실제 로봇으로 전환할 때:

```yaml
robot:
  mode: "rtde"
  ip: "192.168.0.2"       # 실제 로봇 IP

safety:
  max_joint_vel: 0.3      # 처음에는 보수적으로 (기본 0.5)
  max_ee_velocity: 0.05   # 처음에는 보수적으로 (기본 0.1)
  workspace:
    z: [0.10, 1.0]        # 테이블 높이에 맞게 하한 조절

admittance:
  enabled_by_default: false  # 처음에는 비활성화 → 동작 확인 후 T키로 활성화
```

또는 CLI로:

```bash
python3 -m robot.arm.admittance.main --mode rtde --robot-ip 192.168.0.2
```

> **첫 연결 체크리스트**:
> 1. E-Stop 물리 버튼 확인
> 2. `--mode rtde`로 시작
> 3. 저속으로 동작 확인 (`-` 키로 0.5x)
> 4. workspace 범위가 물리 환경과 맞는지 확인
> 5. `Z` 키로 F/T 센서 영점 보정 후 어드미턴스 활성화

### 4.2 핸드 가이딩 (최대 컴플라이언스)

손으로 로봇을 밀어서 자유롭게 움직이는 모드:

```yaml
admittance:
  enabled_by_default: true
  default_preset: "FREE"
  max_displacement_trans: 0.30    # 기본(0.15)의 2배 → 더 넓은 범위
  max_displacement_rot: 0.50      # 기본(0.30)보다 넓게
  force_deadzone: [2.0, 2.0, 2.0, 0.2, 0.2, 0.2]  # 더 민감하게

safety:
  workspace:
    x: [-1.0, 1.0]    # 작업 공간 확장
    y: [-1.0, 1.0]
    z: [0.05, 1.3]

input:
  type: "keyboard"     # 키보드는 백업용 (주로 외력으로 제어)
```

> **FREE 프리셋**: K=0이므로 외력을 제거해도 원래 위치로 돌아가지 않습니다. 댐핑만 있어 자연스럽게 정지합니다.

### 4.3 정밀 작업 (저속, 고강성)

조립/삽입 등 정밀한 위치 제어가 필요할 때:

```yaml
input:
  cartesian_step: 0.003    # 1회 이동량을 3mm로 줄임 (기본 10mm)
  rotation_step: 0.02      # 회전도 줄임 (기본 0.05 rad)

filter:
  alpha_position: 0.7      # 더 부드럽게 (기본 0.85)
  alpha_orientation: 0.7

ik:
  position_cost: 1.0
  orientation_cost: 1.0    # 방향도 정밀하게 (기본 0.5)

safety:
  max_joint_vel: 0.2       # 더 보수적으로
  max_ee_velocity: 0.03

admittance:
  default_preset: "STIFF"
  force_deadzone: [5.0, 5.0, 5.0, 0.5, 0.5, 0.5]  # 높은 데드존 → 미세 진동 방지
```

### 4.4 작업 공간 조절

로봇 설치 환경에 맞게 workspace를 조절합니다.

```yaml
safety:
  workspace:
    # 예시: 테이블 위 좁은 영역
    x: [-0.4, 0.4]       # 좌우 40cm
    y: [0.2, 0.7]        # 앞쪽 20~70cm (로봇 바로 앞 제외)
    z: [0.15, 0.6]       # 테이블(15cm) ~ 60cm 높이
```

> **주의**: workspace를 너무 좁게 설정하면 IK가 해를 찾지 못할 수 있습니다. 최소한 로봇 팔 길이의 30% 이상의 범위를 확보하세요.

### 4.5 입력 감도 조절

#### 키보드 — 이동량 기반

```yaml
input:
  cartesian_step: 0.005   # 줄이면 정밀, 늘리면 빠른 이동
  rotation_step: 0.03     # 줄이면 정밀 회전
```

런타임에 `+`/`-` 키로 0.5x ~ 8.0x 스케일 조절 가능. `cartesian_step * speed_scale`이 실제 이동량.

#### Xbox — 스틱 스케일 기반

```yaml
input:
  xbox_linear_scale: 0.05    # 올리면 더 빠른 병진 (기본 0.03)
  xbox_angular_scale: 0.12   # 올리면 더 빠른 회전 (기본 0.08)
```

스틱을 최대로 기울였을 때의 속도 = `scale * speed_scale`. 런타임에 D-pad 상/하로 스케일 조절.

#### 네트워크 조이스틱 — 연속 입력

```yaml
input:
  network_linear_scale: 0.005   # Xbox보다 낮게 (연속 입력이므로)
  network_angular_scale: 0.015
  network_port: 9870            # joystick_sender.py와 동일 포트
```

> **네트워크 입력**: 원격 PC에서 `joystick_sender.py`가 UDP로 조이스틱 데이터를 전송합니다. Xbox보다 스케일을 낮게 설정하는 이유는 네트워크 패킷이 매 사이클 도착하여 누적 효과가 크기 때문입니다.

### 4.6 필터 응답성 조절

| 상황 | alpha_position | alpha_orientation | 효과 |
|------|---------------|------------------|------|
| 즉각 반응 필요 | 0.95~1.0 | 0.95~1.0 | 필터 거의 없음, 입력에 즉시 반응 |
| 기본 (권장) | 0.85 | 0.85 | 빠른 응답 + 약간의 스무딩 |
| 부드러운 동작 | 0.6~0.7 | 0.6~0.7 | 상당한 스무딩, 데모/촬영용 |
| 매우 부드러움 | 0.3~0.5 | 0.3~0.5 | 느린 반응, 시각적으로 부드러움 |

> **위치와 오리엔테이션을 다르게 설정할 수도 있습니다.** 예: 위치는 빠르게 (0.9), 오리엔테이션은 부드럽게 (0.6) → 이동은 빠르지만 회전이 부드러운 동작.

---

## 참조 파일

| 파일 | 역할 |
|------|------|
| `robot/arm/admittance/config/default.yaml` | 기본 설정 (전체 항목 + 주석) |
| `robot/arm/admittance/teleop_config.py` | YAML 로더 + dataclass 정의 (기본값 포함) |
| `robot/arm/admittance/main.py` (line 435~458) | CLI argparse 정의 |
| `robot/core/compliant_control.py` | 어드미턴스 프리셋 M/D/K 값 |
