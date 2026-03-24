# UR10e Impedance Teleop 사용자 가이드

## 개요

URScript 내부 PD 토크 루프(500Hz)와 Python 텔레옵 파이프라인(125Hz)을 결합한
관절 공간 임피던스 제어 기반 원격 조종 시스템.

UR `direct_torque()` API (PolyScope 5.23.0+)를 활용하여 로봇이 외력에 대해
물리적으로 순응(compliant)하게 동작합니다.

**파이프라인:**
```
Python (125Hz)                          URScript (500Hz)
──────────────                          ─────────────────
Input → ExpFilter → Workspace Clamp     RTDE 레지스터 읽기
    → Pink IK → q_desired              τ = Kp*(q_d - q) - Kd*qd + C(q,qd)
    → RTDE registers ──────────►       direct_torque(τ)
```

**두 가지 백엔드:**
- **rtde** — 실제 UR10e 로봇 (URScript PD 토크 제어 500Hz, **PolyScope 5.23.0+ 필수**)
- **sim** — ROS2 mock hardware (위치 제어 fallback, 토크 미지원)


## 어드미턴스 vs 임피던스 비교

| 항목 | teleop_admittance | teleop_impedance |
|------|------------------|------------------|
| 제어 방식 | 위치 명령 (servoJ) | 토크 명령 (direct_torque) |
| 컴플라이언스 | F/T 센서 → 가상 M-D-K | PD 게인에 의한 물리적 순응 |
| 로봇측 주파수 | 125Hz (Python) | 500Hz (URScript) |
| 외력 반응 | 소프트웨어 계산 변위 | 기계적 스프링-댐퍼 거동 |
| F/T 센서 필요 | 예 (admittance 활성 시) | 아니오 |
| PolyScope 요구 | 제한 없음 | **5.23.0+** |
| 접촉 작업 적합성 | 양호 | 우수 (직접 토크 제어) |


## 요구 사항

- **PolyScope 5.23.0 이상** (RTDE 모드)
  - `direct_torque()`, `get_coriolis_and_centrifugal_torques()` API 필요
  - 버전 미만 시 URScript 업로드 실패
- **ur_rtde** Python 라이브러리 (`pip install ur-rtde`)
- **Pink IK**: `pip install pin-pink proxsuite` (주의: `pip install pink`은 코드 포매터!)
- **numpy < 2**: `pip install "numpy<2"` (ROS Humble pinocchio 호환)
- **Xbox 컨트롤러 사용 시**: `pip install pygame`


## 실행 방법

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드 (mock hardware — 위치 제어 fallback, 토크 없음)
python3 -m standalone.teleop_impedance.main --mode sim --input keyboard

# 실제 로봇 (임피던스 토크 제어)
python3 -m standalone.teleop_impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2

# Xbox 컨트롤러 + CSV 로깅
python3 -m standalone.teleop_impedance.main --mode rtde --input xbox --robot-ip 192.168.0.2 --log

# 커스텀 설정 파일
python3 -m standalone.teleop_impedance.main --config path/to/config.yaml
```

### Sim 모드 사전 준비

```bash
# 별도 터미널에서 mock hardware driver 실행
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
```

### CLI 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `--mode` | config 파일 | `sim` 또는 `rtde` (백엔드 선택) |
| `--input` | config 파일 | `keyboard` 또는 `xbox` (입력 장치) |
| `--robot-ip` | `192.168.0.2` | RTDE 모드 로봇 IP |
| `--config` | `config/default.yaml` | 설정 파일 경로 |
| `--log` | off | CSV 로깅 활성화 (`impedance_teleop_log_YYYYMMDD_HHMMSS.csv`) |


## 키보드 조작

### 이동 (Cartesian)

| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| W / S | X축 +/- | U / O | Roll +/- |
| A / D | Y축 +/- | I / K | Pitch +/- |
| Q / E | Z축 +/- | J / L | Yaw +/- |

### 시스템 제어

| 키 | 동작 |
|----|------|
| +/= | 속도 증가 |
| - | 속도 감소 |
| Space | E-Stop 발동 |
| R | E-Stop 해제 |
| ESC / X | 종료 |

### 임피던스 제어

| 키 | 동작 |
|----|------|
| 1 | STIFF 프리셋 (높은 강성) |
| 2 | MEDIUM 프리셋 (중간) |
| 3 | SOFT 프리셋 (높은 순응성, 기본값) |
| [ | 게인 스케일 감소 (0.25 단위, 최소 0.25x) |
| ] | 게인 스케일 증가 (0.25 단위, 최대 2.0x) |

### 속도 스케일

5단계 속도 조절: **0.5x → 1.0x → 2.0x → 4.0x → 8.0x**

기본 스텝 크기에 배율을 곱합니다:
- 위치: `cartesian_step × speed_scale` (기본 0.01m × 1.0 = 10mm/press)
- 회전: `rotation_step × speed_scale` (기본 0.05rad × 1.0 = 2.86°/press)

키를 **누르고 있으면** 타겟이 계속 누적되어 연속 이동합니다.


## Xbox 컨트롤러 조작

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 X/Y | 선형 X/Y 속도 |
| 오른쪽 스틱 X/Y | 각속도 Roll/Pitch |
| LT / RT 트리거 | 선형 Z 속도 (RT - LT) |
| LB / RB 버튼 | 각속도 Yaw |
| A 버튼 | E-Stop 해제 |
| B 버튼 | E-Stop 발동 |
| Start 버튼 | 종료 |

데드존: 0.1 (이하 입력 무시)


## 임피던스 제어 원리

### 관절 공간 PD 제어

```
τ = Kp × (q_desired - q_actual) - Kd × q̇_actual + C(q, q̇)
```

- **Kp**: 위치 강성 [Nm/rad] — 클수록 목표 위치를 강하게 추종
- **Kd**: 속도 감쇠 [Nm·s/rad] — 진동 억제 (임계 감쇠~과감쇠)
- **C(q, q̇)**: 코리올리/원심력 보상 (선택적, 기본 활성)
- **중력 보상**: `direct_torque()` API가 자동 처리
- **마찰 보상**: `friction_comp=True`로 활성화

### 게인과 순응성의 관계

| 특성 | 낮은 Kp (SOFT) | 높은 Kp (STIFF) |
|------|---------------|-----------------|
| 외력 반응 | 쉽게 밀림 | 강하게 저항 |
| 위치 추종 정밀도 | 낮음 | 높음 |
| 접촉 시 힘 | 작음 | 큼 |
| 적합한 작업 | 안전 우선, 초기 테스트 | 정밀 위치 제어 |

### 프리셋 게인 테이블

관절별 관성이 다르므로 게인도 관절별로 설정됩니다:

| 프리셋 | J1-J2 (shoulder) | J3 (elbow) | J4 (wrist1) | J5-J6 (wrist2-3) |
|--------|-------------------|------------|-------------|-------------------|
| **STIFF Kp** | 800 | 400 | 200 | 100, 50 |
| **STIFF Kd** | 40 | 20 | 10 | 5, 2.5 |
| **MEDIUM Kp** | 400 | 200 | 100 | 50, 25 |
| **MEDIUM Kd** | 20 | 10 | 5 | 2.5, 1.25 |
| **SOFT Kp** | 100 | 50 | 25 | 12.5, 6.25 |
| **SOFT Kd** | 10 | 5 | 2.5 | 1.25, 0.625 |

단위: Kp [Nm/rad], Kd [Nm·s/rad]

> 초기 게인은 추정치입니다. 실제 로봇에서 **SOFT부터 시작**하여 점진적으로 올리세요.

### 런타임 게인 튜닝

- `[` / `]` 키로 현재 프리셋의 모든 게인을 균일하게 스케일링
- 범위: 0.25x ~ 2.0x (0.25 단위)
- 프리셋 변경 시 스케일은 1.0x로 리셋


## 아키텍처 상세

### RTDE 레지스터 할당

| 레지스터 | 범위 | 방향 | 용도 |
|----------|------|------|------|
| `input_double_register_0..5` | 6개 | Python → URScript | q_desired [rad] |
| `input_double_register_6..11` | 6개 | Python → URScript | Kp [Nm/rad] |
| `input_double_register_12..17` | 6개 | Python → URScript | Kd [Nm·s/rad] |
| `input_double_register_18` | 1개 | Python → URScript | mode (0=idle, 1=active, -1=stop) |
| `input_double_register_19` | 1개 | Python → URScript | coriolis 보상 (1=on, 0=off) |
| `output_double_register_0..5` | 6개 | URScript → Python | applied torque [Nm] (모니터링) |
| `output_double_register_6` | 1개 | URScript → Python | heartbeat 카운터 |

### URScript PD 루프 (`impedance_pd.script`)

500Hz로 실행되는 URScript 내부 루프:

1. `read_input_float_register()`로 q_desired, Kp, Kd 읽기
2. `get_actual_joint_positions()`, `get_actual_joint_speeds()`로 현재 상태
3. PD 토크 계산: `τ = Kp*(q_d - q) - Kd*qd`
4. 코리올리 보상 추가 (활성 시): `τ += get_coriolis_and_centrifugal_torques(q, qd)`
5. 관절별 토크 포화: UR10e 스펙 [150, 150, 56, 56, 28, 28] Nm
6. `direct_torque(τ, friction_comp=True)` 실행

### ur_rtde 인터페이스

| 인터페이스 | 용도 |
|-----------|------|
| `RTDEIOInterface` | RTDE input register 쓰기 (`setInputDoubleRegister`) |
| `RTDEReceiveInterface` | 로봇 상태 읽기 (actual_q, actual_qd, TCP pose) |
| `RTDEControlInterface` | URScript 업로드 (`FLAG_CUSTOM_SCRIPT`, `setCustomScriptFile`) |

### URScript 안전 메커니즘

- **토크 포화**: 관절별 최대 토크 클램핑 (URScript 내부)
- **mode=-1 시 정지**: `stopj(4.0)` → 안전 감속
- **RTDE 연결 끊김**: URScript 루프 자동 종료 → position hold
- **`direct_torque()` 미호출 시**: UR 내장 position-hold 자동 복귀


## 안전 시스템 (5단계)

우선순위 높은 순서대로:

| 레벨 | 이름 | 트리거 | 동작 | 임계값 |
|------|------|--------|------|--------|
| 4 | **E-Stop** | Space/B 버튼 | 즉시 mode=0 (idle), 수동 리셋 필요 | - |
| 3 | **위치 편차** | \|q_desired - q_actual\| 초과 | 현재 위치로 q_desired 고정 | 0.3 rad (~17.2°) |
| 2 | **속도 제한** | \|qd\| 초과 | mode=0 (idle) 전환 | 1.0 rad/s |
| 1 | **통신 타임아웃** | 입력 없음 | mode=0 (idle) → 현재 위치 유지 | 100ms |
| 0 | **Workspace** | EE 위치 경계 초과 | 위치 클램핑 (거부 아님) | config 참조 |

### 어드미턴스 대비 차이점

- **타임아웃 더 타이트**: 100ms (어드미턴스: 200ms) — 토크 제어는 더 위험
- **위치 편차 감시** (레벨 3): 토크 모드 전용 — 의도치 않은 큰 편차 감지
- **속도 스케일링 없음**: 게인 기반 제어이므로 속도 비례 스케일링 불필요

### E-Stop 동작

1. Space키 → `mode=0` (idle) → URScript가 `sync()`만 실행 (position hold)
2. R키 → E-Stop 해제 → `mode=1` (active) → IK/필터/타겟을 현재 위치로 재동기화
3. **수동 해제만 가능** — 자동 해제 없음

### Workspace 경계

기본 workspace (base_link 기준):
```
X: [-0.8, 0.8] m
Y: [-0.8, 0.8] m
Z: [0.05, 1.2] m
```
경계에 도달하면 명령이 거부되지 않고 클램핑되어 경계 위에서 움직입니다.


## 설정 파일 (`config/default.yaml`)

```yaml
robot:
  ip: "192.168.0.2"
  mode: "sim"                 # "sim" (ROS2 토픽) | "rtde" (실제 로봇)

control:
  frequency_sim: 50           # Hz (sim 모드)
  frequency_rtde: 125         # Hz (rtde 모드, RTDE 레지스터 업데이트 주기)

input:
  type: "keyboard"            # "keyboard" | "xbox"
  cartesian_step: 0.01        # m/press (1x 속도 기준)
  rotation_step: 0.05         # rad/press (1x 속도 기준)
  xbox_linear_scale: 0.03     # Xbox 스틱 → 선형 속도 스케일
  xbox_angular_scale: 0.08    # Xbox 스틱 → 각속도 스케일

filter:
  alpha_position: 0.85        # EMA alpha (0~1, 높을수록 반응 빠름)
  alpha_orientation: 0.85     # slerp alpha

ik:
  position_cost: 1.0          # Pink FrameTask 위치 비용
  orientation_cost: 0.5       # Pink FrameTask 자세 비용
  posture_cost: 1.0e-3        # PostureTask 비용 (관절 중심화)
  damping: 1.0e-12            # QP 솔버 댐핑

safety:
  packet_timeout_ms: 100      # 입력 없으면 idle 전환 (ms)
  max_joint_vel: 1.0          # rad/s
  max_position_deviation: 0.3 # rad — q_desired vs q_actual 편차 허용치
  max_ee_velocity: 0.2        # m/s
  workspace:
    x: [-0.8, 0.8]            # base_link 기준 (m)
    y: [-0.8, 0.8]
    z: [0.05, 1.2]

impedance:
  default_preset: "SOFT"       # 안전하게 SOFT로 시작
  gain_scale_range: [0.25, 2.0]
  enable_coriolis_comp: true   # 코리올리/원심력 보상
```

### 주요 파라미터 튜닝 가이드

| 파라미터 | 효과 | 권장 |
|----------|------|------|
| `default_preset` | 시작 시 게인 수준 | 첫 테스트: `SOFT` |
| `enable_coriolis_comp` | 동적 보상 정확도 | `true` (빠른 동작 시 필수) |
| `packet_timeout_ms` | 입력 없음 허용 시간 | 로컬: 100, 무선: 200~500 |
| `max_position_deviation` | 편차 안전 임계 (rad) | 보수적: 0.2, 범용: 0.3 |
| `max_joint_vel` | 관절 속도 상한 | 테스트: 0.5, 숙련: 1.0 |
| `alpha_position` | 필터 반응성 | 0.7~0.95 |
| `cartesian_step` | 키 1회당 이동량 (m) | 0.005~0.02 |


## 제어 루프 상세

### RTDE 모드 (매 사이클 8ms = 125Hz)

```
1. 입력 읽기 (timeout 1ms)
2. 타겟 누적 (target += velocity_delta)
3. Exponential 필터 적용 (EMA + slerp)
4. Workspace 클램핑 (Level 0)
5. Pink IK 풀기 (QP solver, proxqp) → q_desired
6. RTDE로 로봇 상태 읽기 (q_actual, qd_actual)
7. Safety 검사 (Level 1~4)
8-a. 안전 시: q_desired를 RTDE 레지스터에 쓰기
8-b. 위험 시: q_actual을 q_desired로 설정 (position hold)
9. 터미널 상태 표시 + CSV 로깅
10. 타이밍 보정 (sleep)
```

### Sim 모드 (매 사이클 20ms = 50Hz)

동일 파이프라인이지만 step 8에서 `backend.send_joint_command(q_desired)` (위치 명령)을 사용합니다.
임피던스 게인은 무시됩니다 (파이프라인 동작 테스트 용도).


## 모듈 구조

```
standalone/teleop_impedance/
├── __init__.py               # 패키지 초기화
├── main.py                   # ImpedanceTeleopController — 메인 제어 루프
├── impedance_config.py       # ImpedanceConfig — YAML 설정 로더
├── impedance_gains.py        # ImpedanceGains + ImpedanceController — PD 게인 관리
├── urscript_manager.py       # URScriptManager — URScript 업로드 + RTDE I/O
├── torque_safety.py          # TorqueSafetyMonitor — 5단계 안전 시스템
├── scripts/
│   └── impedance_pd.script   # URScript PD 루프 (500Hz)
├── config/
│   └── default.yaml          # 기본 설정
└── docs/
    └── user_guide.md         # 이 문서
```

### 공유 모듈 (standalone/core/)

| 모듈 | 용도 |
|------|------|
| `core/input_handler.py` | 키보드/Xbox 입력 추상화 |
| `core/exp_filter.py` | EMA 위치 + slerp 자세 필터 |
| `core/pink_ik.py` | QP 기반 task-level IK |
| `core/robot_backend.py` | RobotBackend ABC + `create_backend()` |
| `core/controller_utils.py` | ControllerSwitcher (sim mode mock hw) |
| `standalone/config.py` | URDF_PATH, JOINT_NAMES, 토크 상수 |


## 터미널 상태 표시

실행 중 8줄 고정 상태 블록이 0.1초 간격으로 갱신됩니다:

```
  EE Pos : x= 0.7295  y=-0.5837  z= 0.5130 m
  EE RPY : R= -90.0  P=  -0.0  Y= -49.9 deg
  Joints : [ -49.9  -65.4   74.3    0.0   86.1    0.0] deg
  Vel    : 0.0450 m/s  |  Speed: 2.0x
  Safety : OK
  E-Stop : off (Space: trigger)
  Imped  : SOFT x1.00 | tau: [  0.0   0.0   0.0   0.0   0.0   0.0]
  Input  : 15ms ago  |  rtde 125Hz
```

- **Imped 라인**: 현재 프리셋, 게인 스케일, applied torque [Nm]
- **Safety 라인**: OK / ESTOP / DEVIATION / VEL_LIMIT / TIMEOUT / WS_CLAMP


## CSV 로그 형식

`--log` 플래그 사용 시 `impedance_teleop_log_YYYYMMDD_HHMMSS.csv` 파일 생성:

```csv
timestamp,ee_x,ee_y,ee_z,ee_roll,ee_pitch,ee_yaw,j1,j2,j3,j4,j5,j6,ee_vel,safety_status,tau1,tau2,tau3,tau4,tau5,tau6
1772934822.286684,0.729509,-0.583755,0.512759,-1.570796,-0.000012,-0.871000,...,0.045000,OK,0.0,0.0,0.0,0.0,0.0,0.0
```

어드미턴스 로그 대비 `tau1~tau6` 컬럼이 추가됩니다 (applied torque).


## 실제 로봇 첫 테스트 절차

1. **PolyScope 버전 확인**: 설정 → 정보에서 5.23.0 이상인지 확인
2. **SOFT 프리셋으로 시작** (기본값)
   ```bash
   python3 -m standalone.teleop_impedance.main --mode rtde --robot-ip 192.168.0.2
   ```
3. **로봇 정지 상태 확인**: 시작 후 URScript 업로드 완료까지 ~0.5초 대기
4. **키보드로 소량 이동**: W/S 키로 X축 이동 확인
5. **EE를 손으로 밀어보기**: SOFT 게인에서 부드럽게 밀려야 정상
6. **프리셋 변경**: `2` (MEDIUM) → `1` (STIFF) 순서로 강성 증가 테스트
7. **게인 미세조정**: `]`로 스케일 증가, `[`로 감소
8. **E-Stop 테스트**: Space → 로봇 정지 확인 → R으로 해제


## 트러블슈팅

### URScript 업로드 실패

- **PolyScope 5.23.0 미만**: `direct_torque()` API 미지원 → 업데이트 필요
- **다른 프로그램 실행 중**: 기존 URScript를 중단하고 임피던스 스크립트로 교체됨 (의도된 동작)
- **방화벽**: 포트 30004 (RTDE), 30002 (Secondary) 개방 필요

### 로봇이 안 움직임 (RTDE 모드)

- Heartbeat 카운터 확인: URScript가 실행 중이면 증가해야 함
- RTDE 연결 상태: `ur_rtde` 라이브러리 설치 확인 (`pip install ur-rtde`)
- mode 레지스터: 1이어야 활성 (0=idle, -1=stop)

### DEVIATION 안전 알림 반복

- q_desired와 q_actual 편차가 0.3rad 초과 시 발생
- **원인**: 너무 큰 스텝으로 급격한 이동 시도
- **해결**: `cartesian_step` 줄이기 또는 `max_position_deviation` 완화 (주의 필요)

### TIMEOUT 상태 지속

- 키보드 입력이 터미널 포커스를 잃으면 발생
- `packet_timeout_ms` 값을 늘려서 완화 가능 (무선 환경: 200~500ms)
- 100ms는 토크 모드에 적합한 타이트한 값 — 필요 시 조정

### Sim 모드에서 토크 관련 표시가 없음

- 정상 동작입니다. Mock hardware는 `direct_torque()`를 지원하지 않습니다.
- Sim 모드는 파이프라인 테스트 용도 (위치 제어 fallback)

### numpy 호환 에러

- `_ARRAY_API not found` → `pip install "numpy<2"` 실행
- ROS Humble의 pinocchio는 numpy 1.x로 컴파일됨

### controller_manager 에러 (sim 모드)

- Isaac Sim 사용 시 정상 — `controller_manager`가 없으므로 자동 스킵됨
- mock hardware 사용 시: `ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true` 먼저 실행
