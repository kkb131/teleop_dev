# core/ 모듈별 핵심 가이드 — What · Why · How · 튜닝

> **난이도**: 중급
> **위치**: `robot/core/`
> **목적**: 각 모듈의 핵심 기능, 존재 이유, 동작 원리, 개인 튜닝 포인트를 빠르게 파악

---

## 목차

1. [robot_backend.py — 로봇 통신 추상화](#1-robot_backendpy--로봇-통신-추상화)
2. [ur_robot.py — RTDEBackend (실제 로봇)](#2-ur_robotpy--rtdebackend-실제-로봇)
3. [sim_robot.py — SimBackend (시뮬레이션)](#3-sim_robotpy--simbackend-시뮬레이션)
4. [trajectory_executor.py — 궤적 리샘플링 + 스트리밍](#4-trajectory_executorpy--궤적-리샘플링--스트리밍)
5. [controller_utils.py — ROS2 컨트롤러 전환](#5-controller_utilspy--ros2-컨트롤러-전환)
6. [kinematics.py — Pinocchio DLS IK](#6-kinematicspy--pinocchio-dls-ik)
7. [pink_ik.py — Pink QP IK](#7-pink_ikpy--pink-qp-ik)
8. [exp_filter.py — 지수 이동 평균 필터](#8-exp_filterpy--지수-이동-평균-필터)
9. [input_handler.py — 입력 장치 추상화](#9-input_handlerpy--입력-장치-추상화)
10. [joystick_sender.py — UDP 조이스틱 송신](#10-joystick_senderpy--udp-조이스틱-송신)
11. [ft_source.py — F/T 센서 추상화](#11-ft_sourcepy--ft-센서-추상화)
12. [compliant_control.py — 어드미턴스 제어](#12-compliant_controlpy--어드미턴스-제어)
13. [전체 튜닝 파라미터 요약](#13-전체-튜닝-파라미터-요약)
14. [모듈 간 의존성 다이어그램](#14-모듈-간-의존성-다이어그램)

---

## 1. robot_backend.py — 로봇 통신 추상화

### What (무엇)
로봇과의 통신을 추상화하는 **ABC(Abstract Base Class)**와 **Factory 함수**. 실제 로봇(RTDE)과 시뮬레이션(ROS2 토픽)을 동일한 인터페이스로 사용할 수 있게 한다.

```python
class RobotBackend(ABC):
    def connect() / disconnect()
    def get_joint_positions() -> List[float]
    def get_joint_velocities() -> List[float]
    def send_joint_command(positions: List[float])
    def get_tcp_force() -> List[float]       # 기본값: [0]*6
    def emergency_stop()                      # 기본값: no-op
    def speed_stop(deceleration=2.0)          # 기본값: no-op

def create_backend(mode: str, **kwargs) -> RobotBackend
    # "rtde" → RTDEBackend, "sim" → SimBackend
```

### Why (왜)
이 추상화가 없다면 **모든 기능 모듈(servo, teleop_admittance, teleop_impedance)이 각각 실제 로봇/시뮬레이션 분기 코드를 가져야 한다**. ABC 하나로 통일하면:
- 기능 모듈은 `get_joint_positions()`, `send_joint_command()` 만 호출하면 되고
- 실/시뮬 전환은 `create_backend("rtde")` ↔ `create_backend("sim")` 한 줄이면 끝

### How (어떻게)

```python
# robot_backend.py:51-59 — Factory 함수
def create_backend(mode: str, **kwargs) -> RobotBackend:
    if mode == "rtde":
        from robot.core.ur_robot import RTDEBackend   # Lazy import!
        return RTDEBackend(robot_ip=kwargs["robot_ip"])
    elif mode == "sim":
        from robot.core.sim_robot import SimBackend
        return SimBackend(**{k: v for k, v in kwargs.items() if k != "robot_ip"})
```

**핵심 패턴:**
- **Lazy import**: `rtde_control`이 설치 안 된 환경에서도 `sim` 모드는 사용 가능
- **Context Manager**: `with create_backend("rtde", robot_ip="...") as robot:` 로 자동 connect/disconnect
- **기본 구현 제공**: `get_tcp_force()`, `emergency_stop()` 등은 ABC에 기본 no-op 구현이 있어, 시뮬레이션 백엔드에서 일일이 오버라이드하지 않아도 됨

### 튜닝 포인트
| 파라미터 | 위치 | 설명 |
|----------|------|------|
| — | — | 이 모듈 자체에는 튜닝 파라미터 없음. 하위 백엔드(ur_robot, sim_robot)에서 튜닝 |

---

## 2. ur_robot.py — RTDEBackend (실제 로봇)

### What (무엇)
ur_rtde 라이브러리를 사용하여 UR10e와 **RTDE 프로토콜**로 통신하는 백엔드. `servoJ` 명령으로 125Hz 실시간 관절 제어를 수행한다.

```python
class RTDEBackend(RobotBackend):
    def __init__(self, robot_ip: str, frequency: int = 125)
    def send_joint_command(positions):
        self._ctrl.servoJ(positions, 0, 0, SERVOJ_DT, SERVOJ_LOOKAHEAD, SERVOJ_GAIN)
```

### Why (왜)
UR 로봇 제어 방법은 여러 가지(MoveIt, URScript, Dashboard)가 있지만, **실시간 위치 서보(servoJ)**는 ur_rtde가 가장 간단하고 저지연이다:
- MoveIt은 ROS2 전체 스택이 필요 → 복잡
- URScript `movel/movej`는 trajectory 단위로만 동작 → 실시간 불가
- `servoJ`는 **매 8ms마다 목표 관절 각도를 직접 전송** → 텔레옵에 적합

### How (어떻게)

```python
# ur_robot.py:59-60 — servoJ 핵심
def send_joint_command(self, positions: List[float]):
    self._ctrl.servoJ(positions, 0, 0, SERVOJ_DT, SERVOJ_LOOKAHEAD, SERVOJ_GAIN)
```

`servoJ(q, speed, accel, dt, lookahead, gain)` 파라미터:
- `speed=0, accel=0`: servoJ에서는 사용되지 않음 (0 전달)
- `dt`: 명령 주기 (0.008s = 125Hz)
- `lookahead`: 미래 예측 시간 — 클수록 부드럽지만 지연 증가
- `gain`: 비례 게인 — 클수록 응답이 빠르지만 오버슈트 위험

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 | 주의사항 |
|----------|------|--------|-----------|----------|
| `SERVOJ_DT` | config.py:38 | 0.008s (125Hz) | 로봇 컨트롤러 주기와 일치시킬 것 | UR 컨트롤러가 500Hz이므로 125Hz는 안전 |
| `SERVOJ_LOOKAHEAD` | config.py:39 | 0.1s | ↑ 부드러움, ↓ 응답성 | 0.03~0.2 범위. 너무 낮으면 진동 |
| `SERVOJ_GAIN` | config.py:40 | 300 | ↑ 빠른 추종, ↓ 부드러움 | 100~2000 범위. 너무 높으면 오버슈트 |
| `RTDE_FREQUENCY` | config.py:15 | 125 Hz | 보통 변경 불필요 | RTDEReceiveInterface 폴링 주기 |

> **실전 팁**: 로봇이 떨리면 `SERVOJ_GAIN`을 낮추고 `SERVOJ_LOOKAHEAD`를 높여본다. 반응이 느리면 반대로.

---

## 3. sim_robot.py — SimBackend (시뮬레이션)

### What (무엇)
ROS2 토픽(`/joint_states`, `/joint_command`)을 통해 Isaac Sim 또는 Mock Hardware와 통신하는 백엔드.

```python
class SimBackend(RobotBackend):
    def __init__(self, joint_states_topic="/joint_states",
                 joint_command_topic="/joint_command")
```

### Why (왜)
실제 로봇 없이도 **전체 파이프라인을 검증**할 수 있어야 한다:
- 새 기능 개발 시 로봇 없이 로직 테스트
- Mock hardware(`use_fake_hardware:=true`)로 ROS2 스택 전체 통합 테스트
- F/T 센서가 없으므로 `get_tcp_force()`는 항상 0 반환 (ABC 기본값)

### How (어떻게)

```python
# sim_robot.py:66-70 — 백그라운드 스레드에서 ROS2 spin
self._spin_thread = threading.Thread(
    target=rclpy.spin, args=(self._node,), daemon=True
)
self._spin_thread.start()
```

**핵심 설계:**
- **`daemon=True` 스레드**: 메인 프로세스 종료 시 자동 정리 (좀비 방지)
- **`_reorder()` 메서드**: JointState 메시지의 관절 순서가 URDF 순서와 다를 수 있어, `JOINT_NAMES` 기준으로 재정렬
- **10초 타임아웃**: `wait_for_first_msg()`로 토픽 수신 확인 — 시뮬레이터가 안 켜져 있으면 즉시 에러

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 |
|----------|------|--------|-----------|
| `joint_states_topic` | 생성자 | `/joint_states` | Isaac Sim의 토픽명이 다르면 변경 |
| `joint_command_topic` | 생성자 | `/joint_command` | Forward controller 토픽에 맞춤 |
| `timeout` | wait_for_first_msg | 10.0s | 네트워크 지연이 크면 늘림 |

---

## 4. trajectory_executor.py — 궤적 리샘플링 + 스트리밍

### What (무엇)
cuMotion이 생성한 궤적(40Hz)을 **125Hz로 리샘플링**하고, 정밀 타이밍으로 로봇에 스트리밍하는 모듈.

```python
def resample_trajectory(positions, source_dt=0.025, target_dt=0.008) -> np.ndarray
def validate_trajectory(trajectory, tolerance=0.05) -> List[str]  # 안전 검사
def check_start_match(planned_start, actual_joints, tolerance=0.05) -> bool
def execute_trajectory(robot, trajectory, command_dt=0.008)  # 스트리밍 실행
```

### Why (왜)
cuMotion(curobo)의 출력은 **40Hz** (25ms 간격)이지만, UR10e의 `servoJ`는 **125Hz** (8ms 간격)로 명령을 받아야 한다. 그냥 40Hz로 보내면:
- 로봇이 25ms 동안 같은 위치를 유지 → 계단식 움직임 (jerky)
- 제어 주기 불일치로 servoJ 경고 발생

리샘플링(선형 보간)으로 부드러운 125Hz 궤적을 생성한다.

### How (어떻게)

```python
# trajectory_executor.py:31-38 — 선형 보간 리샘플링
source_times = np.arange(n_points) * source_dt      # [0, 0.025, 0.05, ...]
target_times = np.arange(0, total_time + 0.5*target_dt, target_dt)  # [0, 0.008, 0.016, ...]
for j in range(n_joints):
    resampled[:, j] = np.interp(target_times, source_times, positions[:, j])
```

```python
# trajectory_executor.py:110-113 — 정밀 타이밍 (busy-wait)
t_target = t_start + (i + 1) * command_dt
while time.perf_counter() < t_target:
    pass  # CPU 점유하며 대기 (time.sleep()보다 정확)
```

**왜 `time.sleep()` 대신 busy-wait?**
- `time.sleep(0.008)`의 실제 정밀도는 OS 스케줄러에 따라 ±1~15ms
- busy-wait은 CPU를 점유하지만 **마이크로초 단위 정밀도** 보장
- 125Hz에서 1ms 오차도 궤적 추종 품질에 영향

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 |
|----------|------|--------|-----------|
| `command_dt` | execute_trajectory 인자 | 0.008s (125Hz) | `SERVOJ_DT`와 동일하게 유지 |
| `tolerance` (validate) | validate_trajectory | 0.05 rad | ↑ 관대한 검사, ↓ 엄격한 검사 |
| `tolerance` (start_match) | check_start_match | 0.05 rad (~2.9°) | 로봇 드리프트가 크면 늘림 |
| `MAX_JOINT_VEL_RAD_S` | config.py:28 | 2.094 rad/s | UR10e 사양. 다른 로봇이면 변경 |

---

## 5. controller_utils.py — ROS2 컨트롤러 전환

### What (무엇)
ROS2 `controller_manager`의 서비스를 호출하여 **trajectory 컨트롤러 ↔ forward_position 컨트롤러** 전환을 수행.

```python
class ControllerSwitcher:
    def activate_forward_position() -> bool   # servo용 컨트롤러 활성화
    def restore_original() -> bool            # 원래 컨트롤러 복원
    def switch_controller(start, stop) -> bool
```

### Why (왜)
Mock hardware에서는 기본적으로 `joint_trajectory_controller`가 활성화되어 있다. 이 컨트롤러는 **전체 궤적을 한 번에 받는 방식**이라 실시간 servo에 부적합. `forward_position_controller`는 **매 주기 관절 위치를 직접 전달**할 수 있다.

> **RTDE 모드에서는 불필요**: ur_rtde가 직접 servoJ를 호출하므로 ROS2 컨트롤러와 무관

### How (어떻게)

```python
# controller_utils.py:103-128 — forward_position 활성화
def activate_forward_position(self) -> bool:
    active = self.get_active_controllers()
    # 현재 활성 trajectory 컨트롤러를 저장 (나중에 복원용)
    self._original_active = [name for name in
        [TRAJECTORY_CONTROLLER, SCALED_TRAJECTORY_CONTROLLER] if name in active]
    return self.switch_controller(
        start=[FORWARD_POSITION_CONTROLLER],
        stop=self._original_active
    )
```

**핵심 설계:**
- **원본 저장 + 복원**: `_original_active`에 기존 컨트롤러를 저장 → `restore_original()`로 원복
- **서비스 호출**: `rclpy.spin_until_future_complete()`로 동기 호출

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 설명 |
|----------|------|--------|------|
| `strictness` | switch_controller | 2 (STRICT) | 1=BEST_EFFORT (실패 무시), 2=STRICT (실패 시 에러) |
| 컨트롤러명 | config.py:57-60 | `joint_trajectory_controller` 등 | 로봇 launch 파일과 일치시킬 것 |

---

## 6. kinematics.py — Pinocchio DLS IK

### What (무엇)
Pinocchio 라이브러리로 **FK(순기구학)**, **Jacobian**, **DLS(Damped Least Squares) 미분 IK**를 제공.

```python
class PinocchioIK:
    def get_ee_pose(q) -> (position[3], rotation[3x3])
    def get_jacobian(q, local=False) -> ndarray(6, nq)
    def compute_joint_delta(q, twist, dt, damping=0.05) -> dq
    def clamp_positions(q) -> q_clamped
```

### Why (왜)
MoveIt 없이도 **실시간 Cartesian 제어**가 가능해야 한다:
- MoveIt IK는 오버헤드가 커서 50Hz+ 실시간 제어에 부적합
- Pinocchio는 **C++ 바인딩**으로 1ms 이하에 FK/Jacobian 계산
- DLS는 **특이점(singularity) 근처에서도 안정적**

### How (어떻게)

DLS 공식: `Δq = J^T @ (J·J^T + λ²·I)^{-1} @ (twist × dt)`

```python
# kinematics.py:88-92 — DLS 핵심
J = self.get_jacobian(q, local=local)
JJt = J @ J.T
JJt_damped = JJt + (damping ** 2) * np.eye(6)
dq = J.T @ np.linalg.solve(JJt_damped, twist * dt)
```

**특이점 문제와 damping:**
- 특이점에서 `J·J^T`은 singular (역행렬 불가)
- `λ²·I`를 더하면 항상 역행렬이 존재 → 수치적 안정성
- 대가: 정확도가 약간 떨어짐 (목표 twist를 100% 달성 못함)

**Jacobian 프레임:**
- `local=False` (기본): **world-aligned** — base_link 기준 속도 명령에 적합
- `local=True`: **tool0 프레임** — 공구 기준 작업에 적합

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 | 주의사항 |
|----------|------|--------|-----------|----------|
| `damping` (λ) | compute_joint_delta | 0.05 | ↑ 안정성 (특이점 안전), ↓ 정확도 | 0.01~0.1 범위. 0이면 순수 의사역행렬 (불안정) |
| `ee_frame` | 생성자 | `"tool0"` | 커스텀 툴 프레임으로 변경 가능 | URDF에 정의된 프레임명이어야 함 |
| `URDF_PATH` | config.py:11 | `.docker/assets/ur10e.urdf` | 다른 로봇이면 해당 URDF 경로 |

> **DLS vs QP IK**: DLS는 빠르고 간단하지만 **관절 한계를 고려하지 않음**. 관절 한계가 중요하면 `pink_ik.py` 사용.

---

## 7. pink_ik.py — Pink QP IK

### What (무엇)
**QP(Quadratic Programming)** 기반 IK 솔버. **관절 한계를 제약 조건**으로 처리하고, 자세 정규화(posture regularization)로 불필요한 관절 움직임을 억제한다.

```python
class PinkIK:
    def initialize(q_current)
    def solve(target_pos, target_quat, dt) -> Optional[q]
    def get_ee_pose(q) -> (position[3], quaternion_xyzw[4])
    def sync_configuration(q)   # E-Stop 후 재동기화
```

### Why (왜)
DLS IK(`kinematics.py`)와의 차이:

| | DLS IK | Pink QP IK |
|---|---|---|
| 관절 한계 | **무시** (clamp으로 후처리) | **QP 제약 조건**으로 보장 |
| 자세 정규화 | 없음 | posture_cost로 중립 자세 유지 |
| 속도 | ~0.1ms | ~0.5ms (QP 풀이) |
| 사용처 | servo/ (간단 teleop) | teleop_admittance/, teleop_impedance/ |

텔레옵에서는 사용자의 임의 입력이 관절 한계를 넘길 수 있으므로, **QP 제약이 안전에 필수**적이다.

### How (어떻게)

```python
# pink_ik.py:76-92 — QP IK 풀이 핵심
# 1. 목표 SE3 구성
quat_pin = pin.Quaternion(w, x, y, z)  # Pinocchio는 wxyz!
target_se3 = pin.SE3(quat_pin.matrix(), target_pos)
self._ee_task.set_target(target_se3)

# 2. QP 풀이
velocity = pink.solve_ik(
    self._config, self._tasks, dt,
    solver="proxqp",          # ProxQP (빠르고 안정적)
    damping=self._damping,    # 수치 안정화 (1e-12)
)

# 3. 속도를 관절 위치로 적분
self._config.integrate_inplace(velocity, dt)
return self._config.q.copy()
```

**쿼터니언 주의:** 입출력은 `[x,y,z,w]` (로봇 관례) 이지만, Pinocchio 내부는 `[w,x,y,z]`. 변환 필수!

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 | 주의사항 |
|----------|------|--------|-----------|----------|
| `position_cost` | 생성자 | 1.0 | ↑ 위치 추종 우선, ↓ 방향 우선 | orientation_cost와의 비율이 중요 |
| `orientation_cost` | 생성자 | 0.5 | ↑ 방향 추종 우선 | 위치보다 낮으면 위치 우선 |
| `posture_cost` | 생성자 | 1e-3 | ↑ 중립 자세 유지력, ↓ EE 추종 우선 | 너무 높으면 EE가 목표에 못 감 |
| `damping` | 생성자 | 1e-12 | QP 수치 안정화. 보통 변경 불필요 | DLS의 damping과 다른 역할 |

> **설치 주의**: `pip install pin-pink proxsuite` 필수. `pip install pink`는 코드 포맷터!

---

## 8. exp_filter.py — 지수 이동 평균 필터

### What (무엇)
**위치는 EMA**, **방향은 Slerp**로 목표 포즈를 스무딩하는 필터.

```python
class ExpFilter:
    def reset(position, quaternion)
    def update(position, quaternion) -> (filtered_pos, filtered_quat)
```

### Why (왜)
조이스틱/키보드 입력은 **이산적(discrete)**이다. 누적된 목표 위치가 매 프레임 작은 점프를 하면 IK 출력도 급변 → 로봇이 떨린다. 필터로 목표를 부드럽게 만들면:
- 로봇 모터에 가해지는 급격한 명령이 완화
- 사용자 체감으로도 자연스러운 움직임

**왜 속도가 아닌 목표 포즈를 필터링하는가?**
속도(delta)를 필터링하면 **적분 드리프트**가 생긴다. 목표 포즈 자체를 필터링하면 드리프트 없이 항상 최종 목표에 수렴한다.

### How (어떻게)

```python
# exp_filter.py:48-55 — EMA + Slerp
# 위치: y = α·x_new + (1-α)·y_prev
filtered_pos = self._alpha_pos * position + (1.0 - self._alpha_pos) * self._prev_pos

# 방향: slerp(q_prev, q_new, α)
new_quat = pin.Quaternion(w, x, y, z)
filtered_quat = self._prev_quat.slerp(self._alpha_ori, new_quat)
```

**왜 방향에 EMA가 아닌 Slerp?**
쿼터니언은 4D 단위구(unit sphere) 위의 점이다. 단순 선형 보간(EMA)을 하면 단위 노름이 깨져서 회전이 아닌 것이 된다. **Slerp는 구면 위에서 보간**하므로 항상 유효한 회전을 반환한다.

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 |
|----------|------|--------|-----------|
| `alpha_pos` | 생성자 | 0.7 | ↑ 응답 빠름 (1.0=필터 없음), ↓ 부드러움 (0.1=매우 느림) |
| `alpha_ori` | 생성자 | 0.7 | 위와 동일. 회전에 별도 설정 가능 |

> **실전 팁**: 처음엔 0.5로 시작하여 로봇 떨림을 확인. 떨리면 ↓, 반응이 느리면 ↑.

---

## 9. input_handler.py — 입력 장치 추상화

### What (무엇)
3종 입력(키보드, Xbox, 네트워크 UDP)을 **동일한 `TeleopCommand`**로 통합하는 모듈.

```python
@dataclass
class TeleopCommand:
    velocity: np.ndarray    # [vx, vy, vz, wx, wy, wz]
    estop: bool             # E-Stop
    speed_scale: float      # 0.5x ~ 8.0x
    admittance_cycle: bool  # 프리셋 순환
    tool_z_delta: float     # 공구 Z축 이동
    ...

class InputHandler(ABC):
    def setup() / cleanup()
    def get_command(timeout) -> TeleopCommand

class KeyboardInput(InputHandler)   # termios 기반
class XboxInput(InputHandler)       # pygame 기반
class NetworkInput(InputHandler)    # UDP 수신

def create_input(input_type: str, ...) -> InputHandler  # Factory
```

### Why (왜)
제어 루프 코드가 입력 장치를 **전혀 몰라야** 한다. `--input keyboard/xbox/network` 플래그 하나로 교체 가능하고, 새 입력 장치 추가 시 `InputHandler`만 구현하면 된다.

### How (어떻게)

**KeyboardInput** — Non-blocking termios:
```python
# select()로 키 입력 대기 (blocking 방지)
rlist, _, _ = select.select([sys.stdin], [], [], timeout)
```

**XboxInput** — pygame 이벤트:
```python
# deadzone 적용
def dz(val, threshold=0.1):
    return val if abs(val) > threshold else 0.0
```

**NetworkInput** — UDP 버퍼 drain:
```python
# 쌓인 패킷 중 마지막만 사용 (stale 데이터 방지)
data = None
try:
    while True:
        data, _ = self._sock.recvfrom(4096)
except BlockingIOError:
    pass  # 더 이상 패킷 없음
```

### 튜닝 포인트
| 파라미터 | 클래스 | 기본값 | 조절 방향 |
|----------|--------|--------|-----------|
| `cartesian_step` | KeyboardInput | 0.005m | ↑ 큰 이동, ↓ 정밀 이동 |
| `rotation_step` | KeyboardInput | 0.05 rad | ↑ 큰 회전, ↓ 정밀 회전 |
| `linear_scale` | Xbox/Network | 0.02 | ↑ 빠른 이동. 네트워크는 0.005 권장 |
| `angular_scale` | Xbox/Network | 0.05 | ↑ 빠른 회전. 네트워크는 0.015 권장 |
| `deadzone` | Xbox/Network | 0.1 | ↑ 노이즈 억제, ↓ 민감도 |
| UDP `port` | NetworkInput | 9870 | 방화벽 확인 필수 |

> **네트워크 스케일이 Xbox보다 낮은 이유**: 네트워크는 50Hz 연속 스트림이라 누적 효과가 큼.

---

## 10. joystick_sender.py — UDP 조이스틱 송신

### What (무엇)
서버PC에서 pygame으로 조이스틱을 읽어 **UDP JSON 패킷**으로 로봇PC에 전송하는 스크립트.

```bash
python3 joystick_sender.py --target-ip <로봇IP> --port 9870 --hz 50
```

### Why (왜)
로봇PC(Jetson 등)에 조이스틱을 직접 연결할 수 없는 환경에서 **네트워크를 통해 원격 입력을 전달**한다.

> 상세 내용은 [udp_joystick_remote.md](udp_joystick_remote.md) 참조

### How (어떻게)

```python
# joystick_sender.py:54-59 — raw 값 전송 (가공 없음)
axes = [js.get_axis(i) for i in range(js.get_numaxes())]
buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
hat = list(js.get_hat(0))
pkt = json.dumps({"axes": axes, "buttons": buttons, "hat": hat})
sock.sendto(pkt.encode(), target)
```

**핵심 설계**: Sender는 **raw 값만 전송**. deadzone·스케일링은 수신측(NetworkInput)에서 처리 → 로봇 측에서 자유롭게 튜닝 가능.

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 |
|----------|------|--------|-----------|
| `--hz` | CLI 인자 | 50 | ↑ 더 부드러운 입력 (대역폭 증가), ↓ 낮은 대역폭 |
| `--port` | CLI 인자 | 9870 | NetworkInput과 일치시킬 것 |

---

## 11. ft_source.py — F/T 센서 추상화

### What (무엇)
F/T(Force/Torque) 센서 데이터를 **Protocol(덕 타이핑)**으로 추상화. 실제 센서, 좌표 변환 래퍼, null 소스 3가지 구현.

```python
class FTSource(Protocol):
    def get_wrench() -> np.ndarray  # [fx, fy, fz, tx, ty, tz]
    def zero_sensor() -> None       # Bias 보정

class RTDEFTSource       # 실제 UR10e F/T
class BaseFrameFTSource  # TCP→base 프레임 변환 래퍼
class NullFTSource       # 시뮬레이션용 (항상 0)
```

### Why (왜)
- **Bias 보정**: UR10e F/T 센서는 중력 영향으로 무부하 상태에서도 0이 아님. `zero_sensor()`로 현재 값을 빼야 정확한 외력 측정 가능
- **좌표 변환**: UR의 `getActualTCPForce()`는 TCP 프레임 기준. 어드미턴스 제어는 base 프레임 기준이므로 변환 필요
- **Protocol 사용**: ABC 상속 없이 `get_wrench()`와 `zero_sensor()`만 있으면 어떤 객체든 사용 가능

### How (어떻게)

```python
# ft_source.py:84-87 — BaseFrameFTSource: X,Y 부호 반전
def get_wrench(self) -> np.ndarray:
    w = self._source.get_wrench()
    return np.array([-w[0], -w[1], w[2], -w[3], -w[4], w[5]])
```

**왜 "negate X,Y" 방식인가?**
UR10e의 EE(End-Effector) 커넥터가 아래를 향한 상태에서, TCP 프레임의 X/Y가 base 프레임과 반대. 이 방법은 **실험적으로 검증됨** (test_wrench_frame.py --servo).

소스 코드에 6가지 변환 후보(#1~#6)가 주석으로 문서화되어 있어, 마운트/IK 변경 시 참고하면 된다.

### 튜닝 포인트
| 파라미터 | 위치 | 기본값 | 조절 방향 |
|----------|------|--------|-----------|
| 변환 방법 (#1~#6) | BaseFrameFTSource.get_wrench | #4 (negate X,Y) | **로봇/마운트 변경 시 반드시 재검증** |
| bias | RTDEFTSource | 최초 0 | `zero_sensor()` 호출로 자동 설정 |

> **중요**: 로봇 자세가 바뀌면 (예: EE가 옆을 향함) 변환 방법이 달라질 수 있다. `test_wrench_frame.py`로 확인.

---

## 12. compliant_control.py — 어드미턴스 제어

### What (무엇)
2차 어드미턴스 동역학 `M·ẍ + D·ẋ + K·x = F_ext`을 구현. 외력에 비례하여 로봇이 유연하게 움직이도록 한다.

```python
@dataclass
class ComplianceParams:
    M: np.ndarray  # 가상 질량 [6]
    D: np.ndarray  # 감쇠 [6]
    K: np.ndarray  # 강성 [6]

COMPLIANCE_PRESETS = { "STIFF", "MEDIUM", "SOFT", "FREE" }

class AdmittanceController(CompliantController):
    def update(f_ext, dt) -> displacement[6]
    def reset()
    def set_params(params)
```

### Why (왜)
사람이 로봇을 직접 밀었을 때 로봇이 **유연하게 반응**해야 안전하고 협업이 가능하다:
- **STIFF**: 외력에 조금만 밀림 → 정밀 작업
- **MEDIUM**: 적당한 유연성 → 일반 협업
- **SOFT**: 쉽게 밀림 → 핸드가이딩
- **FREE**: 스프링 없음, 감쇠만 → 자유 이동

### How (어떻게)

```python
# compliant_control.py:127-139 — 어드미턴스 업데이트 핵심
# 1. Deadzone 적용 (센서 노이즈 제거)
f = f_ext.copy()
mask = np.abs(f) < self._force_deadzone
f[mask] = 0.0
f[~mask] -= np.sign(f[~mask]) * self._force_deadzone[~mask]

# 2. 2차 동역학: M·ẍ = F - D·ẋ - K·x
xddot = (f - p.D * self._xdot - p.K * self._x) / p.M

# 3. Euler 적분
self._xdot += xddot * dt
self._x += self._xdot * dt

# 4. 안전: 최대 변위 클램핑
if np.linalg.norm(self._x[:3]) > self._max_disp_trans:
    self._x[:3] *= self._max_disp_trans / disp_norm
    self._xdot[:3] *= 0.5   # 속도도 감쇠
```

**Deadzone 뺄셈 방식**: 단순히 0으로 만드는 게 아니라 `f -= deadzone * sign(f)` 하여 **deadzone 경계에서 불연속이 없도록** 한다.

**Saturation 보호**: 100N 또는 10Nm 초과 시 내부 상태를 reset하여 **폭주 방지**.

### 튜닝 포인트

#### M, D, K 프리셋

| 프리셋 | M (질량) | D (감쇠) | K (강성) | 체감 |
|--------|----------|----------|----------|------|
| STIFF | [10, 10, 10, 1, 1, 1] | [200, 200, 200, 20, 20, 20] | [500, 500, 500, 50, 50, 50] | 무거운 느낌, 작은 변위 |
| MEDIUM | [5, 5, 5, 0.5, 0.5, 0.5] | [100, 100, 100, 10, 10, 10] | [200, 200, 200, 20, 20, 20] | 적당한 유연성 |
| SOFT | [2, 2, 2, 0.2, 0.2, 0.2] | [40, 40, 40, 4, 4, 4] | [50, 50, 50, 5, 5, 5] | 가벼운 느낌, 큰 변위 |
| FREE | [2, 2, 2, 0.2, 0.2, 0.2] | [30, 30, 30, 3, 3, 3] | [0, 0, 0, 0, 0, 0] | 스프링 없음, 밀면 계속 감 |

#### 안전 파라미터

| 파라미터 | 기본값 | 조절 방향 |
|----------|--------|-----------|
| `max_disp_trans` | 0.15m (150mm) | ↑ 더 멀리 밀림 허용, ↓ 안전 |
| `max_disp_rot` | 0.3 rad (~17°) | ↑ 더 큰 회전 허용, ↓ 안전 |
| `force_deadzone` | [3, 3, 3, 0.3, 0.3, 0.3] N/Nm | ↑ 노이즈 억제, ↓ 민감도 |
| `force_saturation` | 100 N | 초과 시 내부 상태 reset |
| `torque_saturation` | 10 Nm | 초과 시 내부 상태 reset |

> **커스텀 프리셋**: `ComplianceParams(M=..., D=..., K=...)`로 축별 다른 값 설정 가능. 예: Z축만 SOFT, X/Y는 STIFF.

> **M, D, K 관계 팁**:
> - `D = 2·√(M·K)` 이면 **임계 감쇠** (진동 없이 수렴)
> - `D < 2·√(M·K)` 이면 **과소 감쇠** (진동 발생)
> - `D > 2·√(M·K)` 이면 **과대 감쇠** (느린 수렴)

---

## 13. 전체 튜닝 파라미터 요약

### 로봇 통신

| 파라미터 | 파일 | 기본값 | 용도 |
|----------|------|--------|------|
| `RTDE_FREQUENCY` | config.py | 125 Hz | RTDE 폴링 주기 |
| `SERVOJ_DT` | config.py | 0.008s | servoJ 명령 주기 |
| `SERVOJ_LOOKAHEAD` | config.py | 0.1s | 궤적 예측 시간 |
| `SERVOJ_GAIN` | config.py | 300 | 서보 비례 게인 |

### IK 솔버

| 파라미터 | 파일 | 기본값 | 용도 |
|----------|------|--------|------|
| DLS `damping` | kinematics.py | 0.05 | 특이점 안정화 |
| QP `position_cost` | pink_ik.py | 1.0 | EE 위치 가중치 |
| QP `orientation_cost` | pink_ik.py | 0.5 | EE 방향 가중치 |
| QP `posture_cost` | pink_ik.py | 1e-3 | 중립 자세 정규화 |

### 필터 및 입력

| 파라미터 | 파일 | 기본값 | 용도 |
|----------|------|--------|------|
| `alpha_pos` / `alpha_ori` | exp_filter.py | 0.7 / 0.7 | 포즈 스무딩 계수 |
| Xbox `linear_scale` | input_handler.py | 0.02 | 조이스틱 병진 감도 |
| Xbox `angular_scale` | input_handler.py | 0.05 | 조이스틱 회전 감도 |
| Network `linear_scale` | teleop_config.py | 0.005 | 네트워크 병진 감도 |
| `deadzone` | input_handler.py | 0.1 | 조이스틱 불감대 |

### 어드미턴스 제어

| 파라미터 | 파일 | 기본값 | 용도 |
|----------|------|--------|------|
| M / D / K | compliant_control.py | 프리셋별 상이 | 가상 동역학 파라미터 |
| `force_deadzone` | compliant_control.py | [3,3,3,0.3,0.3,0.3] | 센서 노이즈 억제 |
| `max_disp_trans` | compliant_control.py | 0.15m | 최대 병진 변위 |
| `max_disp_rot` | compliant_control.py | 0.3 rad | 최대 회전 변위 |

---

## 14. 모듈 간 의존성 다이어그램

```
config.py (JOINT_NAMES, URDF_PATH, servoJ 파라미터, 안전 한계)
    │
    ├─→ robot_backend.py (ABC)
    │     ├─→ ur_robot.py (RTDEBackend)  ──→ 모든 기능 모듈 (rtde 모드)
    │     └─→ sim_robot.py (SimBackend)  ──→ 모든 기능 모듈 (sim 모드)
    │
    ├─→ trajectory_executor.py  ──→ cumotion/ (궤적 실행)
    │
    ├─→ kinematics.py (DLS IK)  ──→ servo/ (keyboard_cartesian, joystick_cartesian)
    │
    ├─→ pink_ik.py (QP IK)  ──→ teleop_admittance/, teleop_impedance/
    │
    ├─→ exp_filter.py  ──→ teleop_admittance/, teleop_impedance/
    │
    ├─→ input_handler.py  ──→ servo/, teleop_admittance/, teleop_impedance/
    │     └─ NetworkInput  ←── joystick_sender.py (UDP)
    │
    ├─→ ft_source.py  ──→ teleop_admittance/
    │
    ├─→ compliant_control.py  ──→ teleop_admittance/
    │
    └─→ controller_utils.py  ──→ servo/ (sim 모드 전용)
```

**읽는 법**: `A ──→ B` 는 "B가 A를 import해서 사용"

**설계 원칙**: 기능 모듈(servo, teleop_*)은 core/만 import. 기능 모듈끼리는 서로 import 금지.
