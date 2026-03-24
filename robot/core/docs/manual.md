# core/ 학습 매뉴얼 — 공유 인프라

> **난이도**: 기초 (모든 기능 모듈의 선수 지식)
> **위치**: `standalone/core/`
> **목적**: 2개 이상의 기능 모듈이 공유하는 인프라 코드

---

## 1. 개요: core/가 존재하는 이유

standalone 패키지의 핵심 설계 원칙 중 하나는 **"기능 모듈 간 코드 중복 금지"**이다.
servo, cumotion, teleop_admittance, teleop_impedance 모두 로봇과 통신하고, IK를 풀고, 입력을 처리한다.
이 공통 로직을 core/에 모아두면:

- **코드 중복 제거**: 동일 로직을 여러 곳에 복사하지 않음
- **일관성 보장**: 로봇 통신 방식이 바뀌면 한 곳만 수정
- **테스트 용이**: 공통 모듈을 독립적으로 테스트 가능

### 모듈별 의존성

```
                    ┌─────────────────────────┐
                    │       config.py          │  ← 모든 모듈이 import
                    └────────────┬────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                   ▼
     ┌─────────────┐   ┌───────────────┐   ┌──────────────┐
     │ robot_backend│   │  kinematics   │   │ input_handler│
     │ ur_robot     │   │  pink_ik      │   │              │
     │ sim_robot    │   │               │   │              │
     └──────┬──────┘   └───────┬───────┘   └──────┬───────┘
            │                   │                   │
     ┌──────┼───────────────────┼───────────────────┤
     │      │                   │                   │
     ▼      ▼                   ▼                   ▼
  cumotion  servo         teleop_admittance   teleop_impedance
  (backend  (backend      (backend+pink_ik    (backend+pink_ik
   +traj)    +kinematics   +filter+ft         +filter+gains)
             +input)       +compliant+input)   +input)
```

| core 모듈 | 사용하는 기능 모듈 |
|-----------|------------------|
| `robot_backend` + `ur_robot` + `sim_robot` | cumotion, servo, teleop_admittance, teleop_impedance |
| `trajectory_executor` | cumotion |
| `kinematics` (DLS) | servo |
| `pink_ik` (QP) | teleop_admittance, teleop_impedance |
| `input_handler` | servo, teleop_admittance, teleop_impedance |
| `exp_filter` | teleop_admittance, teleop_impedance |
| `ft_source` | teleop_admittance |
| `compliant_control` | teleop_admittance |
| `controller_utils` | servo (sim 모드) |

---

## 2. 로봇 통신 계층 (robot_backend / ur_robot / sim_robot)

### 왜 필요한가

실제 UR10e 로봇(ur_rtde 라이브러리)과 시뮬레이션(ROS2 토픽)은 통신 방식이 완전히 다르다.
하지만 상위 코드(cumotion, servo, teleop)는 "관절 위치 읽기 → 명령 보내기"만 하면 된다.
ABC(Abstract Base Class) + Factory 패턴으로 **코드 변경 없이 `--mode sim`/`--mode rtde` 전환**을 가능하게 한다.

### 핵심 패턴: ABC + Factory + Context Manager

**3개 파일의 관계:**

```
robot_backend.py          ur_robot.py              sim_robot.py
┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ RobotBackend │◄───│  RTDEBackend     │    │  SimBackend      │
│ (ABC)        │◄───│  (ur_rtde)       │    │  (ROS2 topics)   │
│              │    └──────────────────┘    └──────────────────┘
│ + connect()  │
│ + disconnect()│    create_backend("rtde") → RTDEBackend
│ + get_joint_positions()   create_backend("sim")  → SimBackend
│ + send_joint_command()│
│ + __enter__/__exit__  │   ← Context Manager 지원
└──────────────┘
```

### 핵심 코드: RobotBackend ABC

```python
# robot_backend.py — 5개 추상 메서드 + 3개 기본 구현
class RobotBackend(ABC):
    @abstractmethod
    def connect(self): ...
    @abstractmethod
    def disconnect(self): ...
    @abstractmethod
    def get_joint_positions(self) -> List[float]: ...
    @abstractmethod
    def get_joint_velocities(self) -> List[float]: ...
    @abstractmethod
    def send_joint_command(self, positions: List[float]): ...

    # 기본 구현 (필요시 override)
    def get_tcp_force(self) -> List[float]:
        return [0.0] * 6       # sim에서는 센서 없음 → zeros
    def emergency_stop(self): ...  # no-op
    def speed_stop(self, deceleration: float = 2.0): ...  # no-op
```

### 핵심 코드: create_backend() 팩토리

```python
# robot_backend.py:51-59
def create_backend(mode: str, **kwargs) -> RobotBackend:
    if mode == "rtde":
        from standalone.core.ur_robot import RTDEBackend
        return RTDEBackend(robot_ip=kwargs["robot_ip"])
    elif mode == "sim":
        from standalone.core.sim_robot import SimBackend
        return SimBackend(**{k: v for k, v in kwargs.items() if k != "robot_ip"})
    raise ValueError(f"Unknown mode: {mode}")
```

**lazy import** 패턴을 사용한다: `ur_rtde`가 설치되지 않은 환경에서도 sim 모드는 정상 동작한다.

### RTDEBackend vs SimBackend 차이

| 항목 | RTDEBackend | SimBackend |
|------|-------------|------------|
| 통신 | ur_rtde (TCP/IP 직접) | ROS2 토픽 (JointState) |
| 명령 | `servoJ()` (125Hz) | `publish(JointState)` |
| 센서 | `getActualTCPForce()` 지원 | zeros 반환 |
| 의존성 | `pip install ur-rtde` | `rclpy` + ROS2 환경 |
| 스레딩 | 없음 | `rclpy.spin()` 백그라운드 스레드 |

### 사용 패턴 (Context Manager)

```python
# 모든 기능 모듈에서 동일한 패턴으로 사용
with create_backend("rtde", robot_ip="192.168.0.2") as robot:
    q = robot.get_joint_positions()
    robot.send_joint_command(new_q)
# disconnect() 자동 호출
```

---

## 3. 궤적 실행 (trajectory_executor.py)

### 왜 필요한가

cuMotion GPU 플래너는 **40Hz** (25ms 간격)로 궤적 포인트를 생성하지만,
UR10e의 servoJ 명령은 **125Hz** (8ms 간격)로 전송해야 한다.
이 주파수 차이를 **선형 보간(linear interpolation)**으로 해결한다.

### 핵심 코드: resample_trajectory()

```python
# trajectory_executor.py:16-40
def resample_trajectory(positions, source_dt, target_dt):
    """cuMotion 40Hz → servoJ 125Hz 변환 (선형 보간)"""
    n_points, n_joints = positions.shape
    source_times = np.arange(n_points) * source_dt       # 원본 시간축
    total_time = source_times[-1]
    target_times = np.arange(0, total_time + target_dt * 0.5, target_dt)  # 새 시간축

    resampled = np.zeros((len(target_times), n_joints))
    for j in range(n_joints):
        resampled[:, j] = np.interp(target_times, source_times, positions[:, j])
    return resampled
```

`np.interp()`를 관절별로 적용하여 각 관절의 위치를 독립적으로 보간한다.

### 핵심 코드: execute_trajectory() — busy-wait 타이밍

```python
# trajectory_executor.py:89-117
def execute_trajectory(robot, trajectory, command_dt=SERVOJ_DT):
    resampled = resample_trajectory(trajectory["positions"], trajectory["dt"], command_dt)
    t_start = time.perf_counter()
    for i in range(len(resampled)):
        robot.send_joint_command(resampled[i].tolist())
        # Busy-wait: 정확한 타이밍을 위해 sleep 대신 perf_counter 사용
        t_target = t_start + (i + 1) * command_dt
        while time.perf_counter() < t_target:
            pass
    robot.on_trajectory_done()
```

**왜 `time.sleep()` 대신 busy-wait인가?**
- `sleep()`은 OS 스케줄러에 의해 지정 시간보다 늦게 깨어날 수 있음 (수 ms 오차)
- 125Hz 제어에서 8ms 주기 중 수 ms 오차는 궤적 추적 품질에 큰 영향
- `perf_counter()` busy-wait은 마이크로초 수준 정밀도 제공
- 단점: CPU를 100% 점유하지만, 실시간 제어에서는 허용 가능한 트레이드오프

### 안전 검증 함수

```python
validate_trajectory()    # 속도/가속도 한계 초과 검사
check_start_match()      # 로봇 현재 위치 ↔ 궤적 시작점 불일치 검사 (tolerance: 0.05 rad)
```

---

## 4. 역기구학 — DLS vs QP

standalone에는 **2가지 IK 솔버**가 있다. 복잡도와 안전성의 트레이드오프에 따라 선택한다.

### 4.1 kinematics.py — Pinocchio DLS (Damped Least Squares)

#### 왜 존재하는가
가장 빠르고 단순한 IK. servo/ 모듈처럼 관절 한계 검사가 덜 중요한 경우 적합하다.

#### 수식

```
Δq = J^T · (J·J^T + λ²·I)^{-1} · (twist · dt)
```

- `J`: 6×6 기하학적 자코비안 (Jacobian)
- `λ`: 댐핑 계수 (default: 0.05) — 특이점 근처에서 수치 안정성 보장
- `twist`: 목표 카르테시안 속도 [vx, vy, vz, wx, wy, wz]
- `dt`: 시간 스텝

#### 핵심 코드: compute_joint_delta()

```python
# kinematics.py:66-92
def compute_joint_delta(self, q, twist, dt, damping=0.05, local=False):
    """DLS IK: Cartesian twist → joint delta"""
    J = self.get_jacobian(q, local=local)
    JJt = J @ J.T
    JJt_damped = JJt + (damping ** 2) * np.eye(6)
    dq = J.T @ np.linalg.solve(JJt_damped, twist * dt)
    return dq
```

**한계**: 관절 한계(joint limits)를 고려하지 않는다. 관절이 한계에 도달해도 계속 그 방향으로 이동을 시도할 수 있다.

### 4.2 pink_ik.py — Pink QP (Quadratic Programming)

#### 왜 존재하는가
teleop_admittance, teleop_impedance처럼 **안전이 중요한** 모듈에서 사용.
관절 한계를 QP 제약 조건으로 처리하여 **물리적으로 불가능한 해를 반환하지 않는다.**

#### 수식 (개념)

```
minimize   ||Δq||² + c_posture · ||q - q_ref||²
subject to J · Δq = Δx (end-effector 추적)
           q_min ≤ q + Δq ≤ q_max (관절 한계)
```

- `c_posture`: 자세 유지 비용 (default: 1e-3) — 주 태스크에 영향 없이 관절을 기준 자세 근처로 유지
- `proxqp`: 실제 사용하는 QP 솔버 (`pip install proxsuite`)

#### 핵심 코드: PinkIK.solve()

```python
# pink_ik.py:59-92
def solve(self, target_pos, target_quat, dt):
    """QP IK: Cartesian target → joint positions"""
    # SE3 타겟 생성
    quat_pin = pin.Quaternion(target_quat[3], target_quat[0],
                              target_quat[1], target_quat[2])
    target_se3 = pin.SE3(quat_pin.matrix(), target_pos)
    self._ee_task.set_target(target_se3)

    # QP 풀기 (관절 한계 자동 제약)
    velocity = pink.solve_ik(
        self._config, self._tasks, dt,
        solver="proxqp", damping=self._damping,
    )
    self._config.integrate_inplace(velocity, dt)
    return self._config.q.copy()
```

#### sync_configuration() — 외부 이동 후 동기화

```python
# pink_ik.py:122-129
def sync_configuration(self, q):
    """로봇이 외부에서 이동한 후(e-stop 리셋 등) 내부 상태 동기화"""
    self._config = pink.Configuration(
        self._model, self._data, q.copy(), forward_kinematics=True
    )
```

Pink IK는 내부에 현재 관절 상태(`_config`)를 유지한다. 로봇이 IK 외부에서 이동하면 반드시 `sync_configuration()`을 호출해야 한다.

### 4.3 비교: 언제 어떤 것을 쓰는가

| 항목 | PinocchioIK (DLS) | PinkIK (QP) |
|------|-------------------|-------------|
| 입력 | twist (속도) | target_pos + target_quat (목표 포즈) |
| 출력 | Δq (관절 **변화량**) | q (관절 **절대 위치**) |
| 관절 한계 | 미고려 (clamp만 가능) | QP 제약으로 보장 |
| 자세 유지 | 없음 | PostureTask로 기준 자세 근처 유지 |
| 속도 | 매우 빠름 | 빠름 (QP 풀이 포함) |
| 사용 모듈 | servo/ | teleop_admittance/, teleop_impedance/ |
| 적합한 상황 | 단순 텔레옵, 프로토타이핑 | 안전 중요, 장시간 운용 |

---

## 5. 입력 처리 (input_handler.py)

### 왜 필요한가

servo/, teleop_admittance/, teleop_impedance/ 모두 사용자 입력(키보드/Xbox)을 받아야 한다.
**동일한 명령 구조(TeleopCommand)**로 추상화하면 입력 장치를 교체해도 제어 코드는 변경 불필요.

### TeleopCommand 데이터클래스

```python
# input_handler.py:17-33
@dataclass
class TeleopCommand:
    velocity: np.ndarray       # [vx, vy, vz, wx, wy, wz] — 카르테시안 속도
    estop: bool = False        # 비상 정지
    reset: bool = False        # 리셋
    quit: bool = False         # 종료
    speed_scale: float = 1.0   # 현재 속도 배율
    # Admittance 전용
    admittance_toggle: bool = False
    admittance_preset: str = ""   # "STIFF"/"MEDIUM"/"SOFT"
    ft_zero: bool = False         # F/T 센서 영점 조정
    # Impedance 전용
    impedance_preset: str = ""
    gain_scale_up: bool = False
    gain_scale_down: bool = False
```

하나의 데이터클래스에 모든 제어 모드의 명령을 담는다. 사용하지 않는 필드는 기본값(False/"")으로 무시된다.

### Keyboard vs Xbox 구현

| 항목 | KeyboardInput | XboxInput |
|------|--------------|-----------|
| 라이브러리 | `termios` (표준 라이브러리) | `pygame` |
| 입력 방식 | 비차단(non-blocking) `select()` | 이벤트 루프 `pygame.event.get()` |
| 속도 제어 | 이산적 (키 하나당 고정 속도) | 연속적 (조이스틱 아날로그 값) |
| 데드존 | 없음 | threshold=0.1 |

### 팩토리 패턴

```python
# input_handler.py:251-260
def create_input(input_type: str, ...) -> InputHandler:
    if input_type == "keyboard":
        return KeyboardInput(cartesian_step, rotation_step)
    elif input_type == "xbox":
        return XboxInput(linear_scale, angular_scale)
```

CLI 인자 `--input keyboard`/`--input xbox`가 그대로 팩토리에 전달된다.

### 키보드 키 매핑

```
이동:  w/s (X축), a/d (Y축), q/e (Z축)
회전:  u/o (X회전), i/k (Y회전), j/l (Z회전)
제어:  Space=E-Stop, r=Reset, x/ESC=Quit
속도:  +/-  (0.5x ~ 8.0x)
어드미턴스: t=토글, z=F/T영점, 1/2/3=프리셋
임피던스: [/] = 게인 스케일
```

---

## 6. 필터링 (exp_filter.py)

### 왜 필요한가

사용자 입력(키보드/조이스틱)은 노이즈가 많다. 입력이 순간적으로 끊기거나 떨리면 로봇이 급작스럽게 움직인다.
**EMA(Exponential Moving Average) 필터**로 이를 부드럽게 만든다.

### 수식

**위치 (3D)**: 선형 EMA
```
y_filtered = α · x_new + (1 - α) · y_prev
```
- `α` (alpha_pos): 높을수록 새 입력에 민감, 낮을수록 부드러움 (default: 0.7)

**자세 (quaternion)**: Slerp (Spherical Linear Interpolation)
```
q_filtered = slerp(q_prev, q_new, α)
```
- quaternion은 4차원 단위 구 위의 점 → 선형 보간(EMA)하면 정규화가 깨짐
- slerp는 구 위에서 최단 경로로 보간하므로 정규화 유지

### 핵심 코드: ExpFilter.update()

```python
# exp_filter.py:32-64
def update(self, position, quaternion):
    # 위치: 선형 EMA
    filtered_pos = (self._alpha_pos * position +
                    (1.0 - self._alpha_pos) * self._prev_pos)

    # 자세: slerp (Pinocchio Quaternion 사용)
    new_quat = pin.Quaternion(quaternion[3], quaternion[0],
                              quaternion[1], quaternion[2])
    filtered_quat = self._prev_quat.slerp(self._alpha_ori, new_quat)

    self._prev_pos = filtered_pos.copy()
    self._prev_quat = filtered_quat
    return filtered_pos, quat_out  # xyzw 형식
```

**주의**: quaternion 순서가 중요하다.
- Pinocchio: `(w, x, y, z)` — 생성자 인자 순서
- 반환값: `[x, y, z, w]` — 코드 내부 규약 (일반적인 로보틱스 컨벤션)

---

## 7. 힘/토크 센서 (ft_source.py)

### 왜 필요한가

teleop_admittance에서 F/T 센서 데이터를 읽어 어드미턴스 제어에 사용한다.
하지만 시뮬레이션에서는 F/T 센서가 없다. **Protocol 패턴**으로 이를 추상화한다.

### Protocol 패턴 (duck typing)

```python
# ft_source.py:13-22
class FTSource(Protocol):
    def get_wrench(self) -> np.ndarray: ...    # [fx,fy,fz,tx,ty,tz]
    def zero_sensor(self) -> None: ...          # 바이어스 보정
```

`Protocol`은 ABC와 달리 **상속 없이** 메서드 시그니처만 일치하면 타입 체크를 통과한다.
`RTDEFTSource`와 `NullFTSource`는 `FTSource`를 상속하지 않지만, 동일한 메서드를 구현한다.

### RTDEFTSource: 바이어스 보정

```python
# ft_source.py:25-37
class RTDEFTSource:
    def __init__(self, backend):
        self._backend = backend
        self._bias = np.zeros(6)

    def get_wrench(self) -> np.ndarray:
        raw = np.array(self._backend.get_tcp_force())
        return raw - self._bias     # 바이어스 제거

    def zero_sensor(self) -> None:
        self._bias = np.array(self._backend.get_tcp_force())  # 현재값을 바이어스로 저장
```

**바이어스 보정이 필요한 이유**: UR10e F/T 센서는 중력의 영향으로 무부하 상태에서도 0이 아닌 값을 읽는다.
`zero_sensor()`를 호출하면 현재 읽힌 값을 "기준(바이어스)"으로 저장하고, 이후 읽기에서 이를 빼준다.

### NullFTSource: 센서 없는 환경

```python
class NullFTSource:
    def get_wrench(self) -> np.ndarray:
        return np.zeros(6)       # 항상 zeros
    def zero_sensor(self) -> None:
        pass                      # no-op
```

sim 모드나 F/T 센서가 없는 환경에서 사용. 어드미턴스 제어가 비활성 상태가 된다 (외력 = 0이므로).

---

## 8. 컴플라이언스 제어 (compliant_control.py)

### 왜 필요한가

teleop_admittance에서 외력에 반응하여 로봇이 유연하게 움직이게 하는 **어드미턴스 동역학**을 구현한다.
이를 core/에 둔 이유: 향후 다른 기능 모듈에서도 재사용 가능하도록 설계.

### 어드미턴스 동역학

```
M · ẍ + D · ẋ + K · x = F_ext
```

- `M`: 가상 질량 (kg) — 클수록 관성이 커서 반응이 느림
- `D`: 감쇠 (N·s/m) — 클수록 진동 억제, 움직임 저항
- `K`: 강성 (N/m) — 클수록 원래 위치로 빨리 복귀
- `F_ext`: 외력 (F/T 센서에서 읽은 값)
- `x`: 변위 (외력에 의한 카르테시안 이동량)

### 프리셋 설계 근거

```python
# compliant_control.py:22-38
COMPLIANCE_PRESETS = {
    "STIFF":  ComplianceParams(M=[10,10,10,1,1,1], D=[200,...], K=[500,...]),
    "MEDIUM": ComplianceParams(M=[5,5,5,0.5,...],   D=[100,...], K=[200,...]),
    "SOFT":   ComplianceParams(M=[2,2,2,0.2,...],   D=[40,...],  K=[50,...]),
}
```

| 프리셋 | 특성 | 용도 |
|--------|------|------|
| STIFF | 높은 강성/감쇠, 큰 질량 → 작은 변위 | 정밀 작업 (큰 힘에도 약간만 밀림) |
| MEDIUM | 균형 잡힌 파라미터 | 일반 용도 (기본값) |
| SOFT | 낮은 강성/감쇠, 작은 질량 → 큰 변위 | 사람과 협업 (쉽게 밀림) |

회전(tx, ty, tz)의 질량/감쇠/강성은 병진(fx, fy, fz)의 1/10로 설정.
이는 회전 변위가 작은 각도(rad)이고, 동일 비율로 설정하면 회전이 과하게 반응하기 때문.

### 핵심 코드: AdmittanceController.update()

```python
# compliant_control.py:105-148
def update(self, f_ext, dt):
    # 포화 검사: 힘/토크가 한계 초과 시 리셋 (안전)
    if force_mag > self._force_saturation or torque_mag > self._torque_saturation:
        self.reset()
        return self._x.copy()

    # 데드존 적용: 소음 수준의 작은 힘은 무시
    f = f_ext.copy()
    mask = np.abs(f) < self._force_deadzone
    f[mask] = 0.0

    # 2차 동역학 계산
    xddot = (f - p.D * self._xdot - p.K * self._x) / p.M

    # 오일러 적분
    self._xdot += xddot * dt    # 속도 업데이트
    self._x += self._xdot * dt  # 변위 업데이트

    # 변위 제한 (안전)
    # 병진: max 50mm, 회전: max 0.15 rad (~8.6도)
    return self._x.copy()
```

**데드존 (deadzone)**: 3N / 0.3Nm 이하의 힘/토크는 센서 노이즈로 간주하여 무시.
이 없으면 로봇이 정지 상태에서도 미세하게 떨린다.

---

## 9. 컨트롤러 전환 (controller_utils.py)

### 왜 필요한가

ROS2 기반 UR 드라이버는 여러 컨트롤러를 제공한다:
- `joint_trajectory_controller`: MoveIt 등에서 사용 (궤적 추적)
- `forward_position_controller`: servoJ 같은 직접 위치 명령
- `scaled_joint_trajectory_controller`: 속도 스케일링 지원

servo/ 모듈(sim 모드)에서 `forward_position_controller`를 활성화하려면 기존 컨트롤러를 비활성화해야 한다.

### 핵심 코드: ControllerSwitcher

```python
# controller_utils.py:64-101
class ControllerSwitcher:
    def switch_controller(self, start, stop, strictness=2):
        """start 컨트롤러 활성화, stop 컨트롤러 비활성화"""
        req = SwitchController.Request()
        req.activate_controllers = start
        req.deactivate_controllers = stop
        req.strictness = strictness  # 2=STRICT
        future = self._switch_client.call_async(req)
        ...

    def activate_forward_position(self):
        """trajectory → forward_position 전환 (원래 상태 저장)"""
        ...

    def restore_original(self):
        """원래 컨트롤러 복원"""
        ...
```

`activate_forward_position()`은 현재 활성 궤적 컨트롤러를 `_original_active`에 저장하고,
`restore_original()`로 복원할 수 있다. 이로써 servo 종료 후 원래 상태로 자동 복구된다.

> **참고**: RTDE 모드(ur_rtde 직접 통신)에서는 `controller_utils`를 사용하지 않는다.
> ur_rtde가 자체적으로 로봇 컨트롤러를 관리하기 때문이다.

---

## 다음 단계

core/ 모듈을 이해했다면, 기능 모듈을 난이도 순서대로 학습하자:

1. **[servo/](../servo/docs/manual.md)** (난이도 ★) — core/kinematics + input_handler 활용
2. **[cumotion/](../../cumotion/docs/manual.md)** (난이도 ★★) — core/trajectory_executor 활용
3. **[teleop_admittance/](../../teleop_admittance/docs/manual.md)** (난이도 ★★★) — core/pink_ik + exp_filter + ft_source + compliant_control 활용
4. **[teleop_impedance/](../../teleop_impedance/docs/manual.md)** (난이도 ★★★★) — core/pink_ik + exp_filter + 자체 URScript 토크 제어

> 실행 방법, CLI 옵션, 트러블슈팅은 각 모듈의 `docs/user_guide.md`를 참조하세요.
