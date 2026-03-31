# Teleop Admittance 기술 문서

어드미턴스 텔레옵 모듈의 핵심 이론, 구현 기술, 그리고 로테이션/좌표축 처리 방식을 정리한 문서입니다.

---

## 목차

1. [어드미턴스 제어 이론](#1-어드미턴스-제어-이론)
2. [구현 기술 스택](#2-구현-기술-스택)
3. [로테이션 처리](#3-로테이션-처리)
4. [전체 파이프라인](#4-전체-파이프라인)

---

## 1. 어드미턴스 제어 이론

### 1.1 기본 모델

2차 선형 어드미턴스 모델 (가상 질량-댐퍼-스프링):

```
M · ẍ + D · ẋ + K · x = f_ext
```

| 기호 | 의미 | 단위 |
|------|------|------|
| **M** | 가상 질량 (Virtual Mass) | kg (병진), kg·m² (회전) |
| **D** | 댐핑 계수 (Damping) | N·s/m, N·m·s/rad |
| **K** | 강성 계수 (Stiffness) | N/m, N·m/rad |
| **x** | 변위 (6-DOF) | [dx, dy, dz, drx, dry, drz] |
| **f_ext** | 외력 (External Wrench) | [fx, fy, fz, tx, ty, tz] N, Nm |

**직관적 이해**:
- **M이 크면**: 관성이 커서 반응이 느리지만 안정적 (진동 감소)
- **D가 크면**: 움직임이 빨리 감쇠됨 (오버댐핑)
- **K가 크면**: 원래 위치로 돌아가는 복원력이 강함 (STIFF)
- **K = 0이면**: 복원력 없음 → 외력에 의해 자유롭게 밀림 (FREE)

### 1.2 이산화 (오일러 적분)

연속 시간 모델을 디지털 제어 루프에서 사용하기 위해 Forward Euler 방법으로 이산화:

```python
# 가속도 계산: M·ẍ = f_ext - D·ẋ - K·x  →  ẍ = (f - D·ẋ - K·x) / M
xddot = (f - D * xdot - K * x) / M

# 속도 적분: ẋ(t+dt) = ẋ(t) + ẍ·dt
xdot += xddot * dt

# 변위 적분: x(t+dt) = x(t) + ẋ·dt
x += xdot * dt
```

**소스 코드** (`robot/core/compliant_control.py:133-139`):

```python
# AdmittanceController.update()
p = self._params
xddot = (f - p.D * self._xdot - p.K * self._x) / p.M

# Euler integration
self._xdot += xddot * dt
self._x += self._xdot * dt
```

### 1.3 컴플라이언스 프리셋

4개 프리셋은 M/D/K 값 조합으로 서로 다른 순응 특성을 제공합니다.

**소스 코드** (`robot/core/compliant_control.py:22-43`):

| 프리셋 | M (병진/회전) | D (병진/회전) | K (병진/회전) | 특성 |
|--------|--------------|--------------|--------------|------|
| **STIFF** | 10 / 1 | 200 / 20 | 500 / 50 | 높은 강성, 낮은 순응 → 정밀 위치 유지 |
| **MEDIUM** | 5 / 0.5 | 100 / 10 | 200 / 20 | 중간 → 일반 작업 (기본값) |
| **SOFT** | 2 / 0.2 | 40 / 4 | 50 / 5 | 낮은 강성 → 섬세한 접촉 작업 |
| **FREE** | 2 / 0.2 | 30 / 3 | **0 / 0** | 강성 없음 → 핸드 가이딩 |

> **FREE 프리셋의 핵심**: K=0이므로 복원력이 없습니다. 댐핑(D)만 존재하여 외력을 제거하면 자연스럽게 정지하지만, 원래 위치로 돌아가지 않습니다.

### 1.4 안전 장치

어드미턴스 컨트롤러 내부에 3단계 보호가 구현되어 있습니다.

**1) 데드존 (Deadzone)** — 센서 노이즈 제거

```python
# compliant_control.py:128-131
f = f_ext.copy()
mask = np.abs(f) < self._force_deadzone     # 기본값: [3, 3, 3, 0.3, 0.3, 0.3] N/Nm
f[mask] = 0.0
f[~mask] -= np.sign(f[~mask]) * self._force_deadzone[~mask]  # 데드존 오프셋 차감
```

데드존 값 이하의 힘/토크는 무시됩니다. 데드존을 초과하는 값은 데드존 크기만큼 빼서 0부터 시작하는 연속적인 반응을 만듭니다.

**2) 포화 리셋 (Saturation Reset)** — 충돌/이상 감지

```python
# compliant_control.py:121-125
force_mag = np.linalg.norm(f_ext[:3])
torque_mag = np.linalg.norm(f_ext[3:])
if force_mag > self._force_saturation or torque_mag > self._torque_saturation:
    self.reset()  # 변위, 속도 모두 0으로 초기화
    return self._x.copy()
```

- 힘 > 100N 또는 토크 > 10Nm → 충돌로 판단하고 전체 상태 리셋

**3) 변위 클램핑 (Displacement Clamping)**

```python
# compliant_control.py:142-151
# 병진 변위 제한: 0.15m
disp_norm = np.linalg.norm(self._x[:3])
if disp_norm > self._max_disp_trans:
    self._x[:3] *= self._max_disp_trans / disp_norm
    self._xdot[:3] *= 0.5  # 속도도 감쇠

# 회전 변위 제한: 0.3 rad (~17°)
rot_norm = np.linalg.norm(self._x[3:])
if rot_norm > self._max_disp_rot:
    self._x[3:] *= self._max_disp_rot / rot_norm
    self._xdot[3:] *= 0.5
```

- 변위가 한계를 초과하면 정규화하고 속도를 절반으로 줄여서 부드럽게 제한

---

## 2. 구현 기술 스택

### 2.1 라이브러리 구성

| 라이브러리 | 패키지명 | 역할 |
|-----------|---------|------|
| **Pinocchio** | `pinocchio` (시스템) | URDF 파싱, FK, 쿼터니언/회전행렬 연산 |
| **Pink** | `pip install pin-pink` | QP 기반 역운동학 (FrameTask + PostureTask) |
| **ProxQP** | `pip install proxsuite` | QP 솔버 (Pink 내부에서 사용) |
| **ur_rtde** | `pip install ur-rtde` | UR10e 실시간 통신 (servoJ, F/T 읽기) |
| **NumPy** | `pip install "numpy<2"` | 선형대수 (pinocchio ABI 호환 필수) |

> **주의**: `pip install pink`는 코드 포매터입니다! 반드시 **`pin-pink`** 를 설치해야 합니다.

### 2.2 Pink IK (QP 기반 역운동학)

Pink은 Pinocchio 위에 구축된 QP(Quadratic Programming) 기반 IK 솔버입니다.

**핵심 개념**:
- **FrameTask**: EE(tool0) 프레임을 목표 SE3 pose로 추적
- **PostureTask**: 관절 기본 자세 유지 (joint centering)
- QP 문제로 formulate하여 관절 한계를 constraint로 처리

**소스 코드** (`robot/core/pink_ik.py:35-41`):

```python
# 태스크 설정
self._ee_task = pink.FrameTask(
    ee_frame,                          # "tool0"
    position_cost=position_cost,       # 1.0 (위치 우선)
    orientation_cost=orientation_cost,  # 0.5 (방향 차선)
)
self._posture_task = pink.PostureTask(cost=posture_cost)  # 1e-3 (최저 우선순위)
```

**IK 풀이** (`robot/core/pink_ik.py:75-90`):

```python
# 1. 목표 SE3 생성 (base_link 프레임 기준)
quat_pin = pin.Quaternion(target_quat[3], target_quat[0],
                          target_quat[1], target_quat[2])  # xyzw → wxyz 변환
target_se3 = pin.SE3(quat_pin.matrix(), target_pos)
self._ee_task.set_target(target_se3)

# 2. QP 풀이 → 관절 속도 반환
velocity = pink.solve_ik(
    self._config, self._tasks, dt,
    solver="proxqp",       # ProxQP 솔버 사용
    damping=self._damping,  # 1e-12 (특이점 방지)
)

# 3. 관절 속도를 적분하여 관절 위치 업데이트
self._config.integrate_inplace(velocity, dt)
return self._config.q.copy()
```

### 2.3 Exponential Filter (스무딩)

입력 명령의 급격한 변화를 완화하여 부드러운 로봇 움직임을 만듭니다.

**소스 코드** (`robot/core/exp_filter.py`):

```python
# 위치: 선형 EMA (Exponential Moving Average)
filtered_pos = alpha * target_pos + (1 - alpha) * prev_pos

# 오리엔테이션: 쿼터니언 Slerp (Spherical Linear Interpolation)
filtered_quat = prev_quat.slerp(alpha, new_quat)
```

- **alpha = 0.85** (기본값): 새 입력을 85% 반영 → 빠른 응답, 약간의 스무딩
- alpha가 낮을수록 더 부드럽지만 지연이 커짐
- **Slerp 사용 이유**: 오일러 각도 보간은 gimbal lock 문제가 있지만, slerp은 쿼터니언 구면 위를 일정 속도로 보간하여 이 문제를 회피

### 2.4 servoJ (실시간 관절 명령)

ur_rtde의 `servoJ()`는 125Hz로 관절 위치를 스트리밍하는 실시간 명령입니다.

```
제어 루프 1사이클 (8ms):
  1. 현재 관절 읽기 (get_joint_positions)
  2. IK 풀기 (q_target)
  3. 안전 검사 (check_and_apply)
  4. servoJ 전송 (send_joint_command)
```

- 궤적 기반이 아닌 **position streaming** 방식
- 매 사이클마다 다음 관절 위치를 전송
- 125Hz 미만이면 UR 컨트롤러가 protective stop 발생 가능

---

## 3. 로테이션 처리

> **이 섹션이 가장 중요합니다.** 병진(translation)과 오리엔테이션(orientation)의 좌표축이 맞지 않는 문제로 개발 중 여러 차례 이슈가 발생했습니다.

### 3.1 좌표계 정의

```
UR10e base_link 프레임:
  X = 오른쪽 (로봇 기준)
  Y = 앞쪽 (로봇에서 멀어지는 방향)
  Z = 위쪽

tool0 (EE) 프레임:
  EE-down 마운트 시 TCP의 X, Y가 base_link의 X, Y와 반전
  (EE 커넥터가 아래를 향하므로 180° 회전)
```

### 3.2 IK 서보잉 시 로테이션

키보드/조이스틱 입력 → 목표 쿼터니언 갱신 → Pink IK → servoJ

#### 입력: base_link 축 기준 각속도

**소스 코드** (`robot/core/input_handler.py:67-81`):

```python
# KEY_MAP: 키 → (vx, vy, vz, wx, wy, wz) base_link 프레임 기준 단위 방향
KEY_MAP = {
    # 병진
    "w": (0, 1, 0, 0, 0, 0),    # +Y (앞)
    "s": (0, -1, 0, 0, 0, 0),   # -Y (뒤)
    "a": (-1, 0, 0, 0, 0, 0),   # -X (왼쪽)
    "d": (1, 0, 0, 0, 0, 0),    # +X (오른쪽)
    "q": (0, 0, 1, 0, 0, 0),    # +Z (위)
    "e": (0, 0, -1, 0, 0, 0),   # -Z (아래)
    # 회전 (base_link 축 기준)
    "u": (0, 0, 0, 1, 0, 0),    # +Roll  (X축 회전)
    "o": (0, 0, 0, -1, 0, 0),   # -Roll
    "i": (0, 0, 0, 0, 1, 0),    # +Pitch (Y축 회전)
    "k": (0, 0, 0, 0, -1, 0),   # -Pitch
    "j": (0, 0, 0, 0, 0, 1),    # +Yaw   (Z축 회전)
    "l": (0, 0, 0, 0, 0, -1),   # -Yaw
}
```

입력은 `velocity[3:6]` = [ωx, ωy, ωz]로 **base_link** 축 기준 각속도입니다.

#### 핵심: apply_rotation_delta()

각속도 입력을 현재 쿼터니언에 적용하는 함수입니다.

**소스 코드** (`robot/arm/admittance/main.py:69-85`):

```python
def apply_rotation_delta(quat_xyzw, angular_vel, dt):
    angle = np.linalg.norm(angular_vel) * dt
    if angle < 1e-10:
        return quat_xyzw.copy()

    # 1. 각속도 → 회전축 + 각도 (Angle-Axis)
    axis = angular_vel / (np.linalg.norm(angular_vel) + 1e-15)
    aa = pin.AngleAxis(angle, axis)
    dR = aa.matrix()  # 3x3 증분 회전행렬

    # 2. 현재 쿼터니언 → 회전행렬
    q_pin = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])

    # 3. ★ Post-multiply: R_new = R_current @ dR
    R_new = q_pin.matrix() @ dR

    # 4. 결과를 쿼터니언으로 변환
    q_new = pin.Quaternion(R_new)
    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])
```

**Post-multiply (`R_current @ dR`)의 의미**:

```
Pre-multiply  (dR @ R_current):  월드(base_link) 좌표 기준 회전
Post-multiply (R_current @ dR):  바디(tool0) 좌표 기준 회전
```

여기서 **post-multiply**를 사용하므로, 입력 각속도 [ωx, ωy, ωz]는 형식적으로는 base_link 축 방향이지만 **현재 EE 프레임을 기준으로 적용**됩니다. 즉:
- `U/O` (Roll = X축 회전): EE의 현재 자세 기준으로 Roll 회전
- `I/K` (Pitch = Y축 회전): EE의 현재 자세 기준으로 Pitch 회전
- `J/L` (Yaw = Z축 회전): EE의 현재 자세 기준으로 Yaw 회전

이렇게 하면 EE 자세가 어떻든 직관적인 조작이 가능합니다.

#### IK 타겟 생성

```python
# main.py:369-370 — 목표 pose 누적
target_pos = target_pos + cmd.velocity[:3]          # base_link 기준 병진
target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)  # 회전

# main.py:395-396 — Pink IK 풀기
self.ik.sync_configuration(self.q_current)
q_target = self.ik.solve(compliant_pos, compliant_quat, dt)
```

Pink IK 내부에서는 `pin.SE3(rotation_matrix, position)` 형태의 base_link 기준 목표를 tool0 프레임이 추적하도록 QP를 풀어 관절 위치를 산출합니다.

#### 쿼터니언 컨벤션 주의

```
파이프라인 내부: [x, y, z, w]  (xyzw)
Pinocchio API:  [w, x, y, z]  (wxyz)
```

변환은 Pinocchio API 호출 시점에서 수행:

```python
# xyzw → wxyz (Pinocchio에 전달할 때)
quat_pin = pin.Quaternion(quat[3], quat[0], quat[1], quat[2])

# wxyz → xyzw (Pinocchio에서 받을 때)
quat_xyzw = np.array([q.x, q.y, q.z, q.w])
```

### 3.3 F/T 센서 적용 시 로테이션 (어드미턴스)

외력(F/T) → 프레임 변환 → 어드미턴스 동역학 → 변위 → 목표 pose에 합산

#### 문제: TCP 프레임 ≠ base_link 프레임

UR10e의 `getActualTCPForce()`는 **TCP(tool) 프레임** 기준 wrench를 반환합니다. 하지만 어드미턴스 컨트롤러는 **base_link** 프레임 기준으로 동작합니다.

```
EE-down 마운트 (EE 커넥터가 아래를 향함):

    base_link          TCP (tool0)
    +X (오른쪽)    ↔    -X (반전)
    +Y (앞쪽)      ↔    -Y (반전)
    +Z (위)        ↔    +Z (동일)
```

EE가 180° 뒤집혀 있으므로 X, Y 축이 반전됩니다.

#### 해결: X, Y 부호 반전

**소스 코드** (`robot/core/ft_source.py:84-87`):

```python
class BaseFrameFTSource:
    def get_wrench(self) -> np.ndarray:
        w = self._source.get_wrench()  # TCP 프레임 기준 wrench
        # X, Y 부호 반전 → base_link 프레임으로 변환
        return np.array([-w[0], -w[1], w[2], -w[3], -w[4], w[5]])
```

| 성분 | TCP 프레임 | base_link 변환 | 이유 |
|------|-----------|---------------|------|
| fx, fy | 원래값 | **부호 반전** | EE-down으로 X, Y 축 180° 회전 |
| fz | 원래값 | 그대로 | Z축은 동일 방향 |
| tx, ty | 원래값 | **부호 반전** | 회전 축도 동일하게 반전 |
| tz | 원래값 | 그대로 | Z축 토크는 동일 |

> **참고**: `ft_source.py`에는 6가지 대안 변환이 주석으로 기록되어 있습니다 (#1~#6). 현재 UR10e EE-down 구성에서는 #4 (negate X,Y)가 실험적으로 검증되었습니다. 로봇/마운트/IK 변경 시 `test_wrench_frame.py --servo`로 실시간 검증할 수 있습니다.

#### 어드미턴스 변위의 회전 적용

어드미턴스 컨트롤러는 base_link 기준 6-DOF 변위를 반환합니다:

```python
adm_disp = [dx, dy, dz, drx, dry, drz]  # base_link 프레임
```

이 변위를 목표 pose에 합산하는 과정:

**소스 코드** (`robot/arm/admittance/main.py:387-392`):

```python
# 어드미턴스 변위 계산 (base_link 기준)
adm_disp = self.admittance.compute_displacement(self.q_current, dt)

# 병진: 단순 덧셈 (base_link 기준)
compliant_pos = clamped_pos + adm_disp[:3]

# 회전: apply_rotation_delta 사용 (IK와 동일한 방식)
compliant_quat = apply_rotation_delta(filt_quat, adm_disp[3:], 1.0)

# 안전: 어드미턴스 오프셋 후 다시 workspace 클램핑
compliant_pos = self.safety.clamp_workspace(compliant_pos)
```

**핵심 포인트**: 회전 변위 `adm_disp[3:]`도 `apply_rotation_delta()`를 통해 적용되므로, IK 입력과 동일한 **post-multiply** 방식으로 처리됩니다.

#### 드리프트 방지

어드미턴스 변위가 누적 목표(target)에 역류하지 않도록 처리:

```python
# main.py:422 — 목표 쿼터니언을 어드미턴스 적용 전 값으로 동기화
target_quat = filt_quat.copy()
```

이렇게 하면 어드미턴스 오프셋은 **매 사이클 일시적으로만** 적용되고, 다음 사이클에서 새로 계산됩니다. 외력이 사라지면 어드미턴스 변위는 K (강성)에 의해 자연적으로 0으로 수렴합니다.

### 3.4 정리: 두 경로의 비교

| | IK 서보잉 (키보드) | 어드미턴스 (F/T) |
|--|-------------------|-----------------|
| **입력 소스** | 키보드/조이스틱 | F/T 센서 (UR RTDE) |
| **입력 프레임** | base_link (KEY_MAP) | TCP → base_link 변환 (negate X,Y) |
| **회전 표현** | 각속도 [ωx, ωy, ωz] | 변위 [drx, dry, drz] |
| **회전 적용** | `apply_rotation_delta()` post-multiply | 동일: `apply_rotation_delta()` |
| **결과 프레임** | base_link 쿼터니언 | base_link 쿼터니언 |
| **IK 입력** | `pin.SE3(R, p)` base_link 기준 | 동일 (합산 후) |
| **누적** | target에 지속 누적 | 매 사이클 일시 적용 (드리프트 방지) |

---

## 4. 전체 파이프라인

### 4.1 흐름도

```
┌─────────────────────────────────────────────────────────────────┐
│                     제어 루프 (50Hz sim / 125Hz rtde)             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Step 1. Input                                                  │
│  ┌──────────────┐                                               │
│  │ KeyboardInput │ → cmd.velocity[0:3] (병진, base_link)         │
│  │ / XboxInput   │ → cmd.velocity[3:6] (회전, base_link 축)      │
│  └──────────────┘                                               │
│         │                                                       │
│  Step 2. Target 누적                                             │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ target_pos += velocity[:3]                               │   │
│  │ target_quat = apply_rotation_delta(quat, vel[3:], 1.0)  │   │
│  └──────────────────────────────────────────────────────────┘   │
│         │                                                       │
│  Step 3. ExpFilter                                              │
│  ┌─────────────────────────────────────────┐                    │
│  │ filt_pos  = EMA(target_pos, alpha=0.85) │                    │
│  │ filt_quat = slerp(prev, target, 0.85)   │                    │
│  └─────────────────────────────────────────┘                    │
│         │                                                       │
│  Step 4. Workspace Clamp (Safety Level 3)                       │
│  ┌─────────────────────────────────────────┐                    │
│  │ clamped_pos = clamp(filt_pos, bounds)   │  위치만 클램핑     │
│  └─────────────────────────────────────────┘                    │
│         │                                                       │
│  Step 4.5. Admittance (F/T 센서)                                │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ TCP wrench → negate X,Y → base_link wrench             │    │
│  │ → deadzone → saturation check                          │    │
│  │ → M·ẍ + D·ẋ + K·x = f_ext (Euler integration)        │    │
│  │ → displacement clamp                                    │    │
│  │                                                         │    │
│  │ compliant_pos  = clamped_pos + disp[:3]                │    │
│  │ compliant_quat = apply_rotation_delta(quat, disp[3:])  │    │
│  └─────────────────────────────────────────────────────────┘    │
│         │                                                       │
│  Step 5. Pink IK                                                │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ target_se3 = SE3(compliant_quat → R, compliant_pos)    │    │
│  │ → FrameTask(tool0).set_target(target_se3)              │    │
│  │ → QP solve (proxqp) → joint velocity                   │    │
│  │ → integrate → q_target                                  │    │
│  └─────────────────────────────────────────────────────────┘    │
│         │                                                       │
│  Step 6. Safety Check (Levels 1, 2, 4)                          │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ L4 E-Stop: emergency_stop()                            │    │
│  │ L1 Timeout: 입력 없음 > 200ms → hold position          │    │
│  │ L2 Velocity: 관절 속도 > 0.5 rad/s → 스케일 다운        │    │
│  └─────────────────────────────────────────────────────────┘    │
│         │                                                       │
│  Step 7. Send                                                   │
│  ┌─────────────────────────────┐                                │
│  │ backend.send_joint_command  │  → servoJ (125Hz RTDE)         │
│  │ (q_safe.tolist())          │  → topic (50Hz sim)             │
│  └─────────────────────────────┘                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 단계별 프레임/표현 정리

| 단계 | 입력 | 출력 | 프레임 | 표현 |
|------|------|------|--------|------|
| Input | 키/스틱 | velocity[6] | base_link | [vx,vy,vz,ωx,ωy,ωz] |
| Target 누적 | velocity | pos[3] + quat[4] | base_link | position + xyzw quat |
| ExpFilter | pos + quat | filt_pos + filt_quat | base_link | EMA + slerp |
| Workspace Clamp | filt_pos | clamped_pos | base_link | position[3] |
| F/T Source | TCP wrench | base_link wrench | TCP → base_link | negate X,Y |
| Admittance | wrench[6] | displacement[6] | base_link | [dx,dy,dz,drx,dry,drz] |
| Pink IK | pos + quat | q_target[6] | base_link → joint space | SE3 → joint angles |
| Safety | q_target | q_safe | joint space | joint positions |
| servoJ | q_safe | — | joint space | 6-DOF joint command |

---

## 참조 소스 파일

| 파일 | 역할 |
|------|------|
| `robot/core/compliant_control.py` | AdmittanceController, ComplianceParams, 프리셋 |
| `robot/core/ft_source.py` | RTDEFTSource, BaseFrameFTSource (프레임 변환) |
| `robot/core/pink_ik.py` | PinkIK (QP IK, SE3 타겟, proxqp) |
| `robot/core/exp_filter.py` | ExpFilter (EMA + slerp) |
| `robot/core/input_handler.py` | KEY_MAP, KeyboardInput, XboxInput |
| `robot/arm/admittance/main.py` | apply_rotation_delta(), 제어 루프 |
| `robot/arm/admittance/admittance_layer.py` | AdmittanceLayer (F/T→변위 통합) |
| `robot/arm/admittance/safety_monitor.py` | SafetyMonitor (4단계 안전) |
| `robot/arm/admittance/teleop_config.py` | TeleopConfig (YAML 로더) |
