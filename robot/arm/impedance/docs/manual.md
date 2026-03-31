# teleop_impedance/ 학습 매뉴얼 — 임피던스 텔레옵 (난이도 ★★★★)

> **위치**: `robot/arm/impedance/`
> **의존 core 모듈**: `pink_ik`, `exp_filter`, `input_handler`
> **학습 선수 지식**: [core/ 매뉴얼](../../core/docs/manual.md) §4.2 (Pink IK), §6 (필터), [teleop_admittance/ 매뉴얼](../../teleop_admittance/docs/manual.md) (어드미턴스와 비교 필수)

---

## 1. 왜 teleop_impedance/가 존재하는가

teleop_admittance/는 F/T 센서로 외력을 감지하여 **위치를 보정**한다 (F→x).
하지만 이 접근에는 근본적 한계가 있다:

| teleop_admittance 한계 | teleop_impedance 해결 |
|-----------------------|----------------------|
| F/T 센서 필수 (하드웨어 의존) | 센서 불필요 — 위치 오차가 곧 힘 |
| 센서 지연 + 계산 지연 = 느린 반응 | 500Hz URScript → 빠른 물리적 반응 |
| 소프트웨어 시뮬레이션된 컴플라이언스 | 진정한 물리적 스프링-댐퍼 |
| 접촉 시 딱딱한 느낌 (위치 제어 기반) | 부드러운 접촉 (토크 제어 기반) |

### 임피던스 제어란?

**임피던스 제어**는 위치 오차를 토크로 변환하는 방식이다:

```
τ = Kp · (q_desired - q_actual) - Kd · q̇_actual + C(q, q̇)
```

- `Kp · (q_d - q)`: 가상 스프링 — 목표에서 벗어나면 되돌리는 힘
- `-Kd · q̇`: 가상 댐퍼 — 진동 방지
- `C(q, q̇)`: 코리올리스 보상 — 동역학 왜곡 제거

인과관계: **위치 오차 → 토크** (Position Error → Torque).
로봇이 스프링으로 목표에 연결된 것처럼 동작한다.

---

## 2. 듀얼 루프 아키텍처

이 모듈의 핵심 설계는 **두 개의 제어 루프**가 서로 다른 주파수로 동작한다는 점이다.

```
┌─────────────────────────────────────────────────────────────┐
│ Python 루프 (125Hz)                                         │
│                                                              │
│ Input → ExpFilter → Workspace Clamp → Pink IK → q_desired  │
│                                                    │         │
│                              RTDE input registers ◄┘         │
│                              [0..5]: q_desired               │
│                              [6..11]: Kp                     │
│                              [12..17]: Kd                    │
│                              [18]: mode                      │
│                              [19]: enable_coriolis           │
└───────────────────────────┬─────────────────────────────────┘
                            │ RTDE (네트워크)
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ URScript 루프 (500Hz)                  UR 컨트롤러 내부 실행   │
│                                                                │
│ read registers → PD 토크 계산 → 토크 포화 → direct_torque()   │
│                                                                │
│ Output registers:                                              │
│   [0..5]: applied_torque (모니터링용)                          │
│   [6]: heartbeat (생존 확인)                                   │
└───────────────────────────────────────────────────────────────┘
```

### 왜 듀얼 루프인가?

| 요소 | Python (125Hz) | URScript (500Hz) |
|------|----------------|------------------|
| 역할 | 경로 계획 (IK, 필터) | 토크 제어 (PD 법칙) |
| 복잡도 | 높음 (Pinocchio IK, QP) | 낮음 (곱셈 + 덧셈) |
| 지연 허용 | ±8ms 여유 | <2ms 필수 |
| 위치 | PC (Docker 컨테이너) | UR 컨트롤러 (실시간 OS) |

토크 제어는 매우 높은 주파수가 필요하다 (진동, 불안정 방지).
하지만 IK와 필터링은 계산량이 많아 500Hz에서 실행할 수 없다.
따라서 "무엇을 할지"는 Python이 125Hz로 결정하고, "어떻게 토크를 낼지"는 URScript가 500Hz로 실행한다.

---

## 3. RTDE 레지스터 맵 상세

Python과 URScript 사이의 통신은 **RTDE(Real-Time Data Exchange)** 레지스터를 통해 이루어진다.

### Input 레지스터 (Python → URScript)

```python
# urscript_manager.py:37-41
_REG_Q_DESIRED = 0       # input registers 0..5   — 목표 관절 각도 [rad]
_REG_KP = 6              # input registers 6..11  — 위치 강성 [Nm/rad]
_REG_KD = 12             # input registers 12..17 — 속도 감쇠 [Nm·s/rad]
_REG_MODE = 18           # input register 18      — 모드 (0=대기, 1=활성, -1=정지)
_REG_CORIOLIS = 19       # input register 19      — 코리올리스 보상 (0=끔, 1=켬)
```

### Output 레지스터 (URScript → Python)

```python
_REG_TORQUE_OUT = 0      # output registers 0..5  — 적용된 토크 [Nm]
_REG_HEARTBEAT = 6       # output register 6      — 생존 카운터 (매 사이클 +1)
```

### 통신 흐름 예시

```
Python: set_desired_position([1.0, -0.5, ...])  → 레지스터 0~5에 쓰기
Python: set_gains(Kp=[400,...], Kd=[20,...])     → 레지스터 6~17에 쓰기
Python: set_mode(1.0)                            → 레지스터 18에 1.0 쓰기
  ↓ RTDE
URScript: q_d = read_input_float_register(0..5)  → 목표 읽기
URScript: kp = read_input_float_register(6..11)  → 게인 읽기
URScript: tau = kp*(q_d-q) - kd*qd               → PD 계산
URScript: direct_torque(tau)                      → 토크 적용
URScript: write_output_float_register(0..5, tau)  → 적용 토크 기록
  ↓ RTDE
Python: get_applied_torques()                     → 모니터링용 읽기
```

---

## 4. 핵심 모듈 분석

### 4.1 main.py — ImpedanceTeleopController

#### RTDE vs Sim 모드 분기

```python
# main.py:241-244
def run(self):
    if cfg.robot.mode == "rtde":
        self._run_impedance(cfg, dt)   # URScript PD 토크 제어
    else:
        self._run_sim_fallback(cfg, dt)  # 위치 제어 폴백
```

**sim 모드에서 토크 제어가 불가능한 이유**: ROS2 mock hardware는 `JointTrajectoryController`만 지원하고, `direct_torque()`에 해당하는 인터페이스가 없다. 따라서 sim 모드에서는 teleop_admittance와 동일한 위치 제어로 폴백한다.

#### _run_impedance() 초기화 흐름

```python
# main.py:251-297 (요약)
def _run_impedance(self, cfg, dt):
    mgr = URScriptManager(cfg.robot.ip)
    mgr.connect()                              # 1. RTDE 3종 인터페이스 연결

    self.q_current = np.array(mgr.get_joint_positions())
    self.ik.initialize(self.q_current)          # 2. Pink IK 초기화

    mgr.set_desired_position(self.q_current.tolist())  # 3. 초기 목표 = 현재 위치
    mgr.set_gains(self.impedance.Kp.tolist(),          # 4. PD 게인 설정
                  self.impedance.Kd.tolist())
    mgr.set_coriolis_enabled(True)              # 5. 코리올리스 보상 활성화

    mgr.upload_and_start()                      # 6. URScript 업로드 + 실행
    time.sleep(0.5)                             # 7. URScript 초기화 대기
    mgr.set_mode(1.0)                           # 8. PD 활성화 (mode=1)
```

**3단계가 중요한 이유**: URScript가 시작되면 즉시 `q_desired`를 읽어 PD 토크를 계산한다. 초기값이 0이면 `Kp * (0 - q_actual)`이 되어 로봇이 원점으로 급격히 이동하려 한다. 반드시 현재 위치로 초기화해야 한다.

#### _control_loop_impedance() — RTDE 메인 루프

```python
# main.py:382-465 (요약)
while self.running:
    # 1. 입력 읽기 + 명령 처리
    cmd = self.input_handler.get_command(timeout=0.001)

    # 2-4. 목표 누적 → 필터 → 작업공간 클램핑
    #      (teleop_admittance와 동일한 패턴)

    # 5. Pink IK → q_desired
    q_desired = self.ik.solve(clamped_pos, filt_quat, dt)

    # 6. 실제 로봇 상태 읽기 (RTDE)
    q_actual = np.array(mgr.get_joint_positions())
    qd_actual = np.array(mgr.get_joint_velocities())

    # 7. 안전 검사 (q_desired vs q_actual 편차 포함)
    safety_result = self.safety.check(q_desired, q_actual, qd_actual)

    # 9. 명령 전송 또는 유지
    if safety_result.is_safe:
        mgr.set_desired_position(q_desired.tolist())   # q_desired → RTDE
    else:
        mgr.set_desired_position(q_actual.tolist())    # 현재 위치 유지 → 토크 0
```

**안전하지 않을 때 `q_actual`을 전송하는 이유**: `q_desired = q_actual`이면 `Kp * (q_d - q) = 0`, `Kd * qd ≈ 0`이므로 적용 토크가 거의 0이 된다. 로봇이 중력 보상만으로 현재 위치를 유지한다.

### 4.2 urscript_manager.py — URScriptManager

RTDE 인터페이스 3종을 관리하고, URScript 업로드를 담당한다.

#### RTDE 인터페이스 3종

```python
# urscript_manager.py:61-71
def connect(self):
    self._recv = rtde_receive.RTDEReceiveInterface(self._ip)  # 읽기 전용
    self._io = rtde_io.RTDEIOInterface(self._ip)              # 레지스터 쓰기
    for i in range(20):
        self._io.setInputDoubleRegister(i, 0.0)  # 모든 레지스터 초기화
```

| 인터페이스 | 역할 | 주요 메서드 |
|-----------|------|-----------|
| `RTDEReceiveInterface` | 로봇 상태 읽기 | `getActualQ()`, `getActualQd()` |
| `RTDEIOInterface` | 레지스터 쓰기 | `setInputDoubleRegister(i, val)` |
| `RTDEControlInterface` | URScript 업로드 | `FLAG_CUSTOM_SCRIPT`, `setCustomScriptFile()` |

#### URScript 업로드 메커니즘

```python
# urscript_manager.py:73-94
def upload_and_start(self, script_path=None):
    # FLAG_CUSTOM_SCRIPT: 기본 external_control.urscript 대신 커스텀 스크립트 사용
    flags = rtde_control.RTDEControlInterface.FLAG_CUSTOM_SCRIPT
    self._ctrl = rtde_control.RTDEControlInterface(self._ip, 125.0, flags)
    self._ctrl.setCustomScriptFile(str(path))   # 즉시 업로드 + 실행
```

`FLAG_CUSTOM_SCRIPT`가 핵심이다. 이 플래그 없이 `RTDEControlInterface`를 생성하면, ur_rtde 라이브러리가 기본 `external_control.urscript`를 업로드한다. 이 플래그로 우리의 `impedance_pd.script`를 대신 업로드한다.

#### Heartbeat 모니터링

```python
# urscript_manager.py:138-143
def is_script_alive(self) -> bool:
    hb = self.get_heartbeat()          # output register 6 읽기
    alive = hb != self._last_heartbeat  # 값이 변했으면 살아있음
    self._last_heartbeat = hb
    return alive
```

URScript가 500Hz로 실행되며 매 사이클마다 heartbeat를 +1 한다. Python에서 주기적으로 확인하여 URScript 크래시를 감지할 수 있다.

### 4.3 impedance_pd.script — URScript PD 컨트롤러

UR 컨트롤러 내부에서 500Hz로 실행되는 실시간 제어 루프이다.

#### PD 토크 계산

```
# impedance_pd.script:72-80
local q = get_actual_joint_positions()
local qd = get_actual_joint_speeds()

# PD 토크: τ = Kp·(q_d - q) - Kd·q̇
local tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
idx = 0
while idx < 6:
  local pos_err = q_d[idx] - q[idx]
  tau[idx] = kp[idx] * pos_err - kd[idx] * qd[idx]
  idx = idx + 1
end
```

#### 코리올리스 보상

```
# impedance_pd.script:82-91
local use_coriolis = read_input_float_register(19)
if use_coriolis > 0.5:
  local coriolis = get_coriolis_and_centrifugal_torques(q, qd)
  idx = 0
  while idx < 6:
    tau[idx] = tau[idx] + coriolis[idx]
    idx = idx + 1
  end
end
```

**왜 코리올리스 보상이 필요한가?** 로봇이 움직일 때 관절 간 동역학 커플링이 발생한다. 예: 어깨가 빠르게 회전하면 팔꿈치에 의도치 않은 토크가 작용한다. 이를 보상하지 않으면 다관절 동시 운동 시 경로가 왜곡된다.

`get_coriolis_and_centrifugal_torques()`는 **PolyScope 5.23.0+** 에서만 사용 가능하다.

#### 토크 포화 (안전)

```
# impedance_pd.script:93-102
# UR10e 토크 한계: [150, 150, 56, 56, 28, 28] Nm
local max_tau = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]
idx = 0
while idx < 6:
  if tau[idx] > max_tau[idx]:
    tau[idx] = max_tau[idx]
  elif tau[idx] < -max_tau[idx]:
    tau[idx] = -max_tau[idx]
  end
  idx = idx + 1
end
```

**URScript 내부 안전 장치**: Python 측 안전 검사가 실패하더라도, URScript 자체에서 토크를 UR10e 물리 한계 내로 포화시킨다.

#### direct_torque() 호출

```
# impedance_pd.script:116
direct_torque(tau, friction_comp=True)
```

- **중력 보상**: `direct_torque()`가 자동으로 처리. `tau`에 중력 항을 추가할 필요 없음.
- **마찰 보상**: `friction_comp=True`로 정마찰/동마찰을 보상. 저속에서의 스틱-슬립 현상을 줄임.

### 4.4 impedance_gains.py — 게인 프리셋

#### 관절별 게인 설계 근거

```python
# impedance_gains.py:33-46
# UR10e 관절 관성 기반:
#   joints 0-1 (어깨): 높은 관성 → 높은 게인 필요
#   joints 2-3 (팔꿈치/손목1): 중간 관성
#   joints 4-5 (손목2/손목3): 낮은 관성 → 낮은 게인
IMPEDANCE_PRESETS = {
    "STIFF":  ImpedanceGains(
        Kp=[800, 800, 400, 200, 100, 50],     # Nm/rad
        Kd=[40,  40,  20,  10,  5,   2.5]),    # Nm·s/rad
    "MEDIUM": ImpedanceGains(
        Kp=[400, 400, 200, 100, 50,  25],
        Kd=[20,  20,  10,  5,   2.5, 1.25]),
    "SOFT":   ImpedanceGains(
        Kp=[100, 100, 50,  25,  12.5, 6.25],
        Kd=[10,  10,  5,   2.5, 1.25, 0.625]),
}
```

**댐핑비 ≈ 0.7~1.0** (임계 감쇠~과감쇠):
- `ζ = Kd / (2√(I·Kp))` — 여기서 I는 관절 관성
- ζ < 1: 진동 발생 (위험)
- ζ = 1: 임계 감쇠 (진동 없이 가장 빠른 수렴)
- ζ > 1: 과감쇠 (느리지만 안전)

#### 런타임 스케일링

```python
# impedance_gains.py:94-100
def scale_up(self):
    self._scale = min(self._scale + GAIN_SCALE_STEP, GAIN_SCALE_MAX)  # +0.25, 최대 2.0

def scale_down(self):
    self._scale = max(self._scale - GAIN_SCALE_STEP, GAIN_SCALE_MIN)  # -0.25, 최소 0.25
```

`[` / `]` 키로 실행 중 게인을 0.25배 단위로 조절한다. Kp와 Kd가 **동시에** 같은 비율로 스케일링되어 댐핑비가 유지된다.

### 4.5 torque_safety.py — TorqueSafetyMonitor

토크 모드 전용 5단계 안전 시스템. admittance의 4단계와 비교:

| 우선순위 | 레벨 | teleop_admittance | teleop_impedance |
|---------|------|------------------|-----------------|
| 4 (최고) | ESTOP | O | O |
| 3 | DEVIATION | - | **O (신규)** — q_desired vs q_actual |
| 2 | VEL_LIMIT | O (스케일링) | O (거부) |
| 1 | TIMEOUT | O (200ms) | O (**100ms**, 더 엄격) |
| 0 | WS_CLAMP | O | O |

#### 위치 편차 검사 (토크 모드 고유)

```python
# torque_safety.py:117-125
deviation = np.abs(q_desired - q_actual)
max_dev = np.max(deviation)
if max_dev > self._config.max_position_deviation:  # 기본값: 0.3 rad (~17°)
    joint_idx = np.argmax(deviation)
    return TorqueSafetyResult(
        is_safe=False, level="DEVIATION",
        message=f"J{joint_idx+1} dev={np.degrees(max_dev):.1f}deg",
    )
```

**왜 토크 모드에만 이 검사가 있는가?** 위치 제어(admittance)에서는 로봇이 항상 명령 위치로 이동한다. 하지만 토크 제어에서는 외력이 크면 로봇이 목표에서 크게 벗어날 수 있다. 편차가 너무 크면 PD의 `Kp * error`가 과도한 토크를 생성하여 위험하다.

#### 타이밍 차이

```yaml
# teleop_admittance: packet_timeout_ms: 200
# teleop_impedance:  packet_timeout_ms: 100   (2배 엄격)
```

토크 모드는 위치 모드보다 본질적으로 위험하다. 통신 단절 시 마지막 `q_desired`가 유지되므로, 로봇이 해당 위치로 계속 힘을 가한다. 더 빠른 타임아웃 감지가 필수적이다.

---

## 5. Sim 모드 폴백

```python
# main.py:299-335
def _run_sim_fallback(self, cfg, dt):
    backend = create_backend("sim", ...)
    # ... 초기화 (teleop_admittance와 동일) ...
    self._control_loop_sim(cfg, dt, backend)
```

sim 모드에서는 `URScriptManager` 대신 일반 `RobotBackend`를 사용한다. 토크 대신 위치 명령(`send_joint_command`)을 전송한다. UI와 안전 검사는 동일하므로, RTDE 연결 없이도 **파이프라인을 테스트**할 수 있다.

---

## 6. teleop_admittance vs teleop_impedance 종합 비교

| 항목 | admittance | impedance |
|------|-----------|-----------|
| 제어 출력 | 위치 (servoJ) | 토크 (direct_torque) |
| 루프 | 단일 (Python 125Hz) | 듀얼 (Python 125Hz + URScript 500Hz) |
| 통신 | `backend.send_joint_command()` | RTDE input registers |
| 순응 방식 | F/T→동역학→Δx | 위치오차→PD→τ |
| F/T 센서 | 필수 | 불필요 |
| 안전 수준 | 높음 (위치 제어) | 중간 (토크 제어) |
| 접촉 품질 | 간접적 | 직접적 (물리적) |
| PolyScope | 제한 없음 | **5.23.0+** 필수 |
| Sim 모드 | 완전 동작 | 위치 폴백 |

---

## 다음 단계

- [core/ 매뉴얼](../../core/docs/manual.md) — 공유 인프라 심화 학습
- [teleop_admittance/ 매뉴얼](../../teleop_admittance/docs/manual.md) — 어드미턴스 텔레옵 비교
- [cumotion/ 매뉴얼](../../cumotion/docs/manual.md) — 경로 계획 → 텔레옵 조합 학습

> 실행 방법, CLI 옵션, 트러블슈팅은 [teleop_impedance/docs/user_guide.md](user_guide.md)를 참조하세요 (있는 경우).
