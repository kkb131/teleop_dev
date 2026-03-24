# teleop_admittance/ 학습 매뉴얼 — 어드미턴스 텔레옵 (난이도 ★★★)

> **위치**: `standalone/teleop_admittance/`
> **의존 core 모듈**: `robot_backend`, `pink_ik`, `exp_filter`, `input_handler`, `compliant_control`, `ft_source`, `controller_utils`
> **학습 선수 지식**: [core/ 매뉴얼](../../core/docs/manual.md) §2 (로봇 통신), §4.2 (Pink IK), §6 (필터), §8 (컴플라이언스)

---

## 1. 왜 teleop_admittance/가 존재하는가

servo/ 모듈은 가장 단순한 텔레옵을 제공하지만, 실제 로봇 조작에서는 부족한 점이 있다:

| servo/의 한계 | teleop_admittance/의 해결 |
|-------------|------------------------|
| 관절 한계 무시 (DLS IK) | Pink QP IK로 관절 한계 제약 |
| 외력 대응 불가 | F/T 센서 + 어드미턴스 제어로 외력 순응 |
| 안전 시스템 인라인 | 4단계 독립 SafetyMonitor (main.py) |
| 설정 하드코딩 | YAML 기반 설정 시스템 |
| 단일 파일 스크립트 | 모듈 분리 (main, nosafety, safety, admittance, config) |

### 어드미턴스 제어란?

**어드미턴스 제어**는 외력을 감지하여 **위치를 보정**하는 방식이다:

```
외력(F) → 어드미턴스 동역학(M·ẍ + D·ẋ + K·x = F) → 위치 변위(Δx) → IK → 관절 명령
```

인과관계: **힘 → 위치** (Force → Position).
로봇이 외부 힘에 순응하여 부드럽게 밀리는 동작을 구현한다.

---

## 2. 제어 파이프라인 상세

### main.py (안전 모드 포함)

```
Input → ExpFilter → Workspace Clamp → [Admittance Δx] → Pink IK → SafetyMonitor → Robot
  ↑                                        ↑
  keyboard/xbox                       F/T sensor
```

### teleop_nosafety.py (안전 모드 없음)

```
Input → ExpFilter → [Admittance Δx] → Pink IK → Robot (servoJ)
  ↑                       ↑
  keyboard/xbox       F/T sensor
```

Safety가 없으므로 Workspace Clamp, Velocity Limit, Timeout, E-Stop 모두 생략된다.
IK 솔버(Pink QP)의 관절 한계 제약만 적용된다.

### 10단계 제어 루프 (`_control_loop()` — main.py)

```python
# main.py:297-405
while self.running:
    t_start = time.perf_counter()

    # 1. 입력 읽기 — cmd.velocity[6] (linear + angular)
    cmd = self.input_handler.get_command(timeout=0.001)

    # 2. 목표 위치 누적 — 키를 누르고 있으면 계속 이동
    target_pos = target_pos + cmd.velocity[:3]
    target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

    # 3. 지수 필터 — 급격한 변화 완화
    filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

    # 4. 작업 공간 클램핑 — EE 위치를 안전 범위로 제한
    clamped_pos = self.safety.clamp_workspace(filt_pos)

    # 4.5 어드미턴스 변위 — F/T 센서 기반 위치 보정
    adm_disp = self.admittance.compute_displacement(self.q_current, dt)
    compliant_pos = clamped_pos + adm_disp[:3]

    # 5. Pink IK — 카르테시안 → 관절 공간 변환 (QP, 관절 한계 준수)
    q_target = self.ik.solve(compliant_pos, compliant_quat, dt)

    # 6. 안전 검사 — 속도 제한, 타임아웃, E-Stop
    result = self.safety.check_and_apply(q_target, self.q_current, dt)

    # 7. EE 속도 계산 (디스플레이용)
    # 8. 로봇에 명령 전송 (안전하면 전송, 아니면 현재 위치 유지)
    # 9. 상태 디스플레이 + CSV 로그
    # 10. 루프 타이밍 (dt 맞추기 위한 sleep)
```

### teleop_nosafety.py 제어 루프 핵심 차이

```python
# teleop_nosafety.py:248-321
# 4단계가 없음 — 필터 출력에 바로 admittance 합산
compliant_pos = filt_pos + adm_disp[:3]

# IK 결과를 직접 로봇에 전송 (safety check 없음)
backend.send_joint_command(q_target.tolist())
self.q_current = q_target.copy()
```

**핵심 포인트**: 어드미턴스 변위(4.5단계)는 사용자 입력과 **합산**된다. 사용자가 키보드로 로봇을 이동시키면서 동시에 외력에 의해 순응 동작이 추가된다.

---

## 3. 핵심 모듈 분석

### 3.1 main.py — TeleopController

#### 초기화 순서

```python
# main.py:79-98
class TeleopController:
    def __init__(self, config: TeleopConfig, log_path=None):
        self.exp_filter = ExpFilter(
            config.filter.alpha_position,      # 위치 EMA (0.85)
            config.filter.alpha_orientation,   # 자세 slerp (0.85)
        )
        self.ik = PinkIK(
            config.urdf_path,
            ee_frame="tool0",
            position_cost=config.ik.position_cost,      # 1.0
            orientation_cost=config.ik.orientation_cost,  # 0.5
            posture_cost=config.ik.posture_cost,          # 1e-3
            damping=config.ik.damping,                    # 1e-12
        )
```

- `position_cost > orientation_cost`: 위치 추종이 자세보다 우선
- `posture_cost = 1e-3`: 관절 중립 위치로 복귀 (약한 힘)
- `damping = 1e-12`: QP 수치 안정성용 (거의 0)

#### run() 초기화 흐름

```python
# main.py:223-270
def run(self):
    self.backend = create_backend(cfg.robot.mode, ...)   # 1. 백엔드 생성
    self.input_handler = create_input(cfg.input.type, ...)  # 2. 입력 장치
    if cfg.robot.mode == "sim":
        self._setup_sim_controller()  # 3. sim: forward_position_controller 전환
    # ...
    self.q_current = np.array(self.backend.get_joint_positions())
    self.ik.initialize(self.q_current)          # 4. Pink IK 초기화 ⚠️ 중요
    self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
    self.exp_filter.reset(self.ee_pos, self.ee_quat)  # 5. 필터 초기화
    self.safety = SafetyMonitor(cfg.safety, self.backend)  # 6. 안전 모니터
    self.admittance = AdmittanceLayer(...)       # 7. 어드미턴스 레이어
```

sim 모드에서 ROS2 mock hardware를 사용할 때, `forward_position_controller`로 전환해야 실시간 위치 명령이 가능하다. Isaac Sim은 controller_manager가 없으므로 자동 스킵된다.

### 3.2 admittance_layer.py — AdmittanceLayer

core/의 `AdmittanceController`와 `FTSource`를 래핑하여 텔레옵 파이프라인에 통합하는 역할이다.

#### F/T 소스 생성

```python
# admittance_layer.py:41-45
if mode == "rtde":
    raw_ft = RTDEFTSource(backend)
    self._ft_source: FTSource = BaseFrameFTSource(raw_ft)  # negate X,Y
else:
    self._ft_source = NullFTSource()   # 항상 0 반환 → 어드미턴스 무효
```

- **RTDE 모드**: `RTDEFTSource` → `BaseFrameFTSource`로 래핑. `BaseFrameFTSource`가 렌치의 X, Y 성분을 부호 반전하여 TCP 프레임에서 base 프레임으로 변환한다.
- **Sim 모드**: `NullFTSource` → 항상 0 렌치 → `compute_displacement()`도 항상 0 반환

#### 핵심 메서드: compute_displacement()

```python
# admittance_layer.py:79-97
def compute_displacement(self, q: np.ndarray, dt: float) -> np.ndarray:
    if not self._enabled:
        return np.zeros(6)

    # FTSource가 이미 base 프레임 렌치를 반환함
    # (BaseFrameFTSource가 TCP→base 변환을 내부적으로 처리)
    wrench_base = self._ft_source.get_wrench()

    return self._controller.update(wrench_base, dt)
```

**이전 버전과의 차이**: 이전에는 `compute_displacement()` 내부에서 Pinocchio FK rotation을 사용하여 렌치를 변환했지만, 실제 로봇 테스트 결과 `BaseFrameFTSource`의 **negate X,Y** 방식이 올바른 것으로 확인되었다. 현재 코드에서 `q` 파라미터는 API 호환성을 위해 유지되지만 사용되지 않는다.

**왜 FK rotation이 아니라 negate X,Y인가?**

UR 로봇의 `getActualTCPForce()`는 TCP 프레임에서 렌치를 반환한다. 이를 base 프레임으로 변환하려면:
- URDF `base_link`(REP-103, X+ forward)와 UR 컨트롤러 `Base` 프레임은 Z축 기준 180° 회전 관계
- TCP 렌치의 X, Y를 부호 반전하면 base_link 프레임과 정합됨
- `test_wrench_frame.py`의 6가지 변환 후보 테스트에서 "negate X,Y only"가 올바른 것으로 경험적으로 검증됨

#### 런타임 제어

| 키 | 동작 | 코드 |
|----|------|------|
| `t` | 어드미턴스 ON/OFF 토글 | `toggle()` |
| `z` | F/T 센서 제로잉 | `zero_sensor()` |
| `1/2/3` | STIFF/MEDIUM/SOFT 프리셋 | `set_preset("STIFF")` 등 |

### 3.3 safety_monitor.py — SafetyMonitor (main.py 전용)

4단계 안전 시스템. 모든 결과는 `SafetyResult` 데이터클래스로 반환된다.

```python
@dataclass
class SafetyResult:
    is_safe: bool
    q_safe: np.ndarray     # 안전한 관절 명령 (스케일링 적용 후)
    level: str = "OK"       # "OK", "TIMEOUT", "VEL_LIMIT", "WS_CLAMP", "ESTOP"
    message: str = ""
```

#### 4단계 계층 구조

| 우선순위 | 레벨 | 검사 대상 | 동작 |
|---------|------|---------|------|
| 4 (최고) | ESTOP | 사용자 Space 키 | 즉시 정지, 수동 리셋(R키) 필요 |
| 3 | WS_CLAMP | EE 위치 | 작업 공간 경계로 클램핑 (거부 아닌 제한) |
| 2 | VEL_LIMIT | 관절 속도 | 속도 비례 스케일링 `q_safe = q_current + Δq * scale` |
| 1 | TIMEOUT | 입력 간격 | 200ms 무입력 시 현재 위치 유지 |

#### 속도 스케일링 핵심 코드

```python
# safety_monitor.py:117-126
joint_vel = (q_target - q_current) / dt
max_vel = np.max(np.abs(joint_vel))

if max_vel > self._config.max_joint_vel:
    scale = self._config.max_joint_vel / max_vel   # 0~1 사이 비율
    q_safe = q_current + (q_target - q_current) * scale
```

**모든 관절을 동일 비율로 스케일링**: 특정 관절만 줄이면 카르테시안 경로가 왜곡된다. 전체를 같은 비율로 줄여 직선 경로를 유지한다.

### 3.4 teleop_nosafety.py — TeleopNoSafety

main.py의 `TeleopController`를 기반으로 SafetyMonitor 관련 코드만 제거한 버전.

**제거된 항목:**
- `SafetyMonitor` import/생성/호출 전체
- `clamp_workspace()` 호출 (Level 3)
- `check_and_apply()` 호출 (Levels 1,2,4)
- E-Stop/Reset 로직
- Timeout 체크, Velocity limiting
- CSV 로깅

**유지된 항목:**
- `create_input()` (keyboard/xbox)
- `ExpFilter` (smoothing)
- `PinkIK` (QP IK) — `ik.initialize()` 포함
- `AdmittanceLayer` (F/T compliance)
- HUD 표시 (6줄, Safety/E-Stop 라인 제거)
- Sim mode controller 전환

**용도**: 안전 시스템 없이 순수 어드미턴스 텔레옵 동작을 테스트/디버깅할 때 사용.

### 3.5 teleop_config.py — TeleopConfig

YAML → Python 데이터클래스 매핑.

```python
# teleop_config.py
@dataclass
class TeleopConfig:
    robot: RobotConfig           # ip, mode
    control: ControlConfig       # frequency_sim(50Hz), frequency_rtde(125Hz)
    input: InputConfig           # type, cartesian_step, rotation_step
    filter: FilterConfig         # alpha_position, alpha_orientation
    ik: IKConfig                 # position_cost, orientation_cost, damping
    safety: SafetyConfig         # timeout, max_vel, workspace
    admittance: AdmittanceConfig # preset, max_displacement, deadzone

    @property
    def frequency(self) -> int:
        if self.robot.mode == "rtde":
            return self.control.frequency_rtde   # 125Hz (servoJ에 맞춤)
        return self.control.frequency_sim         # 50Hz (시뮬레이션 충분)
```

`frequency` 프로퍼티가 모드에 따라 자동 전환되므로, 나머지 코드는 `config.frequency`만 참조하면 된다. `teleop_nosafety.py`에서도 동일한 `TeleopConfig`를 사용하며, safety 설정은 무시된다.

---

## 4. `ik.initialize()` vs `ik.sync_configuration()` — 핵심 함정

Pink IK를 사용할 때 반드시 이해해야 하는 두 메서드의 차이:

| 메서드 | 역할 | 필수 여부 |
|--------|------|----------|
| `ik.initialize(q)` | PostureTask target 설정 + Configuration 생성 | **최초 1회 필수** |
| `ik.sync_configuration(q)` | Configuration state만 업데이트 | 매 루프 호출 |

### 왜 중요한가?

Pink QP 솔버는 `PostureTask`가 있어야 정상 동작한다. `PostureTask`는 관절이 목표 자세(보통 초기 자세)에서 크게 벗어나지 않도록 하는 약한 제약이다.

**`initialize()`를 호출하지 않으면:**
- PostureTask target이 설정되지 않음
- QP 솔버가 의미 있는 결과를 생성하지 못함
- 로봇이 전혀 움직이지 않음 (solve()가 현재 위치와 동일한 값 반환)

**올바른 패턴:**
```python
# 초기화 시
ik.initialize(q_current)  # PostureTask target 설정 — 필수!

# 매 루프
ik.sync_configuration(q_current)  # Configuration만 업데이트
q_target = ik.solve(target_pos, target_quat, dt)
```

**리셋 시 (E-Stop 해제, 센서 제로잉 등):**
- `test_admittance.py`에서는 리셋 시에도 `ik.initialize(q)` 재호출 (PostureTask target을 현재 위치로 갱신)
- `main.py`, `teleop_nosafety.py`에서는 `ik.sync_configuration(q)` 사용 (이미 initialize 완료 상태이므로 Configuration 동기화만 필요)

---

## 5. 테스트/진단 도구

### 5.1 test_admittance.py — 순수 F/T 어드미턴스 테스터

키보드/Xbox 입력 핸들러 없이, F/T 센서만으로 로봇을 밀어서 어드미턴스 동작을 확인하는 최소 파이프라인:

```
RTDEFTSource → BaseFrameFTSource (negate X,Y) → AdmittanceController → PinkIK → servoJ
```

핵심 코드에서 `ik.initialize(q_current)`를 초기화와 리셋 시점 모두에서 호출한다.

### 5.2 test_wrench_frame.py — 렌치 프레임 진단

F/T 센서 렌치의 좌표계 변환이 올바른지 시각적으로 확인하는 도구. 6가지 변환 후보를 동시에 표시:

1. raw (변환 없음)
2. R_ur @ wrench (UR TCP pose rotation)
3. R_ur.T @ wrench
4. **negate X,Y only** ← 올바른 변환 (경험적 검증)
5. R_ur @ wrench + negXY
6. R_fk @ wrench (Pinocchio FK rotation)

두 가지 모드:
- **센서 모드** (기본): 렌치 값만 표시, 로봇 안 움직임
- **서보 모드** (`--servo`): 선택한 변환으로 실제 어드미턴스 → IK → servoJ 수행

---

## 6. 어드미턴스 vs 임피던스: 근본적 차이

이 모듈(teleop_admittance)과 다음 모듈(teleop_impedance)의 근본적 차이를 이해하는 것이 중요하다.

| 항목 | 어드미턴스 (이 모듈) | 임피던스 (teleop_impedance) |
|------|---------------------|--------------------------|
| 인과관계 | F → x (힘 → 위치) | x → τ (위치 → 토크) |
| 제어 출력 | 위치 명령 (servoJ) | 토크 명령 (direct_torque) |
| 순응성 구현 | 소프트웨어 동역학 시뮬레이션 | 물리적 스프링-댐퍼 (PD 토크) |
| F/T 센서 | **필수** (외력 측정) | 불필요 (위치 오차가 곧 힘) |
| 안전성 | 높음 (위치 제어 기반) | 낮음 (토크 직접 제어) |
| 접촉 품질 | 간접적 (센서→계산→명령) | 직접적 (물리적 컴플라이언스) |

**비유**:
- 어드미턴스 = "센서로 힘을 측정하고, 컴퓨터가 계산해서, 로봇에게 새 위치를 알려줌"
- 임피던스 = "스프링으로 연결된 것처럼, 위치 차이가 곧바로 힘이 됨"

---

## 7. 주요 패턴과 설계 결정

### 영속적 목표(Persistent Target) 패턴

```python
# main.py:303-304
target_pos = self.ee_pos.copy()   # 초기화: 현재 EE 위치
target_quat = self.ee_quat.copy()

# main.py:348-349
target_pos = target_pos + cmd.velocity[:3]   # 누적 (+=)
target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)
```

키를 누르면 `target_pos`가 **계속 누적**된다. 키를 놓으면 `cmd.velocity = 0`이므로 목표가 유지된다.
이전 루프에서 얼마나 이동했는지와 무관하게, 목표 자체가 독립적으로 전진한다.

### E-Stop 후 상태 재동기화 (main.py)

```python
# main.py:324-333
if cmd.reset:
    self.safety.reset_estop()
    self.q_current = np.array(self.backend.get_joint_positions())
    self.ik.sync_configuration(self.q_current)      # Pink IK 상태 리셋
    self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
    self.exp_filter.reset(self.ee_pos, self.ee_quat) # 필터 리셋
    self.admittance.reset()                           # 어드미턴스 리셋
    target_pos = self.ee_pos.copy()                  # 목표도 현재 위치로
```

E-Stop 해제 시 모든 상태를 현재 로봇 위치로 **재동기화**한다. 이를 빠뜨리면 누적된 목표와 현재 위치의 차이로 인해 로봇이 급격히 점프한다.

---

## 다음 단계

- [teleop_impedance/ 매뉴얼](../../teleop_impedance/docs/manual.md) — 임피던스 텔레옵 (토크 직접 제어)
- [core/ 매뉴얼](../../core/docs/manual.md) — 공유 인프라 심화 학습

> 실행 방법, CLI 옵션, 트러블슈팅은 [user_guide.md](user_guide.md)를 참조하세요.
