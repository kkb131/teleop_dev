# UR10e 임피던스 제어 — 핵심 이론 및 튜닝 가이드

> **대상**: 임피던스 텔레옵 시스템의 이론적 배경, 사용 기술, 파라미터 튜닝을 이해하고 싶은 개발자
> **관련 문서**: [implementation_notes.md](implementation_notes.md) (구현 디버깅 기록), [user_guide.md](user_guide.md) (사용법), [manual.md](manual.md) (원래 설계)

---

## 1. 핵심 이론: 관절 공간 PD 임피던스 제어

### 1.1 제어 공식

```
tau = Kp * (q_desired - q_actual) - Kd * q_dot_actual + C(q, q_dot)
```

이 한 줄이 전체 임피던스 제어의 핵심이다. 각 항의 물리적 의미:

| 항 | 공식 | 물리적 비유 | 역할 |
|----|------|------------|------|
| **위치 항** | `Kp * (q_d - q)` | 가상 스프링 | 목표 위치에서 벗어나면 되돌리는 복원력 |
| **속도 항** | `-Kd * q_dot` | 가상 댐퍼 | 빠르게 움직일수록 저항 → 진동 억제 |
| **코리올리스 항** | `C(q, q_dot)` | 동역학 보상 | 다관절 운동 시 관절 간 간섭력 상쇄 |
| **중력 보상** | (자동) | — | `direct_torque()` API가 내부적으로 처리 |
| **마찰 보상** | (자동) | — | `friction_comp=True`로 정마찰/동마찰 보상 |

**핵심 직관**: 로봇의 각 관절에 가상의 스프링과 댐퍼가 달려 있다고 생각하면 된다.
- 스프링(Kp)이 강하면 → 목표를 강하게 추종 (STIFF)
- 스프링(Kp)이 약하면 → 외력에 쉽게 밀림 (SOFT)
- 댐퍼(Kd)가 적절하면 → 진동 없이 부드럽게 수렴

### 1.2 댐핑비 (Damping Ratio, ζ)

```
ζ = Kd / (2 * sqrt(I * Kp))
```

여기서 `I`는 관절의 유효 관성. 댐핑비에 따라 동작 특성이 결정된다:

| ζ 값 | 이름 | 동작 | 위험도 |
|------|------|------|--------|
| ζ < 0.7 | 부족감쇠 | 목표 주변에서 **진동** | 높음 (발진 가능) |
| ζ = 0.7~1.0 | 임계감쇠~약과감쇠 | 진동 없이 **빠르게 수렴** | 낮음 (이상적) |
| ζ > 1.0 | 과감쇠 | 진동 없지만 **느리게 수렴** | 매우 낮음 (안전) |

**설계 목표**: 모든 프리셋에서 ζ ≈ 0.7~1.0을 유지. Kp를 변경하면 반드시 Kd도 비례 조정해야 한다.

### 1.3 관절별 게인이 다른 이유

UR10e의 6개 관절은 관성(I)이 크게 다르다:

```
J0(Base) → J1(Shoulder) → J2(Elbow) → J3(Wrist1) → J4(Wrist2) → J5(Wrist3)
 높은 관성              중간 관성              낮은 관성
```

- **J0-J1 (어깨)**: 전체 팔의 관성을 부담 → 높은 Kp/Kd 필요
- **J2-J3 (팔꿈치/손목1)**: 중간 관성
- **J4-J5 (손목2/3)**: 작은 관성 → 낮은 Kp/Kd로도 충분. 과도한 게인은 진동 유발

현재 프리셋의 Kp 비율: `16 : 16 : 8 : 4 : 2 : 1` (STIFF 기준 800:800:400:200:100:50)

### 1.4 코리올리스/원심력 보상

다관절 로봇이 움직일 때, 한 관절의 회전이 다른 관절에 의도치 않은 토크를 가한다:
- 어깨(J0)가 빠르게 회전 → 팔꿈치(J2)에 원심력 토크 발생
- 두 관절이 동시에 움직이면 → 코리올리스 힘이 상호 작용

보상 없이는 다관절 동시 운동 시 경로가 왜곡된다.

**구현**:
```python
# Pinocchio로 코리올리스 행렬 C(q, q_dot) 계산
pin.computeCoriolisMatrix(model, data, q, qd)
tau_coriolis = data.C @ qd  # C(q,qd) * q_dot
```

- `robot/arm/impedance/urscript_manager.py`의 `get_coriolis()` 메서드
- Pinocchio가 URDF에서 로봇 동역학 모델을 빌드하여 계산
- 활성화: `config/default.yaml`의 `impedance.enable_coriolis_comp: true`

---

## 2. 사용 기술 스택

### 2.1 제어 파이프라인 전체도

```
Input (Keyboard/Xbox/Network)
    │
    ▼
ExpFilter (EMA 위치 + SLERP 자세)     ← core/exp_filter.py
    │
    ▼
Workspace Safety Clamp                 ← torque_safety.py
    │
    ▼
Pink IK (QP 역기구학)                  ← core/pink_ik.py
    │  ↕ soft_sync (IK drift 방지)
    ▼
PD 토크 계산 (Python 125Hz)            ← main.py
    │  + 코리올리스 보상 (Pinocchio)    ← urscript_manager.py
    ▼
RTDE 레지스터 → URScript (500Hz)       ← impedance_pd.script
    │
    ▼
direct_torque() → 로봇 모터
```

### 2.2 Pinocchio — 로봇 동역학 엔진

**역할**: URDF에서 로봇 모델을 빌드하여 다양한 동역학 계산 수행

| 기능 | 용도 | 사용 위치 |
|------|------|----------|
| Forward Kinematics (FK) | 관절 각도 → EE 위치/자세 | `pink_ik.py:get_ee_pose()` |
| Coriolis Matrix | C(q, q_dot) 계산 | `urscript_manager.py:get_coriolis()` |
| URDF Parser | 로봇 모델 빌드 | `pink_ik.py`, `urscript_manager.py` |

**왜 Pinocchio인가?**
- C++ 백엔드로 빠른 계산 (~1ms 이내)
- URDF에서 자동으로 관성, 링크 길이, 관절 한계 추출
- URScript에는 행렬 연산 기능이 없으므로 Python에서 계산 필수

### 2.3 Pink IK — QP 기반 역기구학 솔버

**역할**: 목표 EE 포즈(x, y, z, roll, pitch, yaw) → 관절 각도(q_desired) 변환

**핵심 개념**: Quadratic Programming(QP)으로 여러 목표(Task)를 동시에 최적화

```
minimize:  position_error² × position_cost
         + orientation_error² × orientation_cost
         + joint_deviation² × posture_cost
subject to: joint limits, velocity limits
```

| Task | 역할 | cost | 파일 |
|------|------|------|------|
| **FrameTask** | EE를 목표 포즈로 추종 | position: 1.0, orientation: 0.5 | `pink_ik.py` |
| **PostureTask** | 관절을 초기 자세로 유지 (정규화) | 1e-2 | `pink_ik.py` |

**QP 솔버**: ProxQP (`proxsuite` 패키지). 관절 한계를 QP 제약조건으로 처리.

**왜 Pink/QP인가?**
- 단순 Jacobian pseudoinverse 대비: 관절 한계 자동 처리, null-space 활용
- 여러 목표(EE 추종 + 자세 유지)를 가중치로 자연스럽게 조합
- 특이점 근처에서도 안정적 (damping으로 수치적 안정성 보장)

### 2.4 ExpFilter — 입력 스무딩

**역할**: 키보드/조이스틱의 이산적인 입력을 부드러운 연속 신호로 변환

| 구분 | 알고리즘 | 공식 |
|------|----------|------|
| **위치** | Exponential Moving Average (EMA) | `pos_new = α * input + (1-α) * pos_prev` |
| **자세** | Spherical Linear Interpolation (SLERP) | `quat_new = slerp(quat_prev, input, α)` |

- α = 1.0: 필터링 없음 (즉시 반영)
- α = 0.5: 50% 반영 (강한 스무딩)
- α = 0.85 (기본값): 85% 반영 (적당한 반응성)

**왜 자세에 SLERP을 쓰나?**
쿼터니언을 단순 선형 보간하면 정규화가 깨진다. SLERP은 4D 단위 구면 위에서 보간하여 항상 유효한 회전을 보장한다.

### 2.5 URScript 500Hz 토크 릴레이

**왜 듀얼 루프인가?**
- 토크 제어는 **높은 주파수**가 필수 (낮으면 진동/불안정)
- IK + 코리올리스 계산은 Python에서만 가능 (URScript에 행렬 연산 없음)
- 해결: Python이 125Hz로 토크를 계산하고, URScript가 500Hz로 마지막 토크를 반복 적용

```
Python 125Hz:  τ₁ -------- τ₂ -------- τ₃ --------
URScript 500Hz: τ₁ τ₁ τ₁ τ₁ τ₂ τ₂ τ₂ τ₂ τ₃ τ₃ τ₃ τ₃
```

### 2.6 RTDE — Python ↔ URScript 통신

| 인터페이스 | 역할 | 사용 이유 |
|-----------|------|----------|
| **RTDEReceiveInterface** | 로봇 상태 읽기 (q, qd, TCP) | 상태 모니터링 |
| **RTDEIOInterface** | 레지스터 쓰기 (토크, 모드) | 토크 명령 전달 |
| TCP Socket (port 30002) | URScript 업로드 | RTDEControlInterface가 hang됨 |

**주의**: RTDEControlInterface는 우리 UR10e에서 연결 시 무한 대기 발생. 사용 금지.

---

## 3. 핵심 파라미터와 영향

### 3.1 임피던스 게인 (Kp, Kd)

```python
# impedance_gains.py
IMPEDANCE_PRESETS = {
    "STIFF":  Kp=[800, 800, 400, 200, 100, 50],  Kd=[80, 80, 40, 20, 10, 5]
    "MEDIUM": Kp=[600, 600, 300, 150,  75, 37.5], Kd=[64, 64, 32, 16,  8, 4]
    "SOFT":   Kp=[400, 400, 200, 100,  50, 25],   Kd=[48, 48, 24, 12,  6, 3]
}
```

| 파라미터 | 올리면 | 내리면 | 단위 |
|----------|--------|--------|------|
| **Kp** | 더 강하게 추종, 외력 저항 ↑ | 더 유연, 외력에 쉽게 밀림 | Nm/rad |
| **Kd** | 진동 억제 ↑, 움직임 느려짐 | 빠른 응답, 진동 위험 ↑ | Nm·s/rad |
| **gain_scale** | Kp×Kd 동시 비례 증가 | 동시 비례 감소 | 0.25~2.0 |

**중요**: Kp만 올리고 Kd를 그대로 두면 댐핑비 감소 → **진동 발생 위험**. 반드시 함께 조정.

### 3.2 max_joint_error — IK 오차 클램프

```python
# config/default.yaml
max_joint_error: [0.10, 0.10, 0.12, 0.18, 0.18, 0.2]  # rad
```

**역할**: `q_error = clip(q_desired - q_actual, -max_error, max_error)`

IK 솔버가 한 스텝에서 큰 q_desired 점프를 출력하면, `Kp * large_error`로 과도한 토크가 발생한다. 이 클램프가 토크 스파이크를 방지한다.

| 값 | 효과 | 트레이드오프 |
|----|------|-------------|
| 작게 (0.05~0.08) | 매우 부드러운 모션 | 빠른 움직임에 **못 따라감** (sluggish) |
| 크게 (0.2~0.3) | 빠른 반응 | 토크 점프로 **jerk** 발생 가능 |
| 관절별 차등 | 관성 큰 관절은 작게, 작은 관절은 크게 | 관성에 맞춘 최적화 |

**왜 관절별로 다른가?**
- J0-J1 (어깨, 높은 관성): 작은 오차도 큰 토크 → 0.10 rad로 타이트하게
- J4-J5 (손목, 낮은 관성): 빠른 반응 필요 + 관성 작아서 안전 → 0.18~0.20 rad

### 3.3 posture_cost — IK null-space 정규화

```python
# config/default.yaml → ik.posture_cost: 1.0e-2
```

**역할**: IK 솔버가 EE 목표를 달성하면서 **남는 자유도**(null-space)를 어떻게 쓸지 결정.

6관절 UR10e는 6DOF → null-space 없어야 하지만, Pink IK의 QP 솔버는 비용 기반이므로 posture_cost가 "관절 중심으로 돌아가려는 힘"의 크기를 결정한다.

| posture_cost | 효과 | 문제 |
|-------------|------|------|
| 1e-3 (약) | EE 추종 자유로움 | **Yaw 시 wrist1/2가 불필요하게 회전** (관절 커플링) |
| 1e-1 (강) | 관절 안정적 유지 | **EE 움직임 제한** (PostureTask가 FrameTask와 경쟁) |
| **1e-2 (현재)** | 균형 타협 | 적절한 안정성 + 적절한 자유도 |

**PostureTask 한계**: Pink의 PostureTask는 스칼라 cost만 지원. 관절별 가중치를 줄 수 없다. 따라서 전체 관절에 균일한 정규화가 적용된다.

### 3.4 ExpFilter alpha — 입력 스무딩

```yaml
# config/default.yaml
filter:
  alpha_position: 0.85
  alpha_orientation: 0.85
```

| alpha | 스무딩 정도 | 적합한 상황 |
|-------|-----------|------------|
| 0.5~0.7 | 강한 스무딩 (느린 반응) | 노이즈 많은 입력 (네트워크 지연) |
| 0.8~0.9 | 적당한 스무딩 | 키보드/조이스틱 (기본) |
| 0.95~1.0 | 거의 없음 (즉각 반응) | 정밀한 직접 제어 |

### 3.5 soft_sync alpha — IK drift 방지

```python
# core/pink_ik.py
def soft_sync(self, q_actual, alpha=0.05):
    q_blend = (1 - alpha) * q_ik_internal + alpha * q_actual
```

| alpha | 동작 | 트레이드오프 |
|-------|------|-------------|
| 0.0 (sync 없음) | IK가 자유롭게 적분 | **drift → 발작** (장시간 사용 불가) |
| 0.05 (기본) | 5%씩 actual 방향으로 수렴 | IK lead-ahead 유지 + drift 방지 |
| 0.2~0.5 (강) | 강하게 actual에 고정 | IK 반응 느림 (almost sync_configuration) |
| 1.0 (hard sync) | 매 스텝 actual에서 시작 | IK 한 스텝만 앞서감 → **매우 느린 반응** |

**time constant**: 125Hz에서 alpha=0.05이면 약 160ms 이내에 drift가 63% 수렴.

---

## 4. 실제 문제 해결 사례

### 4.1 Yaw 커플링 문제

**증상**: Yaw(J/L) 키를 누르면 wrist3만 회전해야 하는데, **wrist1/2도 함께 불필요하게 회전**.

**원인**: IK의 posture_cost가 너무 낮아서 (1e-3), null-space 정규화가 약함. IK가 EE 자세만 맞추고 관절 배치에는 신경 쓰지 않아서, 여러 관절을 동시에 움직여 같은 EE 회전을 달성.

**해결**:
```yaml
# posture_cost: 1e-3 → 1e-2 (10배 강화)
ik:
  posture_cost: 1.0e-2
```

posture_cost를 올리면 IK가 "기존 관절 자세를 유지하면서" EE를 움직이려 한다. Yaw 입력 시 wrist3만 주로 회전하게 됨.

**교훈**: posture_cost는 "어느 관절을 움직일지"에 큰 영향. 너무 낮으면 의도치 않은 관절 커플링 발생.

### 4.2 Roll/Pitch 발작 (IK Drift)

**증상**: Roll/Pitch/Yaw 입력이 처음에는 정상 동작하다가, 1~2분 후 **갑자기 발작하듯 이상하게 움직임**.

**근본 원인**:
1. Pink IK는 매 스텝 `integrate_inplace(velocity, dt)`로 내부 q를 적분
2. 실제 로봇은 PD + max_joint_error 클램프로 IK를 완벽 추종 불가
3. 시간이 지나면 IK 내부 q ↔ 실제 q_actual 간 **drift 누적**
4. IK가 실제와 전혀 다른 kinematic region에서 풀이 → **특이점/관절한계 진입 → 발작**

**해결**: Soft Sync — 매 스텝 IK 풀이 전에 내부 상태를 actual 방향으로 5% 블렌딩

```python
# main.py (제어 루프, IK solve 직전)
q_actual = np.array(mgr.get_joint_positions())
self.ik.soft_sync(q_actual)  # 5% 블렌딩
q_ik = self.ik.solve(clamped_pos, filt_quat, dt)
```

**왜 hard sync가 아닌 soft sync인가?**
- Hard sync (매 스텝 actual 사용): IK가 매번 actual에서 출발 → 한 스텝만 앞서감 → 느린 반응
- Soft sync (5% 블렌딩): IK의 "앞서나가기" 유지하면서 drift만 제한 → 반응성 + 안정성 동시 확보

### 4.3 번역/회전이 느려지는 문제 (Sluggish Motion)

**증상**: posture_cost를 올리고 max_joint_error를 타이트하게 잡으니, 이동이 **뻑뻑하고 느리게** 느껴짐.

**원인**: 두 파라미터가 동시에 모션을 제한:
- 높은 posture_cost → IK가 관절을 덜 움직이려 함
- 작은 max_joint_error → 큰 q_error를 클램프하여 토크 제한

**해결**: max_joint_error를 관절별로 완화 (특히 손목 관절)

```yaml
# 이전: [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]  # 너무 타이트
# 현재: [0.10, 0.10, 0.12, 0.18, 0.18, 0.2]    # 관절 관성에 맞춤
```

**교훈**: posture_cost와 max_joint_error는 상호 작용한다. 둘 다 타이트하면 모션이 과도하게 제한된다. 하나를 올리면 다른 하나를 완화해야 균형이 맞는다.

### 4.4 프리셋 재구성

**과정**: 기존 STIFF/MEDIUM/SOFT의 간격이 실제 사용감과 맞지 않아 재구성.

**설계 원칙**:
1. 관절 관성 비율에 맞게 Kp 배분: `16:16:8:4:2:1`
2. Kd는 ζ ≈ 0.7~1.0 유지하도록 Kp에 비례
3. STIFF 고정, SOFT를 실제 "부드럽다"고 느껴지는 수준으로 조정
4. MEDIUM은 STIFF와 SOFT의 중간값

---

## 5. 튜닝 가이드

### 5.1 증상별 파라미터 조정

| 증상 | 원인 | 조정할 파라미터 | 방향 |
|------|------|----------------|------|
| **진동/떨림** | 댐핑 부족 | Kd ↑ 또는 gain_scale ↓ | Kd를 Kp 대비 높이기 |
| **외력에 너무 뻣뻣** | 게인 과다 | 프리셋을 SOFT로, gain_scale ↓ | |
| **외력에 너무 물렁** | 게인 부족 | 프리셋을 STIFF로, gain_scale ↑ | |
| **Yaw 시 여러 관절 동시 회전** | posture_cost 부족 | `ik.posture_cost` ↑ (1e-2 → 5e-2) | |
| **이동이 느리고 뻑뻑함** | 클램프 과다 | `max_joint_error` 완화 | 특히 손목(J4-J5) 값 키우기 |
| **장시간 후 발작** | IK drift | `soft_sync` alpha 확인 | 0.05~0.1 범위에서 조정 |
| **입력이 뚝뚝 끊김** | 필터 부족 | `alpha_position` ↓ (0.85 → 0.7) | 더 강한 스무딩 |
| **입력 반응이 느림** | 필터 과다 | `alpha_position` ↑ (0.85 → 0.95) | 스무딩 약화 |
| **다관절 동시 운동 시 경로 왜곡** | 코리올리스 미보상 | `enable_coriolis_comp: true` | |
| **DEVIATION 안전 알림** | q_desired ↔ q_actual 편차 | `max_position_deviation` 완화 | 0.3 → 0.4 (주의) |
| **TIMEOUT 반복** | 입력 감지 실패 | `packet_timeout_ms` ↑ | 무선: 200~500ms |

### 5.2 튜닝 순서 권장

새로운 환경이나 작업에서 문제가 발생하면 다음 순서로 튜닝한다:

**Step 1: 안전 확보 — SOFT 프리셋에서 시작**
```
프리셋: SOFT, gain_scale: 1.0
```
기본 이동(W/S/A/D)이 정상 동작하는지 확인.

**Step 2: 게인 조정 — 강성/순응성 결정**
- `[`/`]` 키로 gain_scale 조정하여 원하는 강성 찾기
- 진동 발생 시 → gain_scale ↓ 또는 SOFT로 전환
- 프리셋 변경(`1`/`2`/`3`)으로 큰 단위 조정 후 scale로 미세 조정

**Step 3: 회전 동작 — posture_cost 확인**
- Roll/Pitch/Yaw 입력 시 의도치 않은 관절 커플링 여부 확인
- 문제 있으면 `ik.posture_cost` 조정 (1e-3 ~ 1e-1 범위)

**Step 4: 반응성 — max_joint_error 조정**
- 이동이 느리면 → 손목(J4-J5) 값 완화
- 토크 점프가 있으면 → 어깨(J0-J1) 값 타이트하게

**Step 5: 장시간 테스트 — soft_sync 확인**
- 1분 이상 연속 조작 후 발작/drift 여부 확인
- 문제 있으면 soft_sync alpha 조정 (0.03~0.15)

### 5.3 파라미터 간 상호작용 맵

```
Kp ─────────┬─── 강성/순응성
             │
             ├──→ max_joint_error와 함께 "최대 토크" 결정
             │    (tau_max ≈ Kp * max_joint_error)
             │
             └──→ Kd 비례 조정 필수 (ζ 유지)

posture_cost ──→ 관절 배치 안정성
             └──→ max_joint_error와 상호작용
                  (둘 다 높으면 sluggish)

soft_sync α ──→ IK drift 방지
             └──→ 너무 높으면 IK 반응 느림

alpha_filter ──→ 입력 스무딩
             └──→ 내부 파이프라인과 독립 (가장 먼저 조정 가능)
```

### 5.4 위험한 조합

| 조합 | 위험 | 대안 |
|------|------|------|
| 높은 Kp + 낮은 Kd | **진동/발진** | Kd를 Kp의 ~10%로 유지 |
| 높은 Kp + 큰 max_joint_error | **과도한 토크 점프** | max_joint_error 줄이기 |
| 높은 posture_cost + 작은 max_joint_error | **모션 거의 불가** | 둘 중 하나 완화 |
| soft_sync = 0 + 장시간 사용 | **IK drift → 발작** | alpha = 0.05 이상 |

---

## 6. 토크 및 관절 리밋

### 6.1 토크 리밋 (하드웨어)

```python
# urscript_manager.py / impedance_pd.script
TORQUE_LIMITS = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]  # Nm
```

URScript 내부와 Python 양쪽에서 클램핑한다 (이중 안전):
- **Python**: `np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)` → RTDE 전송
- **URScript**: 토크 포화 후 `direct_torque()` 호출

### 6.2 관절 속도 리밋

```yaml
safety.max_joint_vel: 1.0  # rad/s
```

어떤 관절이든 1.0 rad/s 초과 시 → 안전 시스템이 토크를 즉시 0으로.

### 6.3 위치 편차 리밋

```yaml
safety.max_position_deviation: 0.3  # rad (~17°)
```

q_desired와 q_actual의 편차가 0.3 rad 초과 시 → 안전 정지. IK가 실제와 너무 다른 목표를 보내는 것을 감지한다.

---

## 7. 참조

| 문서 | 내용 |
|------|------|
| [implementation_notes.md](implementation_notes.md) | 구현 과정 디버깅 기록 (URScript, RTDE, 레지스터) |
| [user_guide.md](user_guide.md) | 실행 방법, CLI, 키보드 조작, 트러블슈팅 |
| [manual.md](manual.md) | 원래 설계 (현재 구현과 다른 부분 있음, 참고용) |

| 소스 파일 | 핵심 내용 |
|----------|----------|
| `impedance_gains.py` | Kp/Kd 프리셋, ImpedanceController |
| `config/default.yaml` | 모든 튜닝 파라미터의 기본값 |
| `core/pink_ik.py` | IK 솔버, soft_sync, posture_cost |
| `core/exp_filter.py` | EMA + SLERP 필터 |
| `main.py` | 제어 루프, PD 토크 계산 |
| `urscript_manager.py` | RTDE 통신, 코리올리스 계산 |
| `torque_safety.py` | 5단계 안전 시스템 |
| `scripts/impedance_pd.script` | URScript 500Hz 토크 릴레이 |
