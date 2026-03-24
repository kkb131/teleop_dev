# UR10e Teleop Parameter Tuning Guide

Pink QP IK + 임피던스 제어 파라미터 분석 및 최적값 제안.
목표: **느리고 부드러운(soft) 원격조종** 느낌.

---

## 1. Pink QP IK 파라미터

### 현재값 vs 권장값

| 파라미터 | 현재 (임피던스) | 현재 (어드미턴스) | 권장 (soft teleop) | 근거 |
|----------|----------------|------------------|-------------------|------|
| `position_cost` | 1.0 | 1.0 | **1.0** | Pink 공식 예제 표준값. RelaxedIK(RSS 2018)도 position을 가장 높은 가중치로 설정 |
| `orientation_cost` | 0.5 | 0.5 | **0.3** | 낮추면 오리엔테이션 추적이 느슨해져 관절 속도 스파이크 완화. 0.3~0.5 범위가 텔레옵에 적합 |
| `posture_cost` | 1e-2 | 1e-3 | **임피던스: 1e-2, 어드미턴스: 1e-3** | 6-DOF 로봇은 진정한 nullspace가 없음. 높은 값은 EE 추적과 경쟁. 임피던스는 토크 안정성 위해 약간 높게 |
| `damping` | 1e-12 | 1e-12 | **1e-6** | 현재값은 사실상 0. 특이점 근처에서 velocity 급증 위험. 1e-6은 추적 정확도 손실 없이 안정성 확보 |
| `soft_sync_alpha` | 0.05 | 0.05 | **임피던스: 0.08, 어드미턴스: 0.05** | 임피던스는 PD 컨트롤러가 별도 추적 보정하므로 IK drift 허용도 높음. 0.08로 올리면 drift 빠르게 수렴 |

### damping 값 상세 분석

```
QP 목적함수: min Σ wᵢ·‖Jᵢ·Δq - eᵢ‖² + λ·‖Δq‖²
                                               ↑ damping
```

| damping (λ) | 효과 | 적합한 상황 |
|-------------|------|-------------|
| 1e-12 | 거의 없음 | 특이점 회피 불필요한 환경 |
| **1e-6** | 수치적 안정화 | **텔레옵 (권장)** |
| 1e-4 | 약한 속도 억제 | 고속 모션 플래닝 |
| 1e-3 | 강한 속도 억제 | 추적 정확도 불필요 시 |

**근거**: Pink 공식 문서는 1e-12 기본값이지만, 이는 정밀 추적용. 텔레옵에서는 사람 입력 자체가 부정확하므로 1e-6 수준의 damping이 안전하고, 특이점 근처 velocity spike를 효과적으로 억제.

### orientation_cost 낮추는 이유

```
오리엔테이션 에러 → SE3 log map → 6D twist 중 angular 부분
→ UR10e wrist joints (J4,J5,J6) 가 주로 담당
→ wrist joints는 관성 낮고 velocity limit 먼저 도달
→ orientation_cost 높으면 QP가 큰 angular velocity 요구 → safety clipping
→ soft teleop에서는 0.3이면 충분히 추적하면서 부드럽게 수렴
```

---

## 2. EMA 필터 파라미터

### 현재값 vs 권장값

| 파라미터 | 현재값 | 권장 (soft teleop) | 근거 |
|----------|--------|-------------------|------|
| `alpha_position` | 0.85 | **0.5~0.6** | 0.85는 매우 반응적. soft 느낌을 위해 낮추면 입력이 부드럽게 스무딩됨 |
| `alpha_orientation` | 0.85 | **0.5~0.6** | 동일. slerp alpha 낮추면 회전이 자연스럽게 보간 |

### alpha 값에 따른 체감

```
alpha = 0.9  : 거의 raw input, 떨림 그대로 전달
alpha = 0.85 : 약간 스무딩 (현재)
alpha = 0.7  : 중간 스무딩
alpha = 0.5  : 부드러운 느낌, ~100ms 지연 (50Hz 기준)
alpha = 0.3  : 매우 부드럽지만 느린 반응 (~200ms)
```

**텔레옵 문헌 참고**:
- Haptic 텔레옵 논문들: α=0.3~0.5 (높은 latency 허용)
- 키보드/조이스틱 텔레옵: α=0.5~0.7 (이산 입력 스무딩)
- 실시간 서보: α=0.8~0.95 (최소 latency)

---

## 3. 임피던스 제어 게인

### 현재 프리셋 분석

UR10e 관절 관성 (근사치):

| 관절 | UR 사이즈 | 최대 토크 | 관성 (I_eff) | 비고 |
|------|----------|----------|-------------|------|
| J0 (shoulder_pan) | Size 4 | 330 Nm | ~10 kg·m² | 전체 팔 회전 |
| J1 (shoulder_lift) | Size 4 | 330 Nm | ~8 kg·m² | 중력 부하 최대 |
| J2 (elbow) | Size 3 | 150 Nm | ~3 kg·m² | |
| J3 (wrist1) | Size 2 | 56 Nm | ~0.5 kg·m² | |
| J4 (wrist2) | Size 2 | 56 Nm | ~0.3 kg·m² | |
| J5 (wrist3) | Size 1 | 28 Nm | ~0.1 kg·m² | |

### 권장 프리셋: TELEOP_SOFT (신규)

현재 SOFT(Kp=400~25)는 산업용 기준으로 여전히 stiff. COMPLIANT(Kp=20~1.25)는 중력보상 없이는 처짐. 그 중간:

```python
"TELEOP_SOFT": GainPreset(
    kp=[120.0, 120.0, 60.0, 30.0, 15.0, 8.0],   # Nm/rad
    kd=[28.0,  28.0,  14.0, 7.0,  3.5,  1.8],    # Nm·s/rad
)
```

**설계 근거**:

1. **Kp 결정**: 현재 SOFT의 30% 수준
   - J0,J1: 120 Nm/rad → ~0.5 rad (약 30도) 변위에서 최대 토크의 약 18%
   - 사람이 밀었을 때 "스프링 같은" 느낌이면서 위험하지 않은 수준

2. **Kd 결정**: `Kd = 2 * ζ * √(Kp * I_eff)` 공식 사용
   - ζ = 0.7 (약간 underdamped → 자연스러운 느낌)
   - J0: Kd = 2 × 0.7 × √(120 × 10) ≈ 48 → 28로 낮춤 (더 soft)
   - 의도적으로 이론값보다 낮게: underdamped 느낌이 텔레옵에서 더 자연스러움

3. **Kd/Kp 비율**: ~0.23
   - 일반 로봇 제어: 0.1~0.3
   - 텔레옵 문헌: 0.2~0.4 (더 높은 댐핑)
   - 0.23은 부드러우면서도 진동 억제하는 중간값

### 프리셋 비교표

| 프리셋 | Kp (J0) | Kd (J0) | Kd/Kp | 체감 | 용도 |
|--------|---------|---------|-------|------|------|
| STIFF | 800 | 80 | 0.10 | 단단함 | 정밀 작업 |
| MEDIUM | 600 | 64 | 0.11 | 보통 | 일반 텔레옵 |
| SOFT | 400 | 48 | 0.12 | 약간 유연 | 현재 기본 |
| **TELEOP_SOFT** | **120** | **28** | **0.23** | **부드러운 스프링** | **soft 텔레옵 (권장)** |
| COMPLIANT | 20 | 8 | 0.40 | 매우 유연 | 핸드 가이딩 |
| GRAVITY_COMP | 0 | 2 | ∞ | 자유 | 교시 |

---

## 4. 안전 파라미터

### 현재값 vs 권장값 (soft teleop)

| 파라미터 | 현재 (임피던스) | 현재 (어드미턴스) | 권장 (soft teleop) | 근거 |
|----------|----------------|------------------|-------------------|------|
| `max_joint_vel` | 1.0 rad/s | 0.5 rad/s | **0.4 rad/s** | ISO/TS 15066: TCP 250mm/s. UR10e 팔길이 1.3m → 0.4 rad/s ≈ 520mm/s (어깨). wrist는 더 느림 |
| `max_ee_velocity` | 0.2 m/s | 0.1 m/s | **0.12 m/s** | soft 느낌 유지하면서 실용적 속도. 0.1은 너무 느릴 수 있음 |
| `max_position_deviation` | 0.3 rad | - | **0.25 rad** | soft 게인에서 deviation 발생 확률 높음. 0.25로 약간 조임 |
| `packet_timeout_ms` | 2000 ms | 200 ms | **유지** | 임피던스: 키보드 이벤트 기반이라 2000 적절. 어드미턴스: 연속 서보라 200 적절 |

---

## 5. 종합 권장 설정 (Impedance Soft Teleop)

```yaml
# config/default.yaml — soft teleop 최적화
control:
  frequency_sim: 50
  frequency_rtde: 125

filter:
  alpha_position: 0.55      # 부드러운 스무딩 (현재 0.85)
  alpha_orientation: 0.55   # 부드러운 회전 보간

ik:
  position_cost: 1.0
  orientation_cost: 0.3     # 낮춰서 wrist velocity spike 방지 (현재 0.5)
  posture_cost: 1.0e-2
  damping: 1.0e-6           # 특이점 안정화 (현재 1e-12)
  soft_sync_alpha: 0.08     # 약간 빠른 drift 보정 (현재 0.05)

safety:
  packet_timeout_ms: 2000
  max_joint_vel: 0.4        # 느린 모션 (현재 1.0)
  max_position_deviation: 0.25
  max_ee_velocity: 0.12     # 느린 EE 속도 (현재 0.2)
  workspace:
    x: [-0.8, 0.8]
    y: [-0.8, 0.8]
    z: [0.05, 1.2]

impedance:
  default_preset: "TELEOP_SOFT"   # 신규 프리셋 (현재 SOFT)
  gain_scale_range: [0.25, 2.0]
  enable_coriolis_comp: true
  max_joint_error: [0.10, 0.10, 0.12, 0.18, 0.18, 0.2]
```

---

## 6. 참고 문헌 및 출처

| 출처 | 내용 | 관련 파라미터 |
|------|------|-------------|
| Pink 공식 문서 (github.com/kevinzakka/pink) | FrameTask/PostureTask 가중치 예제 | position_cost=1.0, orientation_cost=0.5~0.75 |
| RelaxedIK (Rakita et al., RSS 2018) | 텔레옵용 QP IK, position:orientation = 12:7 | orientation_cost 비율 ~0.58 |
| mink (MuJoCo 기반 Pink 포크) | UR5e 예제 | position_cost=1.0, damping=1e-4 |
| ISO/TS 15066 | 협동로봇 안전 속도 기준 | TCP ≤ 250mm/s |
| UR10e 데이터시트 | 관절별 최대 토크, 관성 | 게인 스케일링 근거 |
| Impedance Control Tutorial (Hogan 1985) | `b = 2ζ√(kI)` | Kd 결정 공식 |
| Villani & De Schutter (2016), Force Control | 임피던스 제어 damping ratio | ζ = 0.7~1.0 |

---

## 7. 튜닝 순서 (실제 로봇 테스트 시)

1. **먼저 IK 파라미터** (damping, orientation_cost) → sim에서 확인 가능
2. **필터 alpha** → sim에서 체감 확인
3. **임피던스 게인** → RTDE 모드에서 TELEOP_SOFT 프리셋 테스트
4. **안전 파라미터** → 마지막. 게인 안정 확인 후 조절

각 단계에서 한 파라미터만 변경하고, 3~5회 반복 테스트 후 다음으로 넘어갈 것.
