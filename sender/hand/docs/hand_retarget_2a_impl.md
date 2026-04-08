# [2A] Manus Skeleton → Fingertip IK → DG5F 구현 계획

> 작성일: 2026-04-08
> 기반: `retarget_dev/docs/retargeting_research.md`, `retarget_dev/docs/retarget_history.md` [2A]
> 선행: [1A] Ergonomics Direct Mapping (구현 완료)
> 목표: 다른 터미널에서 이 문서만으로 구현 가능하도록 작성

---

## 1. 왜 [2A]가 필요한가

### [1A]의 한계

| 문제 | 설명 |
|------|------|
| **형태 차이 미보정** | 인간 손과 DG5F 링크 길이가 다르지만 1A는 각도만 전달 → 실제 fingertip 위치가 다름 |
| **파지 정확도 부족** | 물체를 잡을 때 fingertip 위치가 중요한데, 각도 매핑만으로는 동일 위치 보장 불가 |
| **Spread 한계** | Manus Ergonomics의 spread 감지가 불안정 → DG5F spread 제어 부정확 |

### [2A]의 해결

| 개선 | 방법 |
|------|------|
| **Fingertip 위치 직접 매칭** | Manus skeleton의 fingertip 3D 좌표 → DG5F IK로 관절각 역산 |
| **링크 길이 자동 보정** | 캘리브레이션 시 human/robot bone length 비율 계산 → scale 적용 |
| **120Hz Skeleton** | manus_ros2의 raw_nodes (25 노드) 직접 사용 |

---

## 2. 알고리즘 개요

### 2.1 파이프라인

```
Manus ROS2 Topic (/manus_glove_*)
  ↓
ManusGlove.raw_nodes[] (25 nodes, world positions)
  ↓ Tip nodes 추출: [4, 9, 14, 19, 24]
  ↓ Wrist-relative 변환: p_tip_local = p_tip - p_wrist
  ↓ Scale 적용: p_target = p_tip_local * scale[finger]
  ↓
Per-finger DLS IK Solver
  ↓ Redundancy: Abduction = Manus Ergonomics (고정)
  ↓ 3-DOF IK: MCP, PIP, DIP
  ↓ Warm-start: q_prev (이전 프레임)
  ↓
Joint Limit Clamp → EMA Filter → DG5F JointState (20 rad)
```

### 2.2 핵심 수식: DLS (Damped Least Squares) IK

```
error = p_target - FK(q_current)
Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ · error
q_new = q_current + Δq
```

- `J`: 3×3 Jacobian (abduction 고정 후 3-DOF)
- `λ`: damping factor (0.01 ~ 0.1)
- 반복: 5~10 iterations per frame

### 2.3 Per-finger 구조

| Finger | Joint 1 (고정) | Joint 2 | Joint 3 | Joint 4 | IK 방식 |
|--------|---------------|---------|---------|---------|---------|
| **Thumb** | rj_1_1 (X, abd) = Ergo | rj_1_2 (Z, opp) | rj_1_3 (X, flex) | rj_1_4 (X, flex) | **3-DOF DLS** (비평면) |
| **Index** | rj_2_1 (X, abd) = Ergo | rj_2_2 (Y, MCP) | rj_2_3 (Y, PIP) | rj_2_4 (Y, DIP) | **3-DOF DLS** (평면) |
| **Middle** | rj_3_1 = Ergo | rj_3_2 | rj_3_3 | rj_3_4 | 동일 |
| **Ring** | rj_4_1 = Ergo | rj_4_2 | rj_4_3 | rj_4_4 | 동일 |
| **Pinky** | rj_5_1 = Ergo | rj_5_2 | rj_5_3 | rj_5_4 | 동일 |

---

## 3. Redundancy 해결 전략

### 3.1 문제

DG5F는 손가락당 4-DOF (abduction + 3 flexion). Fingertip 위치는 3D → **1-DOF 여유 (redundancy)**.
같은 fingertip 위치에 무한한 관절 조합 가능 → 어떤 해를 선택할 것인가?

### 3.2 전략 비교

| 전략 | 방법 | 구현 난이도 | 결과 품질 | 추천 |
|------|------|-----------|----------|------|
| **(a) Abduction 고정** | `abd = Manus_Ergo_Spread`; 나머지 3-DOF IK | ★★ 쉬움 | ★★★★ | **Phase 1 채택** |
| (b) DLS λ 감쇠 | 최소 노름 해 | ★★ 쉬움 | ★★ 부자연스러움 | 미사용 |
| (c) Minimum Change | `min ‖q - q_prev‖` | ★★★ 보통 | ★★★ 부드러움 | Phase 2 추가 |
| **(a+c) Hybrid** | abd=Ergo + 3-DOF DLS minimizing change | ★★★★ | ★★★★★ | **Phase 2 고도화** |

### 3.3 구현 순서 (구현 성공률 우선)

**Phase 1 — 기본 동작 확보 (전략 a)**
```
1. Abduction = Manus Ergonomics 값 그대로 사용 (1A에서 검증 완료)
2. 나머지 3-DOF = DLS IK (damped least squares)
3. Warm-start = 이전 프레임 q_prev
4. → 가장 단순하고 구현 성공률 높음
```

**Phase 2 — 고도화 (전략 a+c Hybrid)**
```
1. Phase 1 + minimum-change regularization 추가
2. DLS cost에 ‖q - q_prev‖² 항 추가
3. Thumb 전용 solver (비평면 축 → 3D DLS 필수)
4. → 자연스러움 + 부드러움 향상
```

---

## 4. 새 파일 구조

```
sender/hand/
├── core/                          # 기존 공통 인프라 (재사용)
│   ├── retarget_base.py           # HandRetargetBase ABC
│   ├── dg5f_config.py             # Joint names, limits
│   └── filters.py                 # EMAFilter
│
├── gen1a_ergo_direct/             # [1A] 기존 (변경 없음)
│   └── ergo_direct.py
│
├── gen2a_fingertip_ik/            # ★ 신규: [2A] Fingertip IK
│   ├── __init__.py
│   ├── fingertip_ik.py            # FingertipIKRetarget 클래스
│   ├── dg5f_fk.py                 # Pinocchio FK (retarget_dev에서 이식)
│   ├── per_finger_ik.py           # Per-finger DLS solver
│   └── scale_calibrator.py        # Human→Robot bone length 비율 계산
│
└── core/ 추가 예정
    └── dg5f_fk.py                 # gen2a에서 검증 후 core/로 승격
```

---

## 5. 파일별 구현 상세

### 5.1 `gen2a_fingertip_ik/dg5f_fk.py` — DG5F FK (Pinocchio)

```python
"""DG5F Forward Kinematics via Pinocchio.

retarget_dev/models/fingertip_ik/dg5f_fk.py에서 이식.
URDF 로딩 → fingertip positions, Jacobian 계산.
"""

class DG5FKinematics:
    def __init__(self, hand_side: str, urdf_path: str = None):
        # pinocchio.buildModelFromUrdf(urdf_path)
        # tip_frame_ids: [rl_dg_1_tip, rl_dg_2_tip, ..., rl_dg_5_tip]
        # palm_frame_id: rl_dg_palm

    def fingertip_positions(self, q: ndarray[20]) -> ndarray[5, 3]:
        """각 fingertip의 world position."""

    def fingertip_jacobian(self, q: ndarray[20], finger_idx: int) -> ndarray[3, 4]:
        """finger_idx 손가락의 3D position Jacobian (4 joints)."""

    def per_finger_fk(self, q_finger: ndarray[4], finger_idx: int) -> ndarray[3]:
        """단일 손가락 FK (4-DOF → 3D position)."""

    @property
    def q_min(self) -> ndarray[20]: ...
    @property
    def q_max(self) -> ndarray[20]: ...
```

### 5.2 `gen2a_fingertip_ik/per_finger_ik.py` — Per-finger DLS IK

```python
"""Per-finger Damped Least Squares IK solver.

한 손가락의 4-DOF 중 abduction 고정 후 3-DOF IK.
"""

class PerFingerIK:
    def __init__(self, fk: DG5FKinematics, finger_idx: int,
                 damping: float = 0.05, max_iter: int = 10,
                 tol: float = 1e-4):
        self._fk = fk
        self._finger_idx = finger_idx
        self._damping = damping
        self._max_iter = max_iter
        self._tol = tol

    def solve(self, p_target: ndarray[3], abd_angle: float,
              q_init: ndarray[3] = None) -> ndarray[4]:
        """
        Parameters
        ----------
        p_target : 목표 fingertip 위치 (wrist-relative, scaled)
        abd_angle : abduction 각도 (Manus Ergonomics에서)
        q_init : 초기 추정 (warm-start, 이전 프레임)

        Returns
        -------
        ndarray[4] : [abd, joint2, joint3, joint4] (radians)
        """
        q = np.zeros(4)
        q[0] = abd_angle  # 고정

        # q[1:4] = DLS iteration
        for _ in range(self._max_iter):
            p_current = self._fk.per_finger_fk(q, self._finger_idx)
            error = p_target - p_current
            if np.linalg.norm(error) < self._tol:
                break

            J = self._fk.fingertip_jacobian(q, self._finger_idx)[:, 1:]  # 3×3 (abd 제외)
            JJt = J @ J.T + self._damping**2 * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, error)
            q[1:4] += dq
            q[1:4] = np.clip(q[1:4], q_min[1:4], q_max[1:4])

        return q
```

### 5.3 `gen2a_fingertip_ik/scale_calibrator.py` — Scale 계산

```python
"""Human → Robot bone length ratio calibration.

Manus skeleton의 bone lengths와 DG5F URDF link lengths 비교.
"""

class ScaleCalibrator:
    def __init__(self, fk: DG5FKinematics):
        self._fk = fk
        self._robot_lengths = self._compute_robot_lengths()

    def calibrate(self, skeleton: ndarray[N, 7]) -> ndarray[5]:
        """
        Returns per-finger scale factors.
        scale[i] = robot_total_length[i] / human_total_length[i]
        """
        human_lengths = self._compute_human_lengths(skeleton)
        return self._robot_lengths / np.maximum(human_lengths, 1e-6)

    def _compute_robot_lengths(self) -> ndarray[5]:
        """DG5F URDF에서 각 finger의 총 링크 길이."""
        q_zero = np.zeros(20)
        tips = self._fk.fingertip_positions(q_zero)
        palm = self._fk.palm_position(q_zero)
        return np.array([np.linalg.norm(tips[i] - palm) for i in range(5)])

    def _compute_human_lengths(self, skeleton: ndarray[N, 7]) -> ndarray[5]:
        """Manus skeleton에서 각 finger의 wrist→tip 거리."""
        # Tip indices: [4, 9, 14, 19, 24] (Manus Raw Skeleton)
        # 또는 [4, 8, 12, 16, 20] (MediaPipe convention) → 확인 필요
        wrist = skeleton[0, :3]
        tip_indices = [4, 9, 14, 19, 24]  # Manus convention
        return np.array([np.linalg.norm(skeleton[i, :3] - wrist) for i in tip_indices])
```

### 5.4 `gen2a_fingertip_ik/fingertip_ik.py` — 메인 클래스

```python
"""[2A] Manus Skeleton → Fingertip IK → DG5F.

HandRetargetBase를 상속. manus_sender.py에서 --retarget fingertip-ik로 호출.
"""

class FingertipIKRetarget(HandRetargetBase):
    def __init__(self, hand_side: str = "right", damping: float = 0.05):
        super().__init__(hand_side)
        self._fk = DG5FKinematics(hand_side)
        self._solvers = [PerFingerIK(self._fk, i, damping) for i in range(5)]
        self._calibrator = ScaleCalibrator(self._fk)
        self._scale = np.ones(5)  # per-finger scale
        self._q_prev = np.zeros(20)
        self._ema = EMAFilter(alpha=0.4, size=20)

    def calibrate(self, skeleton: ndarray, ergonomics: ndarray):
        """Open-hand 캘리브레이션: bone length scale 계산."""
        self._scale = self._calibrator.calibrate(skeleton)

    def retarget(self, skeleton: ndarray, ergonomics: ndarray, **kwargs) -> ndarray[20]:
        """
        Parameters
        ----------
        skeleton : ndarray[N, 7] — Manus raw skeleton nodes
        ergonomics : ndarray[20] — Manus ergonomics (for abduction)
        """
        wrist = skeleton[0, :3]
        tip_indices = [4, 9, 14, 19, 24]
        q = np.zeros(20)

        for f in range(5):
            # 1. Target position (wrist-relative, scaled)
            p_human = skeleton[tip_indices[f], :3] - wrist
            p_target = p_human * self._scale[f]

            # 2. Abduction from Ergonomics
            abd_angle = ergonomics[f * 4]  # Spread joint

            # 3. Per-finger IK
            q_init = self._q_prev[f*4 : f*4+4]
            q_finger = self._solvers[f].solve(p_target, abd_angle, q_init[1:])
            q[f*4 : f*4+4] = q_finger

        # 4. Clamp + EMA
        q = np.clip(q, self._fk.q_min, self._fk.q_max)
        q = self._ema.filter(q)
        self._q_prev = q.copy()
        return q

    def get_method_name(self) -> str:
        return "2A-fingertip-ik"
```

---

## 6. manus_sender.py 수정

```python
# --retarget choices에 "fingertip-ik" 추가
parser.add_argument("--retarget", default="none",
                    choices=["none", "ergo-direct", "fingertip-ik"])

# 생성
if args.retarget == "fingertip-ik":
    from sender.hand.gen2a_fingertip_ik import FingertipIKRetarget
    retarget = FingertipIKRetarget(hand_side=hand_side)
    _retarget_uses_skeleton = True  # skeleton + ergonomics 둘 다 필요

# 호출
dg5f_q = retarget.retarget(skeleton=data.skeleton, ergonomics=data.joint_angles)
```

---

## 7. Manus Skeleton 노드 매핑

### Manus Raw Skeleton (25 nodes)

```
[0]     Wrist
[1-4]   Thumb:  CMC(1), MCP(2), IP(3), TIP(4)
[5-9]   Index:  MCP(5), PIP(6), DIP(7), TIP(8), ?(9)
[10-14] Middle: MCP(10), PIP(11), DIP(12), TIP(13), ?(14)
[15-19] Ring:   MCP(15), PIP(16), DIP(17), TIP(18), ?(19)
[20-24] Pinky:  MCP(20), PIP(21), DIP(22), TIP(23), ?(24)
```

> **주의**: ROS2 `ManusGlove.raw_nodes[]`의 노드 순서와 인덱스는
> 실제 데이터로 확인 필요. `ros2 topic echo`로 node_id / joint_type 확인.

### DG5F Tip Frame Names

```
Right: rl_dg_1_tip, rl_dg_2_tip, rl_dg_3_tip, rl_dg_4_tip, rl_dg_5_tip
Left:  ll_dg_1_tip, ll_dg_2_tip, ll_dg_3_tip, ll_dg_4_tip, ll_dg_5_tip
```

---

## 8. 캘리브레이션

### 8.1 Bone Length Scale (1회, 시작 시)

```
1. 사용자: 손을 완전히 편 상태 (open hand)
2. Manus skeleton에서 각 finger의 wrist→tip 거리 측정
3. DG5F URDF에서 각 finger의 wrist→tip 거리 계산 (FK at q=0)
4. scale[i] = robot_length[i] / human_length[i]
```

### 8.2 좌표계 정렬 (선택적)

```
1. SVD Procrustes alignment: R, t = argmin ‖R·human_tips + t - robot_tips‖²
2. 또는: wrist-relative 좌표만 사용 (정렬 불필요)
→ Phase 1에서는 wrist-relative만 사용 (단순화)
```

---

## 9. 의존성

| 패키지 | 용도 | 설치 |
|--------|------|------|
| pinocchio | URDF 로딩, FK, Jacobian | `pip install pin` (NOT `pip install pinocchio`) |
| numpy | 수치 연산 | 기존 |

### URDF 파일 위치

```
src/dg5f_ros2/dg5f_description/urdf/dg5f_right.urdf
src/dg5f_ros2/dg5f_description/urdf/dg5f_left.urdf
```

---

## 10. 구현 순서 (Step-by-step)

### Phase 1: 기본 동작 (전략 a — Abduction 고정)

```
Step 1: gen2a_fingertip_ik/dg5f_fk.py
        - retarget_dev/models/fingertip_ik/dg5f_fk.py 이식
        - per_finger_fk(), per_finger_jacobian() 추가

Step 2: gen2a_fingertip_ik/per_finger_ik.py
        - DLS solver 구현
        - Abduction 고정, 3-DOF IK

Step 3: gen2a_fingertip_ik/scale_calibrator.py
        - Bone length ratio 계산

Step 4: gen2a_fingertip_ik/fingertip_ik.py
        - FingertipIKRetarget 클래스 통합

Step 5: manus_sender.py에 --retarget fingertip-ik 추가

Step 6: 단위 테스트 (mock skeleton → IK → joint angles 확인)
```

### Phase 2: 고도화 (전략 a+c Hybrid)

```
Step 7: per_finger_ik.py에 minimum-change regularization 추가
        Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ error + α(q_prev - q)

Step 8: Thumb 전용 solver 검증 (비평면 X-Z-X-X 축)

Step 9: SVD 좌표 정렬 (선택적)

Step 10: 성능 벤치마크 (30Hz+ 확인)
```

---

## 11. 예상 성능

| 항목 | [1A] Ergo Direct | [2A] Fingertip IK |
|------|------------------|-------------------|
| **Fingertip 위치 정확도** | ★★ | ★★★★ |
| **Spread 정확도** | ★★ (Ergo 한계) | ★★★ (Ergo + IK) |
| **파지 성공률** | ★★ | ★★★★ |
| **연산 속도** | < 1ms | 5~10ms |
| **구현 난이도** | ★★ | ★★★★ |
| **Pinocchio 의존** | 불필요 | 필수 |

---

## 12. [3A] Multi-Cost로의 확장 경로

[2A]의 fingertip IK가 [3A]의 C1 (position cost) 기반이 됨:

```
[3A] = w1·C1_fingertip_pos     ← [2A]의 FK + IK 재사용
     + w2·C2_bone_direction
     + w3·C3_angle_prior       ← [1A]의 ergonomics 재사용
     + w4·C4_smoothness        ← EMAFilter 또는 q_prev regularization
     + w5·C5_self_collision    ← 신규
     + w6·C6_joint_limit       ← dg5f_config.py 재사용
```

---

## 13. 핵심 구현 포인트 (체크리스트)

- [ ] Manus Raw Skeleton 노드 인덱스 실제 데이터로 확인 (tip_indices)
- [ ] DG5F URDF tip frame 이름 확인 (rl_dg_F_tip 존재 여부)
- [ ] Pinocchio per-finger Jacobian 추출 방법 확인 (전체 20-DOF Jacobian에서 4열 추출)
- [ ] Thumb IK: 비평면 축 (X-Z-X-X) → 3D DLS 필수, 평면 IK 불가
- [ ] Damping factor λ 튜닝: 너무 크면 느림, 너무 작면 불안정
- [ ] scale 캘리브레이션: 한 번만 실행 (매 프레임 X)
- [ ] `retarget(skeleton=, ergonomics=)` 시그니처 — HandRetargetBase의 kwargs 인터페이스 준수
