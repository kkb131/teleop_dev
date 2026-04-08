# [2A] Fingertip IK 디버그 로그

> 작성일: 2026-04-08
> 증상: spread 포즈에서 DG5F가 spread하지 않음. 역방향 굽힘/주먹 형태 발생.

---

## 증상 상세

| Finger | 기대 (spread) | 실제 출력 | 문제 |
|--------|-------------|----------|------|
| Index | MCP≈0, PIP≈0, DIP≈0 | MCP 0.00, PIP **-1.57**, DIP **-0.67** | 손등 방향 역굽힘 |
| Ring | MCP≈0, PIP≈0, DIP≈0 | MCP **1.79**, PIP **1.54**, DIP **0.94** | 주먹 형태 |
| Thumb | 소폭 abd | 예측 불가 | 비정상 |

---

## 의심 원인 목록

### Bug #1: Manus Skeleton Tip 인덱스 오류 (CRITICAL)

**위치**: `scale_calibrator.py` L15, `fingertip_ik.py`에서 import

```python
MANUS_TIP_INDICES = [4, 8, 12, 16, 20]  # ← 21노드(MediaPipe) 기준
```

**문제**: Manus ROS2는 **25노드** 스켈레톤 제공. 실제 tip 인덱스:
```
Thumb TIP  = [4]   ← 맞음 (5노드 체인: 0-4)
Index TIP  = [9]   ← [8]은 TIP가 아닌 다른 노드
Middle TIP = [14]  ← [12]는 DIP
Ring TIP   = [19]  ← [16]는 MCP
Pinky TIP  = [24]  ← [20]은 MCP
```

**올바른 값**: `[4, 9, 14, 19, 24]`

**영향**: 잘못된 노드의 위치를 읽으므로 IK 타겟이 실제 fingertip보다 짧음 → 역굽힘

**검증 방법**:
```python
# Manus ROS2 데이터에서 노드별 위치 확인
ros2 topic echo /manus_glove_0 --once
# raw_nodes[].node_id, joint_type, chain_type 확인
# 또는:
python3 -c "
from sender.hand.manus_reader_ros2 import ManusReaderROS2
import numpy as np
reader = ManusReaderROS2('right', verbose=True)
reader.connect()
import time; time.sleep(3)
data = reader.get_hand_data()
if data and data.skeleton is not None:
    skel = data.skeleton
    print(f'Skeleton nodes: {skel.shape[0]}')
    wrist = skel[0, :3]
    for i in range(skel.shape[0]):
        dist = np.linalg.norm(skel[i, :3] - wrist)
        print(f'  Node[{i:2d}]: dist={dist:.4f}m  pos=[{skel[i,0]:.4f}, {skel[i,1]:.4f}, {skel[i,2]:.4f}]')
reader.disconnect()
"
```

---

### Bug #2: 좌표계 불일치 (CRITICAL)

**위치**: `fingertip_ik.py` L93-94

```python
p_human = skeleton[tip_idx, :3] - wrist   # Manus world frame, wrist-relative
p_target = p_human * self._scale[f]       # 스케일된 wrist-relative 위치
```

**문제**:
- `p_target`은 **Manus wrist 기준** 상대 좌표
- IK solver의 FK (`finger_tip_position`)는 **DG5F URDF world frame** 좌표 반환
- 두 좌표계의 **원점, 축 방향이 다름**

**영향**: IK solver가 전혀 다른 위치를 타겟으로 받음 → 비정상 관절각

**검증 방법**:
```python
# DG5F FK의 fingertip 위치와 Manus skeleton 위치 비교
from sender.hand.gen2a_fingertip_ik.dg5f_fk import DG5FKinematics
import numpy as np

fk = DG5FKinematics('right')
q_zero = np.zeros(20)
tips = fk.fingertip_positions(q_zero)
palm = fk.palm_position(q_zero)
print(f'DG5F palm: {palm}')
print(f'DG5F tips at q=0:')
for i, name in enumerate(['Thumb','Index','Middle','Ring','Pinky']):
    rel = tips[i] - palm
    print(f'  {name}: world={tips[i]}, palm-rel={rel}, dist={np.linalg.norm(rel):.4f}')

# 그 후 Manus skeleton의 wrist-relative tip 위치와 비교
# 크기 오더(0.01m vs 0.1m 등)와 축 방향 확인
```

---

### Bug #3: Dead Code (잔여 디버그 코드)

**위치**: `fingertip_ik.py` L102-103, L112

```python
# L102: * 0 → 아무 효과 없음
p_target=p_target + self._fk.fingertip_positions(np.zeros(20))[0] * 0,

# L112: wrist * 0 → 아무 효과 없음
self._last_errors[f] = np.linalg.norm(p_target - (p_achieved - wrist * 0))
```

**영향**: 직접적 버그는 아니지만 의도한 좌표 변환이 빠져있음을 시사

**검증**: 코드 리뷰로 확인 완료. 삭제 필요.

---

### Bug #4: Scale 기준점 불일치

**위치**: `scale_calibrator.py` L47-53

```python
def _compute_robot_lengths(self):
    tips = self._fk.fingertip_positions(q_zero)
    wrist = np.zeros(3)  # ← URDF world origin
    return np.array([np.linalg.norm(tips[i] - wrist) for i in range(5)])
```

**문제**: DG5F의 "wrist"는 world origin(0,0,0)이 아닌 **palm frame**이어야 함

**검증 방법**: 위 Bug #2 검증 코드에서 `palm` vs `np.zeros(3)` 차이 확인

---

### Bug #5: Manus→DG5F 축 방향 매핑 미확인

**문제**: Manus skeleton의 X/Y/Z 축과 DG5F URDF의 X/Y/Z 축이 다를 수 있음
- Manus: 보통 X=right, Y=up, Z=forward (OpenGL convention)
- DG5F URDF: 축 방향은 URDF 정의에 따라 다름

**영향**: spread 포즈에서 fingertip이 Y축으로 벌어지는데, DG5F에서는 다른 축이 될 수 있음

**검증 방법**:
```python
# Manus skeleton의 spread 포즈에서 각 tip의 XYZ 좌표 변화 관찰
# X가 변하는지, Y가 변하는지, Z가 변하는지 확인
# 그리고 DG5F FK에서 같은 방향에 해당하는 축 확인
```

---

## 수정 우선순위

| 순서 | Bug # | 작업 | 예상 효과 |
|------|-------|------|----------|
| 1 | #1 | Tip 인덱스 수정: `[4,9,14,19,24]` | 올바른 fingertip 위치 사용 |
| 2 | #2+#4 | 좌표계 통일 (palm-relative) | IK 타겟이 올바른 프레임 |
| 3 | #3 | Dead code 제거 | 코드 정리 |
| 4 | #5 | 축 매핑 검증 + 회전 행렬 적용 | 방향 정확도 |

---

## 수정 전 검증 스크립트

위 검증 코드들을 하나의 스크립트로 통합:

```bash
python3 -m sender.hand.tests.test_2a_debug --hand right
```

스크립트가 출력해야 할 것:
1. Manus skeleton 노드 수 + 각 노드의 wrist-relative 거리
2. DG5F FK fingertip 위치 (palm-relative)
3. 두 좌표계의 크기 오더 비교
4. Spread 포즈에서 어떤 축이 변하는지

---

## 수정 후 검증

```bash
# spread 포즈에서 모든 flexion 관절이 ≈ 0° 인지 확인
python3 -m sender.hand.manus_sender --retarget fingertip-ik --sdk-mode ros2 \
  --target-ip <IP> --calibrate
# receiver에서: MCP/PIP/DIP ≈ 0, Spread만 변화
```

---

## Bug #6: Fist 시 역방향 굽힘 (2026-04-08)

### 증상

Spread는 동작하지만, fist 시 Index/Middle의 PIP/DIP가 **음수(역방향)** → 손등 뒤로 꺾임.

### 원인

URDF에서 PIP/DIP 관절 한계가 `(-π/2, +π/2)` → 음수도 허용.
DLS IK solver는 최소 노름 해를 찾으면서 음수 방향으로 수렴할 수 있음.
실제 인간 손가락은 **양수 방향(fist)으로만** 굽혀야 함.

### 해결책 목록

| 방안 | 방법 | 난이도 | 효과 | 상태 |
|------|------|--------|------|------|
| **A. Anatomical constraints** | IK solver에 해부학적 제약 추가: Index~Pinky MCP/PIP/DIP ≥ 0 | ★★ | ★★★★ | **적용됨** |
| B. Posture prior | IK cost에 `‖q - q_prior‖²` 항 추가. q_prior = ergonomics 기반 추정 | ★★★ | ★★★★★ | Phase 2 |
| C. PIP/DIP coupling | DIP = α × PIP (α≈0.6~0.8). 자유도 3→2로 감소 → IK 안정화 | ★★★ | ★★★★ | Phase 2 |
| D. Ergonomics fallback | IK 수렴 실패 시 1A(ergo-direct)로 폴백 | ★★ | ★★★ | Phase 2 |

### A. Anatomical constraints (적용됨)

`per_finger_ik.py`에서 Index~Pinky의 MCP/PIP/DIP 하한을 0으로 설정:

```python
if finger_idx >= 1:  # Index ~ Pinky
    self._q_min[1] = max(self._q_min[1], 0.0)  # MCP ≥ 0
    self._q_min[2] = max(self._q_min[2], 0.0)  # PIP ≥ 0
    self._q_min[3] = max(self._q_min[3], 0.0)  # DIP ≥ 0
```

Thumb(idx=0)은 비평면 축(X-Z-X-X)이므로 기존 URDF 한계 유지.

### B. Posture prior (Phase 2 예정)

IK cost를 수정:
```
cost = ‖p_target - FK(q)‖² + λ_prior · ‖q - q_prior‖²
```
- `q_prior`: Manus ergonomics에서 추정한 관절각 (1A의 ErgoDirectRetarget 출력)
- 효과: IK가 "인간다운" 자세를 선호하게 유도
- DexPilot/AnyTeleop에서 사용하는 방법

### C. PIP/DIP coupling (Phase 2 예정)

인간 손가락의 해부학적 특성:
- DIP ≈ 0.6~0.8 × PIP (동시에 굽힘)
- 자유도를 3-DOF → 2-DOF로 줄임:
  ```
  q = [abd, MCP, PIP]  # DIP = 0.7 * PIP (자동)
  ```
- IK가 더 안정적이고 자연스러운 해를 찾음

### D. Ergonomics fallback (Phase 2 예정)

IK 수렴 실패 (error > threshold) 시:
```python
if np.linalg.norm(error) > fallback_threshold:
    q_fallback = ergo_direct.retarget(ergonomics=ergo)
    return q_fallback  # 1A 결과로 대체
```
- 안전망 역할: IK가 비정상일 때 1A가 보완

---

## Bug #7: Index 안 움직임 + 캘리브레이션 시 전체 손가락 고장 (2026-04-08)

### 증상
- Index만 모든 관절 0° (음수 해만 도출 → anatomical constraint로 clamp)
- `--calibrate` 적용 시 5손가락 모두 비정상 동작

### 근본 원인 3가지

**A1. p_target 프레임 오류 (CRITICAL)**
```python
# 이전 (잘못됨):
p_target = R @ p_human_local * scale + palm_offset  # palm_offset 혼합!

# 수정:
p_target = R @ p_human_local * scale  # URDF origin = wrist = [0,0,0], offset 불필요
```
- URDF world origin이 wrist mount point (= [0,0,0])
- FK 출력도 wrist frame
- palm_offset을 더하면 프레임 불일치 → IK가 엉뚱한 위치 추적

**A2. SVD reflection (MAJOR)**
```python
# 이전: det(R)이 -1일 수 있음 → 축 반전
d = np.linalg.det(Vt.T @ U.T)
D = np.diag([1, 1, d])  # d=-1이면 reflection!

# 수정: np.sign(d)로 강제 +1
D = np.diag([1, 1, np.sign(d)])
```

**B1. Scale 기준점 불일치**
```python
# 이전: palm-relative distance
palm = self._fk.palm_position(q_zero)
return [norm(tips[i] - palm) ...]

# 수정: wrist-relative distance (URDF origin = wrist)
return [norm(tips[i]) ...]  # tips already wrist-relative
```

**B2. SVD 입력 프레임 불일치**
```python
# 이전: human=wrist-rel, robot=palm-rel → 다른 origin
robot_tips = fk.fingertip_positions(q_zero) - palm_offset

# 수정: 둘 다 wrist-relative
robot_tips = fk.fingertip_positions(q_zero)  # already wrist-rel
```

### 적용 결과
- Spread: MCP/PIP/DIP ≈ 0° (정상)
- Fist: Index MCP=46°, PIP=26.5° (양수 — 정방향 굽힘)
- det(R) = +1.0 (pure rotation, no reflection)
- Mean error: 4.2mm (spread), 18.2mm (fist)
