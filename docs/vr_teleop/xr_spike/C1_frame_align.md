# Phase C · Unit C1 — WebXR wrist → 로봇 base_link relative motion

## 목적

WebXR `local-floor` 좌표계의 wrist pose 를 robot 의 `base_link` 좌표계의 target TCP pose 로 매핑. **상대 motion** — 사용자 손목 origin 과 robot TCP origin 동시 capture 후 delta 만 누적.

## 출처

- xr_teleop: [`xr_teleop/scripts/run_teleop_ur10e_ws.py:307-358`](../../../../xr_teleop/scripts/run_teleop_ur10e_ws.py#L307) — `RECALIBRATE` flag + delta_p / R_delta 처방
- 검증: xr_teleop Week 4 U5+ (commit `02199d7+`) — relative motion 모드로 init pose 불일치 문제 해결, 사용자 시각 검증 통과

## 왜 relative motion?

xr_teleop Week 4 발견: UR10e init pose (T-pose 예: `[0.0, -1.18, 2.06, -0.88, 1.50, 0.0]`) 와 사용자 손목 시작 위치 가 무관 → **절대 매핑** 시 robot 이 init pose 에서 wrist 로 jump. 위험 + 사용자 직관 불일치.

**상대 motion** 으로:
1. 'r' 시점에 robot TCP pose 와 사용자 wrist 동시 capture (origin pair)
2. 그 이후 사용자 wrist delta 만 robot origin 에 더함 — `target_pos = origin_robot_pos + (curr_wrist - origin_wrist) * scale`
3. 회전 1:1: `target_R = (curr_wrist_R @ origin_wrist_R^T) @ origin_robot_R`
4. `c` (immediate recalibrate) / `p` (resume 시 자동 recalibrate) 로 jump 회피

## 신규 파일

`src/teleop_dev/sender/arm/xr_frame_align.py`:
- `FrameOrigin` dataclass — origin pair (user_pose, robot_pos, robot_quat)
- `XRRelativeFrameAligner` — calibrate() + apply()
- helper: `_quat_wxyz_to_rotmat`, `_rotmat_to_quat_wxyz`, `_is_valid_pose`

## API

```python
from sender.arm.xr_frame_align import XRRelativeFrameAligner

aligner = XRRelativeFrameAligner(scale=1.0)

# 'r' 키 시점: robot 의 query_pose 응답 + bridge.right_arm_pose 둘 다 캡처
aligner.calibrate(user_pose_4x4, robot_pos_3, robot_quat_wxyz_4)

# 매 loop:
target_pos, target_quat = aligner.apply(curr_user_pose_4x4)
# calibrate 전: zeros / last_target 반환 (안전 hold)
# invalid user_pose: last_target 유지 (jump 회피)
```

## 검증 (단위)

```bash
python3 -c "
import numpy as np
from sender.arm.xr_frame_align import XRRelativeFrameAligner, _quat_wxyz_to_rotmat, _rotmat_to_quat_wxyz, _is_valid_pose

# 1. quat <-> rotmat round-trip
for _ in range(5):
    q = np.random.randn(4); q /= np.linalg.norm(q)
    if q[0] < 0: q = -q
    R = _quat_wxyz_to_rotmat(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
    q2 = _rotmat_to_quat_wxyz(R)
    if q2[0] < 0: q2 = -q2
    assert np.allclose(q, q2, atol=1e-10)

# 2. calibrate identity → robot_origin 반환
al = XRRelativeFrameAligner(1.0)
user = np.eye(4); user[:3, 3] = [0.5, 0.2, 1.4]
al.calibrate(user, np.array([0,-0.4,0.4]), np.array([0,0.707,0.707,0]))
pos, _ = al.apply(user)
assert np.allclose(pos, [0,-0.4,0.4])

# 3. +0.1m x delta
user_now = user.copy(); user_now[:3,3] += [0.1,0,0]
pos, _ = al.apply(user_now)
assert np.allclose(pos, [0.1,-0.4,0.4])

# 4. scale 0.5
al2 = XRRelativeFrameAligner(0.5)
al2.calibrate(user, np.array([0,-0.4,0.4]), np.array([0,0.707,0.707,0]))
pos, _ = al2.apply(user_now)
assert np.allclose(pos, [0.05,-0.4,0.4])

# 5. invalid user_pose → last_target hold
pos_bad, _ = al.apply(np.zeros((4,4)))
assert np.allclose(pos_bad, pos)
print('xr_frame_align OK')
"
```

위 5 가지 단위 테스트 모두 본 PC 에서 통과 확인.

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/arm/xr_frame_align.py`
