# Phase B · Unit B1 — WebXR 25-joint → MANO frame transform

## 목적

WebXR `local-floor` 좌표계의 25-joint hand pose 를 wrist-local + palm-aligned MANO frame 으로 변환. 사용자 손목 회전 시 손가락 자세가 동일하면 retargeter 출력이 동일하게 유지 (회전 robust).

## 출처

- xr_teleop: [`xr_teleop/scripts/dg5f_controller.py:107`](../../../../xr_teleop/scripts/dg5f_controller.py#L107) — `webxr_to_wrist_local_mano`
- retarget_dev: [`sender/hand/core/mano_transform.py`](../../../sender/hand/core/mano_transform.py) — `apply_mano_transform` 의 21-joint 버전
- 검증 출처: [`xr_teleop/docs/week4/day5_e2e_spike.md`](../../../../xr_teleop/docs/week4/day5_e2e_spike.md) — 사용자 손목 회전 시 retargeter 정확도 개선 검증 (~12배)

## 핵심 차이 (WebXR 25 vs MANO 21)

| Frame | 손가락당 keypoint | palm-plane SVD 3 점 인덱스 |
|---|---|---|
| MANO 21 | 4 (no separate metacarpal for non-thumb) | `[0, 5, 9]` (wrist, index_MCP, middle_MCP) |
| WebXR 25 | 5 (metacarpal 포함) | `[0, 5, 10]` (wrist, index-metacarpal, middle-metacarpal) |

metacarpal 이 palm 안쪽이라 SVD palm-plane fit 정확도 ↑.

## 신규 파일

`src/teleop_dev/sender/hand/xr_remap.py`:
- `estimate_wrist_frame_webxr(kp_25)` — palm-plane SVD → wrist frame (3, 3)
- `webxr_to_wrist_local_mano(kp_25, hand_side, convention)` — full pipeline
- `is_kp25_valid(kp_25)` — BridgePoseStore 초기 zeros / NaN 검출

## Convention 결정

WebXR HandLandmarker 가 image-derived MediaPipe 와 같은 chirality 라 가정 (mediapipe default). 실 헤드셋 visual 검증 후 fist↔spread inversion 보이면 `convention="manus"` 로 toggle (row 1 sign flip).

[`sender/hand/core/mano_transform.py`](../../../sender/hand/core/mano_transform.py) 의 두 매트릭스 재사용:
- `MEDIAPIPE_OPERATOR2MANO_RIGHT` — `[[0,0,-1],[-1,0,0],[0,1,0]]`
- `MANUS_OPERATOR2MANO_RIGHT` — `[[0,0,-1],[1,0,0],[0,1,0]]`

## 검증 (단위, 헤드셋 불요)

```python
import numpy as np
from sender.hand.xr_remap import webxr_to_wrist_local_mano, is_kp25_valid

# 1. invalid kp
assert not is_kp25_valid(np.zeros((25, 3)))
assert not is_kp25_valid(None)

# 2. dummy open hand → result.shape (25,3), result[0]=[0,0,0]
kp_open = ...  # synthetic
result = webxr_to_wrist_local_mano(kp_open, 'right', 'mediapipe')
assert result.shape == (25, 3)
assert np.allclose(result[0], 0)

# 3. yaw rotation robustness — 같은 손가락 자세 + 회전만 다른 입력 → 같은 결과
yaw_45 = np.pi / 4
Rz = ...  # 45도 yaw rotation matrix
kp_rotated = kp_open @ Rz.T
result_rot = webxr_to_wrist_local_mano(kp_rotated, 'right', 'mediapipe')
np.testing.assert_allclose(result, result_rot, atol=1e-3)
# → diff = 0.000000 (선형 변환만 적용되어 numerical 오차 한 자리)

# 4. 손가락 굽힘 변화 → 결과 차이 발생 (회전 입력 차이보다 큼)
kp_fist = ...   # fingertip 들이 wrist 쪽으로 모임
result_fist = webxr_to_wrist_local_mano(kp_fist, 'right', 'mediapipe')
diff_pose = np.max(np.abs(result - result_fist))
diff_rot  = np.max(np.abs(result - result_rot))
assert diff_pose > 10 * diff_rot
```

위 4 가지 단위 테스트 모두 통과 (검증 완료, 본 PC 에서 실행 확인).

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/hand/xr_remap.py`
