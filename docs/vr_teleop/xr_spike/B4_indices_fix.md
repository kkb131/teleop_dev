# Phase B · Unit B4 — DexPilot auto-generated indices 가 MANO 21-joint 가정인 버그 수정

> 발견 시점: 2026-05-19 — Galaxy XR → 실 DG-5F E2E 테스트 ([B3_test_guide.md §3-3](B3_test_guide.md)) 진행 시
> 영향 범위: Phase B (XR 손 sender) 의 retargeting 정확도 — 사용자 손 펴기/쥐기 미추종
> 처방 commit: yml 에 `target_link_human_indices` 명시 + retargeter init 단계 진단 강화

## 증상

Galaxy XR → 실 DG-5F E2E 테스트:
- ✅ Phase B3 단위 / smoke / receiver dry-run 모두 통과
- ✅ §3-3 실 DG-5F (dg5f_driver PidController + Modbus) 에서 DG-5F 모터 동작
- ❌ **사용자 open hand → DG-5F 안 펴짐**
- ❌ **사용자 fist → DG-5F 안 쥐임**

데이터는 정상 전달되는데 retargeter 출력 자체가 손 자세에 거의 반응 안 함.

## 근본 원인 — `target_link_human_indices` 자동 생성이 MANO 21-joint 가정

설치된 [`dex_retargeting==0.5.0`](file:///usr/local/lib/python3.10/dist-packages/dex_retargeting/optimizer.py#L357) 의 DexPilotOptimizer:

```python
# optimizer.py:357-364
origin_link_index, task_link_index = self.generate_link_indices(num_fingers)
if target_link_human_indices is None:
    target_link_human_indices = (
        np.stack([origin_link_index, task_link_index], axis=0) * 4   # ← MANO 가정
    ).astype(int)
```

`* 4` 곱셈 — MANO 21-joint 가정 (각 손가락 4 joint, fingertip 인덱스 = finger# × 4):
- 자동 생성 fingertip indices: **`[4, 8, 12, 16, 20]`** (MANO)
- 실제 WebXR 25-joint fingertip indices: **`[4, 9, 14, 19, 24]`** (각 손가락 5 keypoint — metacarpal 포함)

## Off-by-finger-index 누적 효과 — 사용자 증상 정확히 일치

`target_indices[1, :]` 의 fingertip 부분이 `[4, 8, 12, 16, 20]` 일 때 WebXR `hand_local[25, 3]` 인덱싱:

| Finger | "Tip" 으로 인덱스되는 joint | WebXR 실 tip | 오차 (joint #) |
|---|---|---|---|
| Thumb (0) | `hand_local[4]` = thumb-tip ✓ | 4 | 0 (정확) |
| Index (1) | `hand_local[8]` = **distal phalanx** | 9 | 1 |
| Middle (2) | `hand_local[12]` = **intermediate phalanx** | 14 | 2 |
| Ring (3) | `hand_local[16]` = **proximal phalanx** | 19 | 3 |
| **Pinky (4)** | `hand_local[20]` = **metacarpal (손가락 base!)** | 24 | **4** |

→ Pinky 의 "fingertip" target 이 손가락 BASE — 사용자가 손을 펴거나 쥐어도 metacarpal 은 거의 움직이지 않음. retargeter 가 "pinky tip 이 안 움직이니 pinky joints 도 가만히 두자" 로 풀이. middle / ring 도 비슷한 효과 누적.

→ **DG-5F 가 손가락이 거의 펴지지 / 쥐어지지 않는 사용자 증상과 정확히 일치**.

## xr_teleop 도 같은 버그 — silent failure

xr_teleop 의 [`assets/dg5f_hand/dg5f_right.yml:38`](../../../../xr_teleop/assets/dg5f_hand/dg5f_right.yml#L38) 가 `target_link_human_indices_dexpilot` 키 사용 — `_dexpilot` suffix 가 있는 잘못된 키. [`dex_retargeting/retargeting_config.py:157-160`](file:///usr/local/lib/python3.10/dist-packages/dex_retargeting/retargeting_config.py#L157) 는 정확한 키 이름 `target_link_human_indices` 만 인식 → xr_teleop 의 명시 인덱스 **silently ignored**. xr_teleop IsaacSim 도 사실 같은 MANO auto-gen 사용 중.

xr_teleop 사용자가 "open / fist / pinch OK" 라고 보고했으나 ([week4_report.md](../../../../xr_teleop/docs/week4_report.md)):
- thumb 만 인덱스 정확 (4 → 4) → thumb-index pinch 가 가장 정확
- middle / ring / pinky 는 fingertip 인덱스 mismatch 가 누적되나 IsaacSim 의 부드러운 모터 다이내믹 + 정량 측정 부재로 시각 인식 어려웠을 가능성

## Phase B2 의 잘못된 가정

[B2_retarget.md](B2_retarget.md) 의 이전 단락 ("dex_retargeting 버전 호환성"):
> "xr_teleop 코드는 target_link_human_indices_dexpilot 명시 인덱스 사용.
>  teleop_dev 의 dex_retargeting==0.5.0 은 해당 키 비지원 — 자동 생성만 지원.
>  yml 에서 명시 인덱스 제거하면 동일 결과 (auto-generated [4, 9, 14, 19, 24])"

→ 검증 누락. 실제로는 auto-gen 이 **MANO 가정의 [4, 8, 12, 16, 20]** 생성. B4 에서 정정.

## 처방

### 수정 1: yml 에 명시 인덱스 추가 (정확한 키 이름)

[`sender/hand/config_xr/dg5f_right_xr.yml`](../../../sender/hand/config_xr/dg5f_right_xr.yml) 끝부분:

```yaml
right:
  type: DexPilot
  urdf_path: dg5f_right_retarget.urdf
  wrist_link_name: "rl_dg_palm"
  finger_tip_link_names: [...]

  # WebXR 25-joint 의 (origin, task) pair indices — 명시 필수.
  #   row 0 (origin): 10 finger-pair-distance origin + 5 wrist-to-tip origin (wrist=0)
  #   row 1 (task)  : 10 finger-pair-distance task   + 5 wrist-to-tip task   (tip)
  # 10 pair-distance: (4-9), (4-14), (4-19), (4-24), (9-14), (9-19), (9-24), (14-19), (14-24), (19-24)
  # 5  wrist-to-tip: (0-4), (0-9), (0-14), (0-19), (0-24)
  target_link_human_indices:
    - [ 9, 14, 19, 24, 14, 19, 24, 19, 24, 24,  0,  0,  0,  0,  0]
    - [ 4,  4,  4,  4,  9,  9,  9, 14, 14, 19,  4,  9, 14, 19, 24]

  scaling_factor: 1.2
  low_pass_alpha: 0.2
```

xr_teleop 원본 yml 과 같은 값 + 정확한 키 이름 (no `_dexpilot` suffix).

### 수정 2: retargeter init 단계 진단 + selftest assert

[`sender/hand/xr_dex_retargeter.py`](../../../sender/hand/xr_dex_retargeter.py) `__init__` 끝에 task indices 검증:

```python
if self.target_indices is not None and self.target_indices.ndim == 2:
    task_tail = np.asarray(self.target_indices[1, -5:]).tolist()
    webxr_tips = [4, 9, 14, 19, 24]
    if sorted(set(task_tail)) == webxr_tips:
        print(f"[XRDexRetargeter] task fingertip indices = {task_tail} ✓ matches WebXR 25-joint fingertips")
    else:
        print(f"[XRDexRetargeter] ⚠️  fingertip indices {task_tail} != WebXR {webxr_tips}")
        print(f"  → yml 의 target_link_human_indices 명시 누락 가능성. ...")
```

`_selftest` 에 hard assert 추가:
```python
task_tail = sorted(set(np.asarray(rt.target_indices[1, -5:]).tolist()))
assert task_tail == [4, 9, 14, 19, 24], (
    f"task fingertip indices {task_tail} != WebXR. "
    f"yml 의 target_link_human_indices 키 누락 또는 잘못된 suffix 가능성."
)
```

→ 향후 yml / library 버전 변경 시 자동으로 잡힘.

## 검증

### 1. 단위 (헤드셋 / 실 robot 불요)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m sender.hand.xr_dex_retargeter --selftest
```

기대 출력:
```
[XRDexRetargeter] dg5f_right_xr.yml target=20 fixed=0 convention=mediapipe side=right
[XRDexRetargeter] task fingertip indices = [4, 9, 14, 19, 24] ✓ matches WebXR 25-joint fingertips
[selftest] init OK: 20 target joints
[selftest] task fingertip indices = [4, 9, 14, 19, 24] ✓
[selftest] invalid kp → q20 zeros OK
[selftest] open hand → q20 shape=(20,), range=[-0.479, 1.048]
[selftest] fist → q20 range=[-0.482, 0.899]
[selftest] mean PIP delta (fist - open) = +0.060  (should be positive)
[selftest] yaw rotation robustness: max(|q - q_rot|) = 0.5212  (should < 0.1)
[selftest] PASS
```

핵심 확인: `task fingertip indices = [4, 9, 14, 19, 24] ✓`.

### 2. 실 DG-5F 재현 (B3_test_guide.md §3-3)

robot PC:
```bash
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72
python3 -m robot.hand.receiver --hand right
```

조종 PC:
```bash
adb reverse tcp:8013 tcp:8013
python3 -m sender.hand.xr_hand_sender --target-ip <ROBOT_PC_IP>
```

헤드셋 Chrome → `http://localhost:8013/` → Enter VR → 손 들이밀기.

| 자세 | B4 fix 이전 (auto-gen indices) | B4 fix 이후 (명시 indices) | 통과 |
|---|---|---|---|
| Open hand (5 손가락 펴짐) | DG-5F 거의 변화 없음 (특히 pinky) | DG-5F 5 손가락 모두 펴짐 | ✓ |
| Fist | DG-5F 거의 안 쥐임 | DG-5F 5 손가락 모두 굽힘 | ✓ |
| Thumb-index pinch | DG-5F thumb-index 만 약간 동작 | DG-5F thumb-index 명확히 만남 | ✓ |
| 손 yaw 회전 | (frame transform 효과 — 손가락 자세 유지) | 동일 | ✓ |

### 3. fist↔spread 가 시각적으로 반대 (별도 2 차 fallback)

본 fix 만으로 안 풀리는 경우 ([B3_test_guide.md](B3_test_guide.md) 트러블슈팅 표):
```bash
python3 -m sender.hand.xr_hand_sender --target-ip <ROBOT_PC_IP> --convention manus
```

WebXR HandLandmarker 의 chirality 가 MediaPipe 와 다를 때 (row 1 sign flip). 본 indices fix 와 독립적인 별도 문제.

## Left hand 도 같은 fix 적용 (Phase B5)

본 B4 의 명시 인덱스 fix 는 hand_side 와 무관 — WebXR human keypoint 인덱싱은 left/right 동일. Phase B5 ([B5_left_hand.md](B5_left_hand.md)) 에서 yml 의 `left:` block 도 같은 `target_link_human_indices` 블록을 명시. selftest 가 `--hand left` 옵션으로 left URDF + indices 정확성 둘 다 검증.

## 영향 받는 파일

수정:
- `src/teleop_dev/sender/hand/config_xr/dg5f_xr.yml` (Phase B5 에서 `dg5f_right_xr.yml` 에서 rename) — 명시 인덱스 블록 추가 + 주석 정정
- `src/teleop_dev/sender/hand/xr_dex_retargeter.py` — init 진단 + selftest assert
- `src/teleop_dev/docs/vr_teleop/xr_spike/B2_retarget.md` — Phase B2 의 잘못된 claim 정정

신규:
- `src/teleop_dev/docs/vr_teleop/xr_spike/B4_indices_fix.md` (이 파일)

수정 없음:
- `xr_remap.py` — frame transform 변경 없음
- `xr_hand_sender.py` — 송신 로직 변경 없음
- `robot/hand/receiver.py` / `dg5f_ros2_client.py` — robot PC 측 영향 없음

## 출처 / 참조

| 항목 | 파일 | 줄 |
|---|---|---|
| **버그 원인** | `/usr/local/lib/python3.10/dist-packages/dex_retargeting/optimizer.py` | 357-364 (`* 4` MANO 가정) |
| **yml 키 인식 위치** | `/usr/local/lib/python3.10/dist-packages/dex_retargeting/retargeting_config.py` | 157-160 |
| **xr_teleop silent failure 위치** | `src/xr_teleop/assets/dg5f_hand/dg5f_right.yml` | 38 (`_dexpilot` suffix) |
| **WebXR 25-joint 정의** | `src/teleop_dev/sender/xr_common/bridge_pose_store.py` | 89-101 (JOINT_NAMES) |
| **MediaPipe 21-joint 비교 출처** | `src/teleop_dev/sender/hand/core/mano_transform.py` | 모듈 docstring |

## Why this fix is the right level

1. **정확히 사용자 증상과 매칭**: pinky 가 가장 안 움직이고 thumb 만 정확 — off-by-finger-index 누적 효과의 정확한 표현.
2. **xr_teleop 검증 통과의 모순 해석**: xr_teleop sim 도 사실 같은 자동 인덱스 사용 (yml 키 오타로 silent fail). thumb-index pinch 위주 시각 검증으로 가려졌을 가능성.
3. **dex_retargeting 표준 인터페이스 그대로**: 라이브러리 패치 / monkey-patch 불요. yml 명시 한 블록으로 해결.
4. **회귀 위험 최소**: yml + retargeter init log + selftest assert 만. 다른 sender / receiver / frame transform 변경 없음.
5. **재발 방지**: retargeter init 시점에 자동 검증 → 향후 yml / library 버전 변경으로 indices 가 다시 wrong 으로 바뀌면 즉시 selftest fail.

## Out of Scope

- xr_teleop 측 yml 수정 (`_dexpilot` suffix 제거) — 별도 repo. xr_teleop 가 정확한 indices 로 IsaacSim 재검증 받는 게 유익하나 본 작업 우선순위 아님.
- `convention="manus"` toggle — 본 fix 와 독립적인 2 차 fallback. fist↔spread 시각 반대 시 진행.
- DG-5F 실 driver 다이내믹 튜닝, EMA alpha — retarget 정확도 확보 후 분리.
