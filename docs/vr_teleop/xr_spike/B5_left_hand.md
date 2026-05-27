# Phase B · Unit B5 — DG-5F Left Hand 지원 + dg5f_xr.yml rename + selftest --hand

> 발견 시점: 2026-05-19 — B4 indices fix 적용 후 right hand E2E 정상 동작 검증 완료. 사용자가 left hand 도 같은 path 로 테스트 요청.
> 영향 범위: XR sender 의 hand_side 인프라 활성화 + left URDF / meshes / yml block 추가
> 작업 commit: yml rename + left URDF/meshes 이식 + retargeter DEFAULT_YML_PATH/selftest 갱신 + 문서

## 배경

B4 fix 이후 `--hand right` 정상 동작. 코드 자체는 left 도 지원하는 인프라 95% 준비 — 한 곳 막힌 곳이 `dg5f_right_xr.yml` 에 `right:` top-level key 만 있던 점.

## 이미 준비된 인프라 (수정 없음)

| 모듈 | left 지원 상태 |
|---|---|
| [`sender/hand/xr_remap.py:85-126`](../../../sender/hand/xr_remap.py#L85) | `hand_side` 따라 `MEDIAPIPE_OPERATOR2MANO_LEFT` 선택 ✓ |
| [`sender/hand/xr_dex_retargeter.py:122-125`](../../../sender/hand/xr_dex_retargeter.py#L122) | `cfg[hand_side]` 로 yml top-level key 자동 선택 ✓ |
| [`sender/hand/xr_hand_sender.py`](../../../sender/hand/xr_hand_sender.py) | `--hand right\|left` CLI ✓ |
| [`sender/arm/xr_sender.py:238-241`](../../../sender/arm/xr_sender.py#L238) | `store.left_arm_pose` 분기 ✓ |
| [`scripts/run_xr_teleop.py:114-119`](../../../scripts/run_xr_teleop.py#L114) | hand thread 의 left 분기 ✓ |
| [`sender/xr_common/bridge_pose_store.py:188-203`](../../../sender/xr_common/bridge_pose_store.py#L188) | `left_hand_position_shared` 등 ✓ |
| [`sender/hand/core/mano_transform.py:70-77`](../../../sender/hand/core/mano_transform.py#L70) | `MEDIAPIPE_OPERATOR2MANO_LEFT` 정의 ✓ |
| [`robot/hand/dg5f_ros2_client.py:40-78`](../../../robot/hand/dg5f_ros2_client.py#L40) | `LEFT_JOINT_NAMES`, `prefix=lj_dg` 자동 전환 ✓ |
| [`robot/hand/receiver.py`](../../../robot/hand/receiver.py) | `--hand left` 옵션 + packet `"hand"` 필드 파싱 ✓ |
| WebXR HTML (`webxr_to_pose.html`) | `handedness: "left"` / `"right"` 자동 publish ✓ |

→ 결국 **신규 자료 추가 + DEFAULT_YML_PATH 변경 + selftest 옵션 한 줄** 만 필요.

## 핵심 변경

### 1. 신규 자료: `sender/hand/config_xr/dg5f_left_retarget.urdf`

원본: `/workspaces/tamp_ws/src/dg5f_ros2/dg5f_description/urdf/dg5f_left.urdf` (745 lines, native left URDF)

3 가지 수정 적용:

**A. Mesh path 치환** (56 회):
```diff
- package://dg5f_description/meshes/dg5f_left/
+ ./meshes/
```
sed 일괄 적용. right URDF 와 같은 상대 경로 패턴 (`./meshes/{visual,collision}/`).

**B. PIP/DIP joint limit** — `lj_dg_{1..5}_3` 및 `lj_dg_{1..5}_4` 10 개 joint (axis `"1 0 0"`):
```diff
- <limit lower="-1.5707963267948966" upper="1.5707963267948966" .../>
+ <limit lower="0.0"                  upper="1.5707963267948966" .../>
```

손가락 anatomical 굴곡 방향이 left/right 모두 positive 라 `lower=0` 패턴 동일.

**C. (B5 사후 fix) 4 개 single-sign joint limit — right URDF 와 sign 동일하게 변경**.

⚠️ **사용자 1차 실측 (2026-05-27)**: left 의 다른 손가락 (index/middle/ring/pinky) 은 잘 굴곡되는데 **엄지 (thumb) 가 안 굴곡됨** 보고. 진단 결과:

| 항목 | 결과 |
|---|---|
| `dg5f_ros2_client.py` (publisher) | 단순 passthrough, sign 변환 없음 |
| `dg5f_driver` (`set_position_rad`) | radian → uint16 직접 cast, per-joint sign 없음 |
| `dg5f_*_pid_all_controller.yaml` | 양쪽 동일 p=1.5, sign 변환 없음 |
| native `dg5f_left.urdf` 의 `_*_2` MCP flex joint | **양쪽 limit 동일 `[0, 2.0]`** (axis `0 1 0`) |
| native `dg5f_left.urdf` 의 `_1_2`, `_1_1`, `_5_1`, `_5_2` | left 가 right 와 sign mirror — visualization 전용 |

→ DG-5F hardware 는 좌/우 **동일 단일 motor unit + 동일 회전 방향 convention**. native dg5f_left.urdf 의 sign mirror 는 sim/RViz visualization 만을 위한 것. 실 hardware 사용 시 retargeter 가 잘못된 sign 으로 motor 명령 → 특히 thumb opp 같은 single-sign joint 에서 motor 가 정반대로 움직임 또는 안 움직임.

`_*_2` MCP flex 가 **양쪽 동일** 이라는 사실이 결정적 증거 — DG-5F URDF author 가 일부 joint 만 mirror 하고 나머지 (단방향만 가능한 joint) 는 hardware convention 그대로 둔 것으로 추정.

**fix**: 4 개 joint limit 를 right 와 sign 동일하게:

| joint | left native (잘못, sim 전용) | left retarget (B5 fix) | right (reference) |
|---|---|---|---|
| `lj_dg_1_1` (thumb abd) | `[-0.89, 0.38]` | `[-0.38, 0.89]` | `[-0.38, 0.89]` |
| **`lj_dg_1_2` (thumb opp)** | **`[0, π]`** | **`[-π, 0]`** | `[-π, 0]` |
| `lj_dg_5_1` (pinky abd) | `[-1.05, 0.02]` | `[-0.02, 1.05]` | `[-0.02, 1.05]` |
| `lj_dg_5_2` (pinky flex) | `[-0.61, 0.42]` | `[-0.42, 0.61]` | `[-0.42, 0.61]` |

⭐ `lj_dg_1_2` 는 single-sign (negative-only) joint — retargeter 가 limit 안에서만 풀이하므로 limit 가 잘못된 부호로 설정되면 motor 가 반대 방향으로만 출력 → 사용자 thumb 미동작 증상 직접 원인.

다른 abduction joint (`_1_1`, `_5_1`, `_5_2`) 들은 양방향 range 라 retargeter 가 두 방향 모두 시도 가능, 사용자가 thumb 만큼 명확한 증상 보지 못 했을 수 있음 — 그래도 정확성 위해 일괄 fix.

**불변**: `_*_2` MCP flex (index/middle/ring/pinky 4 joint) 는 native 가 이미 양쪽 동일 `[0, 2.0]` 이라 fix 불요.

### 2. 신규 자료: `sender/hand/config_xr/meshes/{visual,collision}/ll_dg_*`

원본: `/workspaces/tamp_ws/src/dg5f_ros2/dg5f_description/meshes/dg5f_left/{visual,collision}/`

28 개 visual (`.dae`) + 28 개 collision (`.STL`) 그대로 복사. 기존 right meshes (`rl_dg_*`) 와 **같은 폴더 공존** — 파일명이 다르므로 충돌 없음.

### 3. yml: `dg5f_right_xr.yml` → `dg5f_xr.yml` rename + `left:` block 추가

`git mv` 로 rename. 이유:
- 한 파일에 right + left 두 hand 가 포함되므로 "right" suffix 부적절
- `cfg[hand_side]` 패턴 이미 multi-hand 지원

`left:` block 구조 (right 와 같은 패턴):
```yaml
left:
  type: DexPilot
  urdf_path: dg5f_left_retarget.urdf
  wrist_link_name: "ll_dg_palm"
  finger_tip_link_names:
    - "ll_dg_1_tip"
    - "ll_dg_2_tip"
    - "ll_dg_3_tip"
    - "ll_dg_4_tip"
    - "ll_dg_5_tip"
  # WebXR human keypoint 인덱싱이라 right 와 동일
  target_link_human_indices:
    - [ 9, 14, 19, 24, 14, 19, 24, 19, 24, 24,  0,  0,  0,  0,  0]
    - [ 4,  4,  4,  4,  9,  9,  9, 14, 14, 19,  4,  9, 14, 19, 24]
  scaling_factor: 1.2
  low_pass_alpha: 0.2
```

`scaling_factor` / `low_pass_alpha` / `target_link_human_indices` 는 right 와 동일. `wrist_link_name` / `finger_tip_link_names` / `urdf_path` 만 left prefix (`ll_*`).

### 4. `sender/hand/xr_dex_retargeter.py` 수정

**DEFAULT_YML_PATH**:
```diff
- DEFAULT_YML_PATH = _DEFAULT_CONFIG_DIR / "dg5f_right_xr.yml"
+ DEFAULT_YML_PATH = _DEFAULT_CONFIG_DIR / "dg5f_xr.yml"
```

같은 yml 안의 `cfg[hand_side]` 가 left/right 선택. 호출자 (`xr_hand_sender.py`, `run_xr_teleop.py`) 는 변경 불요 — `XRDexRetargeter(hand_side=args.hand)` 가 이미 hand_side 전달.

**`_selftest(hand_side)` 시그니처**:
```python
def _selftest(hand_side: str = "right") -> int:
    print(f"[selftest] hand_side={hand_side}")
    try:
        rt = XRDexRetargeter(hand_side=hand_side)
    ...
```

**main block**:
```python
parser = argparse.ArgumentParser(...)
parser.add_argument("--selftest", action="store_true")
parser.add_argument("--hand", default="right", choices=["right", "left"])
args = parser.parse_args()
if args.selftest:
    sys.exit(_selftest(hand_side=args.hand))
```

기존 right hand 단위 테스트 + assert 그대로 — left 도 같은 selftest 실행해 PASS 확인.

## 검증

### 1. 단위 (헤드셋 / 실 robot 불요)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator

# right (회귀)
python3 -m sender.hand.xr_dex_retargeter --selftest --hand right
# left (B5 신규)
python3 -m sender.hand.xr_dex_retargeter --selftest --hand left
```

기대 결과 (양쪽 모두 PASS):
```
=== RIGHT ===
[selftest] hand_side=right
[XRDexRetargeter] dg5f_xr.yml target=20 fixed=0 convention=mediapipe side=right
[XRDexRetargeter] task fingertip indices = [4, 9, 14, 19, 24] ✓ matches WebXR 25-joint fingertips
[selftest] init OK: 20 target joints
[selftest] task fingertip indices = [4, 9, 14, 19, 24] ✓
...
[selftest] open hand → q20 shape=(20,), range=[-0.479, 1.048]
[selftest] fist → q20 range=[-0.482, 0.899]
[selftest] mean PIP delta (fist - open) = +0.060
[selftest] PASS

=== LEFT ===
[selftest] hand_side=left
[XRDexRetargeter] dg5f_xr.yml target=20 fixed=0 convention=mediapipe side=left
[XRDexRetargeter] task fingertip indices = [4, 9, 14, 19, 24] ✓ matches WebXR 25-joint fingertips
[selftest] init OK: 20 target joints
[selftest] task fingertip indices = [4, 9, 14, 19, 24] ✓
...
[selftest] open hand → q20 shape=(20,), range=[-1.048, 0.477]
[selftest] fist → q20 range=[-1.011, 0.504]
[selftest] mean PIP delta (fist - open) = +0.105
[selftest] PASS
```

⚠️ **left 의 q20 range 가 right 의 mirror 형태** — `[-1.048, 0.477]` 가 right 의 `[-0.479, 1.048]` 부호 반전. URDF 의 thumb opp 와 mirror geometry 영향 — 정상 결과.

### 2. 실 left DG-5F (B3_test_guide.md §3-3 의 left 변형)

robot PC:
```bash
ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py delto_ip:=<LEFT_DG5F_IP>
python3 -m robot.hand.receiver --hand left
# → "[ROS2] DG5FROS2Client initialized (left hand, mode=real, topic=...)"
```

조종 PC:
```bash
adb reverse tcp:8013 tcp:8013
python3 -m sender.hand.xr_hand_sender --target-ip <ROBOT_PC_IP> --hand left
# 또는 통합 launcher:
python3 -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --hand left --no-arm
```

헤드셋 (Galaxy XR Chrome → `http://localhost:8013/` → Enter VR → **왼손** 들이밀기) 에서:

| 자세 (왼손) | 기대 left DG-5F 동작 | 통과 |
|---|---|---|
| Open hand | 5 손가락 펴짐 | ✓ |
| Fist | 5 손가락 굽힘 | ✓ |
| Thumb-index pinch | thumb-index 만남 | ✓ |
| 좌우 yaw 회전 | 손가락 자세 유지 (frame transform) | ✓ |

만약 fist↔spread 시각 반대 — `--convention manus` toggle. `MANUS_OPERATOR2MANO_LEFT` 의 "not yet hardware-verified" 경고 ([mano_transform.py:95-97](../../../sender/hand/core/mano_transform.py#L95)) 해결할 기회.

### 3. 회귀 — right 영향 없음

```bash
# right 그대로 동작
python3 -m sender.hand.xr_hand_sender --target-ip <IP> --hand right
# 또는
python3 -m scripts.run_xr_teleop --target-ip <IP> --hand right
```

DEFAULT_YML_PATH 변경 (`dg5f_right_xr.yml` → `dg5f_xr.yml`) 에도 right E2E 동일 — 동일 yml 안의 `right:` block 가 같은 값 유지.

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `selftest --hand left` init FAIL: `yml has no 'left' key` | yml rename / left block 누락 | `git status` 로 `dg5f_xr.yml` 존재 확인. yml 안에 `right:` + `left:` 둘 다 있는지 확인 |
| URDF 로드 실패 — mesh 파일 못 찾음 | `dg5f_left_retarget.urdf` 의 mesh path 가 `package://` 잔존 | `grep package:// dg5f_left_retarget.urdf` 로 0 회 확인. sed 재실행 |
| `lj_dg_*_3` 또는 `lj_dg_*_4` 가 음수로 풀이 | PIP/DIP `lower=0` 미적용 | `grep 'lower="-1.5707963267948966"' dg5f_left_retarget.urdf` 로 잔존 limit 확인. sed 재실행 |
| **엄지가 안 굽혀짐 (다른 손가락 OK)** | `lj_dg_1_2` limit 가 native `[0, π]` 그대로 — DG-5F hardware 는 좌/우 동일 회전 convention 이라 mirror 부호가 hardware 와 불일치 | B5 의 §1.C fix 적용 — `lj_dg_1_2` limit `[-π, 0]` 로 변경 (right 와 동일). 다른 3 개 (`lj_dg_1_1`, `lj_dg_5_1`, `lj_dg_5_2`) 도 일괄 fix |
| 헤드셋 왼손 안 잡힘 (`hands=0` log) | hand tracking 의 left 활성 안 됨 | 왼손을 시야 안에 충분히 들이밀기. WebXR `handedness: "left"` 자동 인식 |
| 좌우 헤드셋 손이 바뀜 (오른손 들이밀어도 left DG-5F 움직임) | WebXR HandLandmarker 의 handedness 가 mirror 이미지에선 반대일 수 있음 | 헤드셋 자체 inputsourceschange 디버그 — `[run_xr_teleop:hand] ws_msg=...` log 의 `handedness` 필드 확인 |
| left DG-5F 의 thumb 만 inverse | thumb opp 의 sign convention (right 는 `[-π, 0]`, left 는 `[0, π]`) | URDF 가 native 라 이미 처리됨. retargeter 가 알아서 양쪽 풀이 |

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/hand/config_xr/dg5f_left_retarget.urdf` (745 lines)
- `src/teleop_dev/sender/hand/config_xr/meshes/visual/ll_dg_*.dae` (28 files)
- `src/teleop_dev/sender/hand/config_xr/meshes/collision/ll_dg_*_c.STL` (28 files)
- `src/teleop_dev/docs/vr_teleop/xr_spike/B5_left_hand.md` (이 파일)

수정:
- `src/teleop_dev/sender/hand/config_xr/dg5f_xr.yml` (rename + `left:` block + 주석)
- `src/teleop_dev/sender/hand/xr_dex_retargeter.py` (DEFAULT_YML_PATH + `_selftest(hand_side)` + main argparse)
- `src/teleop_dev/docs/vr_teleop/xr_spike/B3_test_guide.md` (§3-3 left 절차 추가)
- `src/teleop_dev/docs/vr_teleop/xr_spike/B4_indices_fix.md` (left 적용 단락 + yml 경로 갱신)
- `src/teleop_dev/docs/vr_teleop/README.md` (B5 추가)
- `src/teleop_dev/docs/xr_input_guide.md` (left 옵션 예시 추가)
- `src/teleop_dev/docs/ARCHITECTURE.md` (config_xr 구조 갱신)

삭제 (rename 의 부수):
- `src/teleop_dev/sender/hand/config_xr/dg5f_right_xr.yml` (→ `dg5f_xr.yml`)

수정 없음:
- `xr_remap.py`, `xr_hand_sender.py`, `xr_sender.py`, `run_xr_teleop.py`, `bridge_pose_store.py`, `mano_transform.py`, `dg5f_ros2_client.py`, `receiver.py` — left hand 분기 이미 인프라 차원 완비

## 출처 / 참조

| 항목 | 파일 | 줄 |
|---|---|---|
| **dg5f_left 원본 URDF** | `src/dg5f_ros2/dg5f_description/urdf/dg5f_left.urdf` | 745 lines |
| **dg5f_left meshes** | `src/dg5f_ros2/dg5f_description/meshes/dg5f_left/` | 56 files (28 visual + 28 collision) |
| **left thumb opp limit** | `dg5f_left_retarget.urdf` (사본) | `lj_dg_1_2`: `[0, π]` (unchanged) |
| **left PIP/DIP limit** | 같은 file | `lj_dg_*_{3,4}`: `[-π/2, π/2]` → `[0, π/2]` (10 joints) |
| **MANO 매트릭스 LEFT** | `src/teleop_dev/sender/hand/core/mano_transform.py` | 70-77 (mediapipe), 98-105 (manus, hardware-unverified) |
| **xr_remap hand_side 분기** | `src/teleop_dev/sender/hand/xr_remap.py` | 85-126 |
| **xr_dex_retargeter cfg\[hand_side\]** | `src/teleop_dev/sender/hand/xr_dex_retargeter.py` | 122-125 |
| **dg5f_ros2_client LEFT_JOINT_NAMES** | `src/teleop_dev/robot/hand/dg5f_ros2_client.py` | 40-78 |
| **WebXR HTML handedness** | `src/teleop_dev/sender/xr_common/assets/webxr_to_pose.html` | (frame loop 안 `src.handedness` 자동) |

## Why this is the right level

1. **인프라 변경 최소**: 95% 이미 준비된 분기 활용. 신규 코드: DEFAULT_YML_PATH 1 줄 + `_selftest(hand_side)` 시그니처 1 줄 + main argparse 변경.
2. **right 회귀 위험 최소**: yml rename + 같은 위치의 `right:` block 유지 → right E2E 동일.
3. **재발 방지**: selftest 가 `--hand right|left` 양쪽 검증 — 향후 yml/URDF 변경으로 left 가 깨지면 즉시 잡힘.
4. **dg5f_ros2 native URDF 신뢰**: mirror 직접 변환 안 함 — ROS2 패키지의 검증된 URDF 활용 (joint axis / origin / mass 등 hardware 일치 가정).
5. **MANUS_OPERATOR2MANO_LEFT 검증 기회**: 사용자 left hand 실측 시 부수 효과로 manus convention 의 hardware 검증 가능.

## Out of Scope

- **양손 (dual-hand) 동시 운영**: UR10e 한 팔 + DG-5F 한 손 환경 — single-hand. dual-hand 는 future Phase.
- **MANUS convention left 하드웨어 검증**: 본 작업은 mediapipe default 만 검증. fist↔spread inversion 발견 시 별도 toggle 진행.
- **dg5f_left URDF 의 native joint axis 검증**: 사용자 hardware 가 native URDF 와 일치한다고 가정. 실 left DG-5F 가 반대 동작 시 native URDF 의 axis sign 재검토 필요.
- **xr_teleop 측 left 지원**: xr_teleop 은 IsaacSim sim only, single right hand. 별도 repo.
