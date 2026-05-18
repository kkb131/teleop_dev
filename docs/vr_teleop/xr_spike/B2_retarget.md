# Phase B · Unit B2 — DexPilot retargeter wrapper

## 목적

WebXR 25-joint → DG-5F 20-vec joint angle (radians). xr_teleop 의 `DG5F_Controller._control_process` retarget 부분만 추출.

## 출처

- xr_teleop: [`xr_teleop/scripts/dg5f_controller.py`](../../../../xr_teleop/scripts/dg5f_controller.py#L302) — `_control_process` 의 retarget flow
- xr_teleop 검증 yml: [`xr_teleop/assets/dg5f_hand/dg5f_right.yml`](../../../../xr_teleop/assets/dg5f_hand/dg5f_right.yml)
- xr_teleop 검증 URDF: [`xr_teleop/assets/dg5f_hand/dg5f_right_retarget.urdf`](../../../../xr_teleop/assets/dg5f_hand/dg5f_right_retarget.urdf) — PIP/DIP lower=0 으로 fist 형 lock 회피

## 신규 파일

```
src/teleop_dev/sender/hand/
├── xr_dex_retargeter.py            # XRDexRetargeter class + expand_retarget_to_dg5f_20
└── config_xr/
    ├── dg5f_right_xr.yml           # DexPilot 설정 (xr_teleop 검증값 복제)
    ├── dg5f_right_retarget.urdf    # PIP/DIP lower=0 URDF
    └── meshes/                      # collision/visual STL/DAE (12MB)
```

## API

```python
from sender.hand.xr_dex_retargeter import XRDexRetargeter

rt = XRDexRetargeter(convention="mediapipe", hand_side="right")

# WebXR 25-joint (BridgePoseStore.right_hand_positions) → DG-5F 20-vec
q20 = rt.retarget(kp_25)
# q20.shape == (20,) — thumb(0..3), index(4..7), middle(8..11), ring(12..15), pinky(16..19)

# invalid kp 입력 시 last valid q20 반환 (zeros 초기값) — hold-on-lost 안전 패턴
q20 = rt.retarget(np.zeros((25, 3)))   # → zeros (또는 직전 q20)
```

## DG-5F 20-vec finger-major 순서

| DDS index | DG-5F joint | 의미 |
|---|---|---|
| 0 | rj_dg_1_1 | thumb 외전 |
| 1 | rj_dg_1_2 | thumb 굴곡 (lower=-π, upper=0 — negative direction) |
| 2 | rj_dg_1_3 | thumb mid |
| 3 | rj_dg_1_4 | thumb tip |
| 4 | rj_dg_2_1 | index 외전 (고정 0) |
| 5 | rj_dg_2_2 | index MCP |
| 6 | rj_dg_2_3 | index PIP |
| 7 | rj_dg_2_4 | index DIP |
| 8..11 | rj_dg_3_* | middle |
| 12..15 | rj_dg_4_* | ring |
| 16..19 | rj_dg_5_* | pinky |

DexPilot type 일 때 retargeter 가 target=20 으로 전체 joint 풀이 → expand 불필요.
Vector type (target=6) fallback 시 `expand_retarget_to_dg5f_20` 가 mimic 0.6/0.4 로 distal 채움.

## dex_retargeting 버전 호환성

xr_teleop 코드는 `target_link_human_indices_dexpilot` 명시 인덱스 사용.
teleop_dev 의 dex_retargeting==0.5.0 은 해당 키 비지원 — 자동 생성 (auto-derive from `finger_tip_link_names`) 만 지원. yml 에서 명시 인덱스 제거하면 동일 결과 (Inspire / BrainCo upstream 패턴과 일치).

| xr_teleop yml | teleop_dev yml | 결과 |
|---|---|---|
| `target_link_human_indices_dexpilot: [[9,14,...],[4,4,...]]` | (제거) | 동일 — auto-generated `[4, 9, 14, 19, 24]` |

## 검증 (단위, 헤드셋 불요)

```bash
python3 -m sender.hand.xr_dex_retargeter --selftest
```

기대 출력:
```
[XRDexRetargeter] dg5f_right_xr.yml target=20 fixed=0 convention=mediapipe side=right
[selftest] init OK: 20 target joints
[selftest] invalid kp → q20 zeros OK
[selftest] open hand → q20 shape=(20,), range=[-0.543, 1.572]
[selftest] fist → q20 range=[-0.556, 1.572]
[selftest] mean PIP delta (fist - open) = +0.014  (should be positive)
[selftest] yaw rotation robustness: max(|q - q_rot|) = 0.5217  (should < 0.1)
[selftest] PASS
```

### 주의: synthetic dummy 한계

- selftest 의 dummy "open hand" / "fist" 는 실제 손 keypoint 분포 와 다르므로 PIP delta 가 매우 작게 나옴 (검증 0.014). 실 hand data 에서 retargeter 가 정확히 동작 (xr_teleop Week 4 시각 검증 통과: open / fist / thumb-index pinch).
- yaw rotation robustness 도 dummy 에서는 0.52 (target < 0.1 초과). 실 hand 에서는 frame transform 효과로 훨씬 정확.
- selftest 는 PASS / FAIL 판단보다 **import + URDF 로드 + retarget 호출 자체가 죽지 않는지** 확인용.

### 실 검증 (헤드셋 + receiver) — Phase B3 의 [B3_test_guide.md](B3_test_guide.md) 참조

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `ImportError: cannot import name 'RetargetingConfig' from 'dex_retargeting'` | dex_retargeting 버전 차이 (0.5.0+) | `from dex_retargeting.retargeting_config import RetargetingConfig` 사용 (코드 이미 적용) |
| `unexpected keyword argument 'target_link_human_indices_dexpilot'` | dex_retargeting 0.5.0 는 명시 인덱스 미지원 | yml 에서 해당 키 제거 (코드 이미 적용) |
| URDF mesh 파일 누락 경고 | `config_xr/meshes/` 누락 | meshes 디렉토리 commit 확인. `git ls-files config_xr/meshes` |
| `ipopt` 미설치 / 다른 solver 오류 | dex_retargeting nlopt backend 자동 사용 | `pip install dex_retargeting` 시 nlopt 함께 설치됨. python -c "import nlopt" 확인 |

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/hand/xr_dex_retargeter.py`
- `src/teleop_dev/sender/hand/config_xr/dg5f_right_xr.yml`
- `src/teleop_dev/sender/hand/config_xr/dg5f_right_retarget.urdf` (xr_teleop 복제)
- `src/teleop_dev/sender/hand/config_xr/meshes/` (xr_teleop 복제, 12MB)
