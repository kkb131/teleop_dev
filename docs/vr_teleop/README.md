# XR 통합 (Galaxy XR / Meta Quest 3) — 문서 인덱스

> WebXR + USB-C 기반 헤드셋으로 teleop_dev 의 UR10e + DG-5F 를 원격조종. xr_teleop 의 검증된 BridgePoseStore + dex_retargeting 패턴을 teleop_dev 분리 아키텍처에 이식.

## 우선 읽을 문서 순서

1. **[xr_integration_plan.md](xr_integration_plan.md)** — 통합 방향 결정 + 영향 범위 + Phase A~F 작업 단위 + risks
2. **[../xr_input_guide.md](../xr_input_guide.md)** — 사용자 가이드 (설치 / 실행 / 트러블슈팅)
3. **xr_spike/** — Phase 별 spike + test guide (디버그 + 검증 절차)

## Phase 별 산출물

### Phase A — 환경 + ws bridge + pose-only 검증

| 산출 | 파일 |
|---|---|
| spike: env 갱신 | [xr_spike/A1_env.md](xr_spike/A1_env.md) |
| spike: BridgePoseStore + HTML | [xr_spike/A2_bridge.md](xr_spike/A2_bridge.md) |
| spike: xr_pose_diag | [xr_spike/A3_diag.md](xr_spike/A3_diag.md) |
| test guide (헤드셋 실측) | [xr_spike/A2_test_guide.md](xr_spike/A2_test_guide.md) |
| 신규 코드 | `sender/xr_common/` + `scripts/xr_pose_diag.py` |

### Phase B — XR 손 sender (UDP 9872)

| 산출 | 파일 |
|---|---|
| spike: frame transform | [xr_spike/B1_remap.md](xr_spike/B1_remap.md) |
| spike: DexPilot retargeter | [xr_spike/B2_retarget.md](xr_spike/B2_retarget.md) |
| spike: UDP sender | [xr_spike/B3_hand_sender.md](xr_spike/B3_hand_sender.md) |
| test guide (smoke + DG-5F) | [xr_spike/B3_test_guide.md](xr_spike/B3_test_guide.md) |
| **fix: auto-generated indices (open/fist 미동작 버그)** | [xr_spike/B4_indices_fix.md](xr_spike/B4_indices_fix.md) |
| 신규 코드 | `sender/hand/xr_*.py` + `sender/hand/config_xr/` |

### Phase C — XR 팔 sender (UDP 9871)

| 산출 | 파일 |
|---|---|
| spike: relative motion frame align | [xr_spike/C1_frame_align.md](xr_spike/C1_frame_align.md) |
| spike: UDP sender + 키 매핑 | [xr_spike/C2_arm_sender.md](xr_spike/C2_arm_sender.md) |
| test guide (sim + UR real) | [xr_spike/C2_test_guide.md](xr_spike/C2_test_guide.md) |
| 신규 코드 | `sender/arm/xr_sender.py` + `sender/arm/xr_frame_align.py` |

### Phase D — 통합 launcher

| 산출 | 파일 |
|---|---|
| spike + smoke | [xr_spike/D1_unified.md](xr_spike/D1_unified.md) |
| 신규 코드 | `scripts/run_xr_teleop.py` |

### Phase E — 안전 layer + 실로봇

| 산출 | 파일 |
|---|---|
| spike: watchdog + workspace clamp | [xr_spike/E1_safety.md](xr_spike/E1_safety.md) |
| 실 UR10e 운영 절차 (3 단계) | [xr_spike/E2_realrobot.md](xr_spike/E2_realrobot.md) |
| 신규 코드 | `sender/xr_common/watchdog.py` |

### Phase F — 문서화

| 산출 | 파일 |
|---|---|
| docs spike | [xr_spike/F_docs.md](xr_spike/F_docs.md) |
| 갱신 | `docs/ARCHITECTURE.md`, `docs/setup_guide.md` |
| 신규 사용자 가이드 | `docs/xr_input_guide.md` |

## Gate 통과 기준 (요약)

| Gate | 검증 | 통과 기준 |
|---|---|---|
| A | xr_pose_diag --measure 30 | mean_freq ≥ 30Hz / lost < 5% / recovery < 1s / jitter < 2cm |
| B | 단위 + (선택) DG-5F sim | open/fist/pinch 시각 추종 |
| C | sim + (선택) UR10e 저속 | r/p/c/Space 모두 동작, 손동작 매끄러운 추종 |
| D | 통합 smoke | 한 process 에서 두 sender 60Hz 동시 송신 |
| E | UR10e + DG-5F 저속 10분 | 무사고, E-stop 100ms, watchdog/clamp 정상 |
| F | 신규 사용자 1시간 재현 | xr_input_guide.md 만으로 sync 시작 |

## 영향 받지 않은 영역 (중요)

XR 통합으로 **수정 없음**:
- `protocol/arm_protocol.py`, `protocol/hand_protocol.py` (wire format)
- `robot/` 전체 (admittance, impedance, receiver, kinematics, safety_monitor 등)
- 기존 sender (`sender/arm/vive_sender.py`, `sender/hand/manus_sender.py`, `sender/hand/realsense_sender.py` 등)

XR sender 가 같은 wire format 으로 송신 → robot PC 측은 sender 종류를 모름 (기존 sender 와 동일하게 처리).

## 향후 작업 (별도 Phase G+)

- WebRTC 카메라 publisher 통합 (현재 비활성)
- WAN 분리 운영 (Tailscale + Thick Relay)
- imitation learning 데이터 수집 (LeRobot 호환 episode writer)
- 양손 (left hand) sender — DG-5F left URDF 필요
- F/T 시각화 AR overlay
- eye-tracking + gaze-guided manipulation
