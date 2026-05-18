# Phase F · Documentation 정리

## 목적

기존 teleop_dev 문서에 XR 통합 내용 반영 + 신규 XR 가이드 작성.

## 수정 파일

### `docs/setup_guide.md`
- §1.5 실행 테스트에 `run_xr_teleop` 명령 추가
- §1.5b 신설: XR 추가 설치 단계 요약 (Google ADB / udev / aiohttp / adb reverse)

### `docs/ARCHITECTURE.md`
- 시스템 개요 다이어그램에 `Galaxy XR / Quest 3` + `BridgePoseStore` 추가
- 디렉토리 구조에 신규 폴더 / 파일 추가:
  - `sender/xr_common/` (bridge_pose_store, watchdog, config, assets)
  - `sender/arm/xr_sender.py` + `xr_frame_align.py`
  - `sender/hand/xr_hand_sender.py` + `xr_dex_retargeter.py` + `xr_remap.py` + `config_xr/`
  - `scripts/run_xr_teleop.py` + `scripts/xr_pose_diag.py`
- 실행 명령에 XR sender 3종 추가
- 의존성 표에 `aiohttp` / `dex_retargeting` / `mediapipe` 등 분류

## 신규 파일

### `docs/xr_input_guide.md`
사용자 가이드 — Galaxy XR / Quest 3 사용 전체 절차:
1. 전체 흐름 (다이어그램)
2. 사전 요건 (헤드셋 / 조종 PC / 로봇 PC)
3. 초기 설치 (Google ADB / udev / conda / PYTHONPATH)
4. 매번 사용 절차 (헤드셋 연결 / pose-only 검증 / 통합 실행 / 키 매핑 / 옵션)
5. 카메라 영상 (현재 비활성, 추후 WebRTC 추가 시 활성화 절차)
6. 단계별 검증 (Phase A~E 가이드 cross-link)
7. 트러블슈팅 (4 카테고리: 헤드셋 ↔ 조종 PC, 손, 팔, 환경)
8. 관련 문서

`arm_input_guide.md` / `hand_manus_guide.md` 와 같은 형태의 사용자 가이드.

### `docs/vr_teleop/xr_integration_plan.md`
사용자가 이미 옮긴 통합 계획서 (Phase A~F + 영향 범위 + risks).

### `docs/vr_teleop/xr_spike/*.md`
각 phase 의 spike + test guide (Phase A~E).

## 검증

```bash
# 1. 모든 cross-link 가 존재하는 파일을 가리키는지 확인
cd /path/to/teleop_dev
ls docs/vr_teleop/xr_spike/   # A1, A2, A3, B1, B2, B3, C1, C2, D1, E1, E2, F 등

# 2. 신규 사용자 시나리오 검증 (가이드만 보고 환경 구축)
#    - setup_guide.md §1.5b 따라 의존성 설치
#    - xr_input_guide.md §3 따라 ADB / udev / PYTHONPATH
#    - xr_input_guide.md §4 따라 첫 실행
```

## Gate F 통과 기준

- ARCHITECTURE.md 의 디렉토리 구조 / 의존성 표가 신규 파일 모두 반영
- setup_guide.md 에 XR 추가 절차 명시
- xr_input_guide.md 가 self-contained (다른 문서 안 보고도 사용 가능)
- vr_teleop/xr_integration_plan.md 의 Phase A~F 모두 commit 완료

## 영향 받는 파일

수정:
- `src/teleop_dev/docs/ARCHITECTURE.md`
- `src/teleop_dev/docs/setup_guide.md`

신규:
- `src/teleop_dev/docs/xr_input_guide.md`
- `src/teleop_dev/docs/vr_teleop/xr_spike/F_docs.md` (이 파일)
