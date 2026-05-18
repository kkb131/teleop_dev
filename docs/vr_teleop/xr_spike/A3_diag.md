# Phase A · Unit A3 — xr_pose_diag.py

## 목적

조종 PC ↔ 헤드셋 ws bridge 의 정량 검증 (Gate A).

## 출처 (참고 — 복제 아님)

- [xr_teleop/scripts/test_pose_only_ws.py](../../../../xr_teleop/scripts/test_pose_only_ws.py) — 핵심 측정 패턴 (mean_freq / lost / recovery / jitter)

## 측정 metric

| 항목 | 의미 | 통과 기준 |
|---|---|---|
| `mean_freq_hz` | 1초 윈도우당 ws msg 수 평균 | ≥ 30 Hz |
| `lost_per_field` | field (head/left_arm/right_arm/left_hand/right_hand) 가 invalid (zeros or NaN) 상태인 sample 비율 | < 5% |
| `recovery_latency_s` | right_arm 이 invalid → valid 전환 시 lost streak 최대 길이 × poll_dt | < 1.0 s |
| `wrist_jitter_cm` | 마지막 5초 valid wrist position 표준편차의 norm × 100 (cm) | < 2 cm |

### 검증 방식

- 200 Hz 로 BridgePoseStore property 폴링 (sender 가 보통 보는 read 빈도)
- field invalid 판정:
  - SE(3): `np.allclose(M, 0)` 또는 `M[3,3] != 1`
  - 25×3 keypoint: `np.allclose(kp, 0)`
- 1초 단위로 ws msg count delta → msg/s 시계열 → 평균 = `mean_freq_hz`

## 인터페이스

```bash
# smoke (1Hz 콘솔 로그, Ctrl+C 종료)
python3 -m scripts.xr_pose_diag

# 측정 모드
python3 -m scripts.xr_pose_diag --measure 30
python3 -m scripts.xr_pose_diag --measure 30 --report docs/vr_teleop/xr_spike/A3_diag_result.md

# port 변경
python3 -m scripts.xr_pose_diag --port 8014
```

exit code: Gate A 모두 통과 시 0, 하나라도 미달이면 1.

## 결과 보고

`--report <path>.md` 옵션 사용 시 표 형태로 append. 측정 timestamp 헤더 포함.

## 영향 받는 파일

신규:
- `src/teleop_dev/scripts/xr_pose_diag.py`
