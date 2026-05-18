# Phase A · Unit A2 — 헤드셋 실측 테스트 가이드

> 헤드셋 (Galaxy XR 또는 Quest 3) USB-C 연결된 PC 에서 [`sender/xr_common`](../../../sender/xr_common/) 동작 검증.

## T1 — 자동 selftest (헤드셋 없이)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m sender.xr_common.bridge_pose_store --selftest
```

✅ `[selftest] PASS` 출력되면 통과.

## T2 — adb reverse 설정 (헤드셋 USB-C 연결 후)

```bash
# 1. 헤드셋 PC USB-C 연결
adb devices
# → 헤드셋 시리얼 (예: R3KYA01R62L) device 표시

# 2. port forwarding
adb reverse tcp:8013 tcp:8013
adb reverse --list
# → (reverse) UsbFfs tcp:8013 tcp:8013
```

⚠️ Galaxy XR 첫 연결 시 USB 디버깅 RSA 키 지문 팝업 → "이 컴퓨터에서 항상 허용" 체크.

## T3 — server + 헤드셋 접속

```bash
# PC 측
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m sender.xr_common.bridge_pose_store

# 헤드셋 Chrome
# → http://localhost:8013/ 접속
# → "Auto diagnostic" 단락에서 'immersive-vr: true', 'immersive-ar: true' 표시 확인
```

## T4 — Enter VR (또는 AR) + 손 들이밀기

1. 헤드셋 Chrome 페이지에서 **Enter VR** (또는 **Enter AR** — pass-through 모드) 클릭
2. 손을 시야 안에 들이밀기
3. 헤드셋 화면 (Live session log) 에서 1Hz 간격으로 다음 로그 확인:
   ```
   xrFrame=N ws.sent=60/s ws.fail=0/s ws=open hands=2 right.wrist=[+0.xxx,+0.xxx,-0.xxx]
   ```
4. PC 콘솔에서:
   ```
   [BridgePoseStore] ws client connected: ...
   [bridge_pose_store] msgs=120 head=60 hand=60
   ```

## T5 — pose-only diagnostic 측정 (Gate A)

```bash
# PC 측, 별도 터미널에서
python3 -m scripts.xr_pose_diag --measure 30 \
    --report docs/vr_teleop/xr_spike/A3_diag_result.md
```

가이드:
- 처음 ~10초: 손/팔 자연스럽게 움직임
- 10~15초: 손을 시야 밖으로 5초간 뺐다가 다시 들이밀기 (recovery 측정)
- 25~30초: 손 정지 (wrist jitter 측정)

### Gate A 통과 기준

| 항목 | 측정값 | 통과 기준 |
|---|---|---|
| mean_freq_hz | ≥ 30 Hz | ≥ 30 |
| lost_per_field (head/right_arm/right_hand) | < 5% | < 5% |
| recovery_latency_s | < 1.0 s | < 1.0 |
| wrist_jitter_cm | < 2 cm | < 2 |

→ 통과 시 docs/vr_teleop/xr_spike/A3_diag_result.md 에 결과 표 append.

## T6 — smoke mode (1Hz 콘솔 로그)

```bash
python3 -m scripts.xr_pose_diag
# 1Hz 로그가 흐르면서 head/rArm/rHand 의 OK 상태 + wrist 좌표 표시
# Ctrl+C 종료
```

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| 헤드셋에서 `http://localhost:8013/` "이 페이지에 도달할 수 없습니다" | adb reverse 안 됨 | `adb reverse --list` 로 확인, `adb kill-server && adb start-server` 후 재시도 |
| "Enter VR/AR" 비활성 | WebXR API 미지원 또는 Chrome flag 필요 | Galaxy XR 의 경우: `chrome://flags/#webxr-incubations` enable + 재시작 |
| 헤드셋 화면 켜졌는데 PC 콘솔에 msg=0 | ws 연결 실패 | 헤드셋 화면 (Live session log) 의 `ws closed; reconnect in 1s` 메시지 확인. adb reverse 검증 |
| wrist 좌표가 `[0,0,0]` | 손이 시야 안에 들어가야 함 | 손 들이밀기, hand tracking 권한 확인 |
| `lost_right_hand: 100%` | 헤드셋이 hands 인식 못 함 | inputsourceschange 로그 `hands=0` 이면 카메라 가림. controller 들고있으면 `controllers=2` |
| mean_freq_hz < 30 | 헤드셋 USB bandwidth / Chrome throttle | 다른 USB 디바이스 분리. PC USB3 포트 사용 |
