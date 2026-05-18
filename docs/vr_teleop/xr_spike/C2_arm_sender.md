# Phase C · Unit C2 — XR 팔 sender (UDP 9871 publisher)

## 목적

BridgePoseStore.right_arm_pose → XRRelativeFrameAligner → TeleopPosePacket → UDP 9871 → robot/arm/(admittance|impedance)/main.py + UnifiedNetworkInput (변경 없음).

## 출처

- xr_teleop: [`xr_teleop/scripts/run_teleop_ur10e_ws.py:284-358`](../../../../xr_teleop/scripts/run_teleop_ur10e_ws.py#L284) — r/c/p/q 키 + relative motion 메인 루프
- teleop_dev: [`sender/arm/sender_base.py`](../../../sender/arm/sender_base.py) — TeleopSenderBase (query_pose 핸드셰이크, run() 루프)
- teleop_dev 참고: [`sender/arm/keyboard_sender.py`](../../../sender/arm/keyboard_sender.py) — termios non-blocking 키 입력 패턴

## 신규 파일

`src/teleop_dev/sender/arm/xr_sender.py`:
- `XRArmSender(TeleopSenderBase)` — sender_base 의 query_pose / run() 흐름 활용 + _read_input override
- `_calibrate_now()` — robot.query_pose 응답 + bridge.right_arm_pose 동시 capture
- `main()` — argparse + run()

## sender_base 와의 차이

teleop_dev 의 `TeleopSenderBase` 는 누적 모델 (`_apply_delta(dp, drot)`) 가정. xr_sender 는 **절대 매핑** 이라 매 loop 에서 `virtual_pos / virtual_quat` 를 직접 set:

```python
def _read_input(self) -> InputResult:
    # ... 키 처리 ...
    target_pos, target_quat = self._aligner.apply(user_pose)
    self._virtual_pos = target_pos
    self._virtual_quat = target_quat
    return InputResult(...)   # delta=0 → sender_base._apply_delta 가 noop
```

sender_base 의 `_send_packet` 가 `self._virtual_pos / _virtual_quat` 를 그대로 송신 → wire format / robot PC 인터페이스 호환 (UnifiedNetworkInput 가 받는 그대로).

## 키 매핑

| 키 | 동작 |
|---|---|
| `r` | sync 시작 또는 recalibrate. robot query_pose 재요청 + user origin 캡처 |
| `p` | pause / resume. pause 시 virtual_pose 유지 (robot 명령 그대로) → 손 새 위치로 옮길 때. resume 시 자동 recalibrate |
| `c` | immediate recalibrate. pause 없이 즉시 — jump 가능 |
| `Space` | E-Stop (ButtonState.estop=True 송신 → robot 측 admittance safety_monitor estop) |
| `x` / `Esc` / `q` | quit |
| `+` / `-` | speed up / down (ButtonState 송신 → robot 측 적용) |

## State 표시 (status line)

sender_base 의 `_get_speed_label` 을 override 해 한 줄로:
```
[Sender] #     30  pos=[+0.10, -0.40, +0.40]  spd=SYNC x0.2
```

- `READY`: 시작 전 (still query_pose 안 했거나 r 안 누름)
- `SYNC`: 정상 동작
- `PAUSE`: paused
- `x0.5`: speed scale

## CLI

```bash
# loopback smoke
python3 -m sender.arm.xr_sender --target-ip 127.0.0.1

# 실 환경
python3 -m sender.arm.xr_sender --target-ip 192.168.0.10 --scale 0.3
```

### 옵션

| 옵션 | default | 설명 |
|---|---|---|
| `--target-ip` | 필수 | Robot PC IP |
| `--port` | 9871 | UDP target port |
| `--hz` | 50 | 송신 rate |
| `--scale` | 1.0 | Position scale. rotation 은 항상 1:1 |
| `--convention` | mediapipe | (hand sender 와 동일 옵션 유지, arm 측 미사용) |
| `--bridge-port` | 8013 | BridgePoseStore ws port |
| `--hand` | right | DG-5F single right hand 환경 |
| `--no-keyboard` | False | termios 비활성 — sshd / headless |

## query_pose 핸드셰이크

sender_base 의 `query_initial_pose()` 가 startup 시 robot PC 에 `query_pose` 송신 → robot 측 UnifiedNetworkInput.`_handle_pose_query()` 가 현재 TCP pose 응답. xr_sender 는:

1. **Startup**: `query_initial_pose()` 가 robot TCP 응답 받으면 virtual_pos / virtual_quat 그 값으로 init (sender_base 가 처리)
2. **`r` / `c` / `p resume`**: `_calibrate_now()` 안에서 다시 `query_initial_pose()` 호출 (timeout 짧음) → 최신 robot TCP 로 origin 갱신

robot 측 코드 변경 없음 — 기존 query_pose 응답 메커니즘 그대로.

## 검증 1 — 단위 (헤드셋 / robot 불요)

```bash
python3 -c "
import numpy as np
from sender.arm.xr_sender import _is_valid_pose
assert not _is_valid_pose(np.zeros((4,4)))
assert _is_valid_pose(np.eye(4))
print('xr_sender helper OK')
"

# C1 의 xr_frame_align 단위 테스트 묶음 (C1_frame_align.md 참조)
```

## 검증 2 — smoke (헤드셋 + robot loopback)

```bash
# Terminal 1: mock robot side (admittance sim mode)
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m robot.arm.admittance.main --mode sim --input unified

# Terminal 2: xr_sender
adb reverse tcp:8013 tcp:8013
python3 -m sender.arm.xr_sender --target-ip 127.0.0.1
```

기대 동작:
1. xr_sender startup: `[Sender] Initial pose received: pos=[...]` (robot 의 mock TCP 응답)
2. 헤드셋 Chrome → `http://localhost:8013/` → Enter VR/AR → 손 들이밀기
3. PC 콘솔에서 'r' 키 → `[XRArmSender] calibrate (r)` + user/robot origin 출력
4. 손 움직임 → sender 가 절대 pose 송신 → robot 측 admittance 가 mock TCP 추종 (콘솔 status 로그 확인)

## 검증 3 — 실 robot (admittance / impedance, 저속)

[`C2_test_guide.md`](C2_test_guide.md) 참조.

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/arm/xr_sender.py`

수정 없음:
- `sender/arm/sender_base.py` — TeleopSenderBase 그대로 (run, query, _send_packet 재사용)
- `protocol/arm_protocol.py` — TeleopPosePacket / ButtonState 그대로
- `robot/core/input_handler.py` — UnifiedNetworkInput query_pose 응답 그대로
- `robot/arm/admittance/main.py`, `robot/arm/impedance/main.py` — 변경 없음
