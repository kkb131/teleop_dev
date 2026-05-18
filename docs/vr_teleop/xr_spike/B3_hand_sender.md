# Phase B · Unit B3 — XR 손 sender (UDP 9872 publisher)

## 목적

BridgePoseStore → XRDexRetargeter → UDP 9872 송신. receiver.py (변경 없음) 가 retargeted=True passthrough.

## 출처

- xr_teleop: [`xr_teleop/scripts/run_teleop_ur10e_ws.py`](../../../../xr_teleop/scripts/run_teleop_ur10e_ws.py) — DDS rt/dg5f/cmd 발행 흐름
- teleop_dev: [`sender/hand/manus_sender.py`](../../../sender/hand/manus_sender.py) — UDP 송신 + KeyboardState 패턴

## 신규 파일

`src/teleop_dev/sender/hand/xr_hand_sender.py`:
- `_build_packet(q20, wrist_pose, hand_side, buttons, tracking)` — `retargeted=True` 패킷 빌드
- `_build_null_packet(hand_side, buttons)` — tracking lost 패킷
- `_rotmat_to_quat_wxyz(R)` — 회전행렬 → wxyz quaternion
- `main()` — argparse + BridgePoseStore + XRDexRetargeter + 60Hz UDP loop

## Wire format (UDP 9872)

[`protocol/hand_protocol.py`](../../../protocol/hand_protocol.py) 의 HandData 와 호환:

```json
{
  "type": "xr",
  "hand": "right",
  "joint_angles": [20 floats],
  "finger_spread": [5 zeros],
  "wrist_pos": [x, y, z],
  "wrist_quat": [w, x, y, z],
  "tracking": true,
  "buttons": {"estop": false, "reset": false, "quit": false, "speed_up": false, "speed_down": false},
  "timestamp": 1.234e9,
  "retargeted": true
}
```

`retargeted: true` 가 핵심 — receiver.py 의 [`_recv_loop`](../../../robot/hand/receiver.py#L96) 가 이 플래그 보고 passthrough (sender 가 retarget 완료 가정). 기존 manus_sender (`type: "manus"`, retargeted=true/false 선택) 와 동일 contract.

## CLI

```bash
python3 -m sender.hand.xr_hand_sender --target-ip <ROBOT_PC_IP>
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1 --hz 60 --hand right --convention mediapipe
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1 --convention manus  # fist↔spread 반대 시
```

### 옵션

| 옵션 | default | 설명 |
|---|---|---|
| `--target-ip` | 필수 | Robot PC IP |
| `--port` | 9872 | UDP target port |
| `--hz` | 60 | 송신 rate |
| `--hand` | right | DG-5F single right hand 환경. left 미검증 |
| `--convention` | mediapipe | MANO 변환 chirality. fist↔spread inversion 시 manus toggle |
| `--bridge-port` | 8013 | BridgePoseStore ws port |
| `--no-keyboard` | False | pynput 비활성 (sshd / headless) |

## 키 입력 (KeyboardState 통과)

receiver 측에서 처리:
- `Space` — estop (receiver 가 hand command 무시)
- `R` — reset (manual reset 트리거)
- `Q` / `Esc` — sender 종료
- `+` / `-` — speed up/down (receiver 적용)

## 검증 1 — 단위 (헤드셋 / receiver 불요)

```bash
python3 -c "
import sys
sys.path.insert(0, '/workspaces/tamp_ws/src/teleop_dev')
from sender.hand.xr_hand_sender import _rotmat_to_quat_wxyz, _build_packet, _build_null_packet
import numpy as np

# rotmat → quat
assert np.allclose(_rotmat_to_quat_wxyz(np.eye(3)), [1, 0, 0, 0])
print('OK')
"
```

## 검증 2 — smoke (헤드셋 + UDP monitor)

receiver 없이 monitor 로 확인:

```bash
# Terminal 1: UDP monitor (port 9872)
cd /workspaces/tamp_ws/src/teleop_dev
python3 -c "
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 9872))
print('listening 0.0.0.0:9872 ...')
n = 0
import time
t0 = time.time()
while True:
    raw, addr = s.recvfrom(8192)
    pkt = json.loads(raw)
    n += 1
    if n % 30 == 0:
        dt = time.time() - t0
        print(f'#{n} ({n/dt:.1f} Hz) type={pkt[\"type\"]} tracking={pkt[\"tracking\"]} '
              f'retargeted={pkt.get(\"retargeted\")} q[0:6]={pkt[\"joint_angles\"][:6]}')
"

# Terminal 2: sender (헤드셋 USB 연결 + adb reverse 후)
adb reverse tcp:8013 tcp:8013
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1
# 헤드셋 Chrome → http://localhost:8013/ → Enter VR/AR → 손 들이밀기
```

기대 결과:
- Terminal 1: `~60 Hz`, `type=xr`, `tracking=True`, `retargeted=True`, q[0:6] 변동
- Terminal 2: sender 콘솔에 1Hz 간격 `TRACK` 로그 + ws_msg 카운터 증가

## 검증 3 — receiver.py 연동 (DG-5F sim 또는 real)

[`B3_test_guide.md`](B3_test_guide.md) 참조.

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/hand/xr_hand_sender.py`

수정 없음:
- `protocol/hand_protocol.py` — HandData 그대로
- `robot/hand/receiver.py` — retargeted=True passthrough 그대로 (manus_sender 에서 검증된 path)
