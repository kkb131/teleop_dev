# Phase B · 손 sender 실측 테스트 가이드

## T1 — 단위 테스트 (헤드셋 / receiver 불요)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator

# B1 — frame transform
python3 -c "
import numpy as np
from sender.hand.xr_remap import webxr_to_wrist_local_mano, is_kp25_valid
assert not is_kp25_valid(np.zeros((25, 3)))
kp = np.random.rand(25, 3) + 0.1
out = webxr_to_wrist_local_mano(kp, 'right', 'mediapipe')
assert out.shape == (25, 3)
assert np.allclose(out[0], 0)
print('xr_remap OK')
"

# B2 — retargeter selftest
python3 -m sender.hand.xr_dex_retargeter --selftest
# 기대: [selftest] PASS

# B3 — sender helper unit
python3 -c "
import numpy as np
from sender.hand.xr_hand_sender import _rotmat_to_quat_wxyz, _build_packet
q = _rotmat_to_quat_wxyz(np.eye(3))
assert np.allclose(q, [1, 0, 0, 0])
print('xr_hand_sender helpers OK')
"
```

## T2 — UDP smoke (헤드셋 필요, receiver 불요)

### 2-1. UDP monitor 띄우기

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 9872))
print('UDP 0.0.0.0:9872 listening')
n, t0 = 0, time.time()
while True:
    raw, addr = s.recvfrom(8192)
    pkt = json.loads(raw)
    n += 1
    if n % 30 == 0:
        dt = time.time() - t0
        q = pkt['joint_angles']
        print(f'#{n:>5d} {n/dt:5.1f}Hz type={pkt[\"type\"]} track={pkt[\"tracking\"]} '
              f'retarget={pkt.get(\"retargeted\")} q[5/9/13/17/1]=[{q[5]:+.2f}/{q[9]:+.2f}/{q[13]:+.2f}/{q[17]:+.2f}/{q[1]:+.2f}]')
"
```

### 2-2. 헤드셋 + sender

```bash
# 헤드셋 USB-C 연결 + adb reverse
adb devices
adb reverse tcp:8013 tcp:8013

# sender 시작 (loopback 으로 monitor 와 같은 PC)
conda activate teleop_operator
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1
```

헤드셋 Chrome 에서 `http://localhost:8013/` → **Enter VR** (또는 AR) → 손 들이밀기.

### 2-3. 기대 결과

UDP monitor (Terminal 1):
```
#30   60.0Hz type=xr track=True retarget=True q[5/9/13/17/1]=[+0.32/+0.41/+0.50/+0.62/-0.14]
#60   60.0Hz type=xr track=True retarget=True q[5/9/13/17/1]=[+0.40/+0.55/+0.61/+0.75/-0.20]
```

sender (Terminal 2):
```
[xr_hand_sender] #    60 TRACK ws_msg=58 q[0:6]=[-0.12 -0.31 -0.19 -0.12 +0.00 +0.32]
```

손동작 별 기대값:
- **open hand**: q[5/9/13/17] (MCP) ≈ 0~0.3, q[1] (thumb opp) ≈ 0
- **fist**: q[5/9/13/17] ≈ 1.5~2.0, q[1] ≈ -1.0~-2.0
- **thumb-index pinch**: q[0] (thumb abd) + q[5] 변화

## T3 — receiver.py 연동 (sim mode)

receiver 가 retargeted=True 패킷을 받아 EMA 후 ROS2 / Modbus 로 publish 하는지 확인.

### 3-1. dry-run (ROS2 없이 콘솔 print)

```bash
# Terminal 1: receiver dry-run
python3 -m robot.hand.receiver --dry-run --hand right
# → "Waiting for data..." 출력

# Terminal 2: xr_hand_sender (위 T2 와 동일)
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1
```

기대: receiver 콘솔에 finger-major 20-vec 표 출력 (recv 와 send 값 비교):
```
  Hand: RIGHT | Mode: RETARGETED (passthrough)
  Finger    Spread(R/S)  MCP(R/S)    PIP(R/S)    DIP(R/S)
  Thumb     -0.12/-0.12  -0.31/-0.30  -0.19/-0.18  -0.12/-0.11
  Index     +0.00/+0.00  +0.32/+0.31  +0.20/+0.19  +0.13/+0.12
  ...
```

### 3-2. Isaac Sim mode (선택)

```bash
# Terminal 1: receiver sim mode (ROS2 필요)
python3 -m robot.hand.receiver --hand right --mode sim
# → /dg5f_right/joint_commands publish 시작

# Terminal 2: xr_hand_sender
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1

# Isaac Sim 측에서 DG-5F controller 가 subscribe 하여 손가락 움직임 시각 확인
```

### 3-3. 실 DG-5F (선택, 사용자가 sub network 접근 가능 시)

**Right hand** (Phase B4 fix 후 검증 완료):
```bash
# Terminal 1: dg5f_driver PID controller (별도 PC 또는 같은 PC)
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

# Terminal 2: receiver real mode
python3 -m robot.hand.receiver --hand right
# → "[ROS2] DG5FROS2Client initialized (right hand, mode=real, topic=...)"

# Terminal 3: xr_hand_sender
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1
```

**Left hand** (Phase B5 신규):
```bash
# Terminal 1: dg5f_driver PID controller — left launch
ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py delto_ip:=<LEFT_DG5F_IP>

# Terminal 2: receiver real mode — --hand left
python3 -m robot.hand.receiver --hand left
# → "[ROS2] DG5FROS2Client initialized (left hand, mode=real, topic=...)"

# Terminal 3: xr_hand_sender — --hand left
python3 -m sender.hand.xr_hand_sender --target-ip 127.0.0.1 --hand left
```

DG-5F 손가락이 헤드셋 (right 시 오른손 / left 시 **왼손**) 동작을 따라가는지 시각 확인.

⚠️ left 검증 시:
- 헤드셋에서 **왼손** 들이밀어야 함 (BridgePoseStore 의 `left_hand_positions` 가 채워짐)
- WebXR HandLandmarker 가 `handedness: "left"` 자동 인식 → packet 에 `"hand": "left"` 송신
- 처음 fist↔spread 가 시각적으로 반대로 보이면 `--convention manus` toggle 시도 ([mano_transform.py:95-97](../../../sender/hand/core/mano_transform.py#L95) 의 left convention 하드웨어 검증 기회).

## Gate B 통과 조건

| 항목 | 측정 | 통과 기준 |
|---|---|---|
| B1 단위 테스트 | 콘솔 출력 | "xr_remap OK" |
| B2 selftest | exit code | 0 (PASS 표시) |
| B3 helper 단위 | 콘솔 출력 | "xr_hand_sender helpers OK" |
| UDP smoke 송신 frequency | monitor `#N/Hz` | ≥ 30 Hz |
| `tracking=True` 비율 | monitor 출력 | 손 시야 안 들어왔을 때 100% |
| open hand → MCP 양수 (`q[5,9,13,17] ≥ 0`) | monitor | ✓ |
| fist → MCP 큰 양수 (`≥ 1.0`) | monitor | ✓ |
| (선택) receiver dry-run | 콘솔 표 | recv/send 값 표시 |
| (선택) DG-5F sim/real 손가락 추종 | 시각 | open/fist/pinch 모두 동작 |

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `tracking=False` 만 계속 | 헤드셋 hand 인식 안 됨 | 손 시야 안 들이밀기. `inputsourceschange: hands=2` 로그 확인 |
| `open hand → MCP 음수` 또는 `fist → MCP 0` | convention mismatch | `--convention manus` 로 재시도 |
| receiver dry-run 에서 recv/send 큰 차이 | EMA filter 미수렴 (정상) | `--ema-alpha 1.0` 으로 EMA 비활성 시도 |
| Isaac Sim DG-5F 미추종 | sim 측 controller 미설정 | xr_teleop SIM_UR10E_DG5F_BUILD_GUIDE.md 참고 |
| 실 DG-5F 손가락이 휙휙 튐 | retarget jitter | receiver `--ema-alpha 0.1` 으로 강한 smoothing |
