# Phase C · XR 팔 sender 실측 테스트 가이드

## T1 — 단위 테스트 (헤드셋 / robot 불요)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator

# C1 — frame align
python3 -c "
import numpy as np
from sender.arm.xr_frame_align import XRRelativeFrameAligner

al = XRRelativeFrameAligner(scale=1.0)
user = np.eye(4); user[:3,3] = [0.5, 0.2, 1.4]
al.calibrate(user, np.array([0,-0.4,0.4]), np.array([0,0.707,0.707,0]))
pos, _ = al.apply(user)
assert np.allclose(pos, [0,-0.4,0.4])
print('xr_frame_align OK')
"

# C2 — sender helper
python3 -c "
from sender.arm.xr_sender import _is_valid_pose
import numpy as np
assert not _is_valid_pose(np.zeros((4,4)))
assert _is_valid_pose(np.eye(4))
print('xr_sender helper OK')
"
```

## T2 — sim mode (mock robot, 헤드셋 필요)

xr_sender + admittance sim 을 같은 PC 에서 loopback 으로 연동.

### 2-1. mock robot 띄우기 (admittance sim mode)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m robot.arm.admittance.main --mode sim --input unified

# 기대 출력:
#   [UnifiedInput] Listening on UDP port 9871
#   ... admittance sim mock backend 부팅 ...
#   ... 메인 루프 시작 ...
```

### 2-2. xr_sender 시작

```bash
# adb reverse
adb devices
adb reverse tcp:8013 tcp:8013

# xr_sender
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m sender.arm.xr_sender --target-ip 127.0.0.1
```

기대 boot 시퀀스:
```
[BridgePoseStore] ready: http://localhost:8013/  (WS: /pose)
[XRArmSender] BridgePoseStore ready: http://localhost:8013/
[Sender] Target: 127.0.0.1:9871 at 50 Hz
[Sender] Querying robot for initial pose...
[Sender] Initial pose received: pos=[0.000, -0.400, 0.400]
[XRArmSender] ─────────────────────────────────────
[XRArmSender] keys:
  r       — sync 시작 또는 recalibrate
  ...
[Sender] Running. Press Q or Ctrl+C to stop.
```

### 2-3. 헤드셋 + 'r' 키 + 손 움직임

1. 헤드셋 Chrome → `http://localhost:8013/` → Enter VR (또는 AR)
2. 손을 시야 안에 들이밀기
3. PC 콘솔에서 'r' 키 누름
4. 콘솔 메시지:
   ```
   [XRArmSender] calibrate (r)
   [XRRelativeFrameAligner] calibrated.
     user origin p = [0.512 0.234 1.382]
     robot origin p = [ 0.   -0.4   0.4]
     scale = 1.0
   ```
5. 손을 +x 방향 (오른쪽) 으로 10cm 이동 → mock robot status 에서 TCP 가 +0.1m 변동
6. 손을 yaw 90° 회전 → mock robot TCP rotation 90° 변동

### 2-4. 상태 키 검증

| 키 | 기대 동작 |
|---|---|
| `r` | calibrate 메시지 + virtual_pose = robot origin |
| `p` (paused) | `⏸  PAUSED` 메시지, 손 움직여도 robot 미동 |
| `p` (resumed) | `▶  RESUMED — recalibrate` + 손 새 위치에서 시작 |
| `c` | `calibrate (c (immediate))` + 손 위치에서 즉시 origin 갱신 (jump 가능) |
| `Space` | E-Stop 패킷 송신 → mock robot 측 estop 로그 |
| `+` / `-` | speed 변경 메시지 |
| `x` / `Esc` | sender 종료 |

## T3 — 실 UR10e (admittance + ur_rtde, 저속)

⚠️ **안전 절차**:
- 처음에는 **`--scale 0.3`** 으로 시작 (사용자 손 30cm → robot 10cm)
- UR pendant 의 freedrive / safety 활성
- workspace 비어있는 상태에서 시작

### 3-1. robot PC 측

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda activate teleop_operator
python3 -m robot.arm.admittance.main \
    --mode rtde --input unified --robot-ip 192.168.0.2
```

### 3-2. 조종 PC 측

```bash
adb reverse tcp:8013 tcp:8013
python3 -m sender.arm.xr_sender --target-ip <ROBOT_PC_IP> --scale 0.3
```

### 3-3. 시연 절차

1. 헤드셋 → Enter VR → 손 들이밀기
2. PC 'r' 키 → calibrate
3. 손을 매우 천천히 이동 — UR10e 가 매끄럽게 추종해야 정상
4. `p` 키로 자주 pause/resume — 사용자 손이 워크스페이스 한계 근처일 때 안전 관리
5. 이상 동작 시 `Space` (estop) 또는 pendant freedrive

## Gate C 통과 조건

| 항목 | 측정 | 통과 기준 |
|---|---|---|
| C1 단위 테스트 | 콘솔 | "xr_frame_align OK" |
| C2 단위 테스트 | 콘솔 | "xr_sender helper OK" |
| sim mock robot 시작 | 콘솔 | 정상 boot |
| xr_sender query_pose 성공 | 콘솔 | `[Sender] Initial pose received` |
| `r` 키 → calibrate | 콘솔 | user/robot origin 출력 |
| sim mock 손동작 추종 | mock robot status TCP 출력 | x/y/z delta 가 손 이동 방향과 일치 (scale 1.0) |
| `p` / `c` 키 동작 | 콘솔 | 메시지 정확히 출력 |
| (선택) 실 UR10e 저속 운영 10 분 | 시각 | 손동작 추종, 무사고 |

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `[Sender] WARNING: Pose query failed. Using default home pose.` | robot PC 미시작 또는 firewall | robot 측 `--mode sim --input unified` 시작 확인. `nc -ul 9871` 으로 listen 확인 |
| `r` 눌렀는데 calibrate 메시지 없음 | termios 가 키 못 받음 | 터미널 포커스 확인. `--no-keyboard` 로 자동 시작 모드 시도 |
| user origin 이 `[0,0,0]` | bridge 측 hand 미수신 | 헤드셋에서 Enter VR + 손 들이밀기. `python3 -m scripts.xr_pose_diag` 로 확인 |
| 실 UR10e 가 추종 안 함 | rtde 연결 실패 | robot 측 `--robot-ip` 확인. ur_rtde port (30001-4) 방화벽 확인 |
| 실 UR10e 가 너무 빠르게 움직임 | scale 너무 큼 | `--scale 0.3` 으로 줄임. admittance config 의 max_vel 도 확인 |
| 사용자 손목 회전 시 robot 이 이상한 자세 | quat sign / convention | rotation 1:1 검증: 손 yaw 90° → robot yaw 90° 인지 확인. 이상 시 quat 부호 디버그 |
