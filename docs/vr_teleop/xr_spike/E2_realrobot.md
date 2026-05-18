# Phase E · Unit E2 — 실 UR10e + DG-5F 통합 절차 (저속)

## 목적

조종 PC ↔ 로봇 PC 분리 환경에서 헤드셋 → 실 UR10e + DG-5F 동작 검증. **저속 안전 운영** 우선.

## 사전 안전 점검

| 항목 | 확인 |
|---|---|
| UR10e pendant freedrive / E-stop 동작 | ✓ |
| 안전 fence 또는 책상 주변 공간 확보 | ✓ |
| DG-5F Modbus IP / network ping | ✓ (예: `169.254.186.72`) |
| 조종 PC ↔ 로봇 PC 네트워크 UDP 9871/9872 양방향 통과 | `nc -ul 9871` 으로 listen 확인 |
| 헤드셋 USB-C 연결 + adb 인식 | `adb devices` 통과 |
| 환경 / 의존성 (teleop_operator) 활성 | `conda activate teleop_operator` |

## 절차 (3 단계)

### Step 1 — DG-5F 손만 검증 (팔 정지)

가장 안전 — UR10e 정지 상태에서 손가락 동작만 검증.

**로봇 PC**:
```bash
# A. DG-5F driver 시작 (사용자 환경에 맞춰 IP / 명령 조정)
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

# B. receiver (UDP 9872 → Modbus passthrough)
cd /path/to/teleop_dev
conda activate teleop_operator
python3 -m robot.hand.receiver --hand right
# 기대: "[ROS2] DG5FROS2Client initialized (right hand, mode=real, ...)"
```

**조종 PC**:
```bash
cd /path/to/teleop_dev
conda activate teleop_operator
adb devices
adb reverse tcp:8013 tcp:8013

# 손 sender 만
python3 -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --no-arm

# 또는 단독 모듈
python3 -m sender.hand.xr_hand_sender --target-ip <ROBOT_PC_IP>
```

**검증 시나리오**:
1. 헤드셋 → Chrome `http://localhost:8013/` → Enter VR → 손 들이밀기
2. open hand: DG-5F 손가락 모두 펴짐
3. fist: DG-5F 손가락 모두 굽힘
4. thumb-index pinch: thumb 과 index 가 만남
5. 5분 연속 동작 — 무사고

**fist↔spread inversion** 발견 시:
```bash
# convention 변경
python3 -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --no-arm --convention manus
```

### Step 2 — UR10e sim 모드 (mock backend, 팔 sender 검증)

로봇 PC 에서 mock backend (admittance sim) 로 팔 sender 검증. 실 UR 미연결.

**로봇 PC**:
```bash
python3 -m robot.arm.admittance.main --mode sim --input unified
# 기대: "[UnifiedInput] Listening on UDP port 9871" + mock backend 시작
```

**조종 PC**:
```bash
python3 -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --no-hand --scale 1.0
```

**검증 시나리오**:
1. 헤드셋 → Enter VR + 손 들이밀기
2. PC 'r' 키 → calibrate 메시지 + user/robot origin 출력
3. 손 +x 10cm 이동 → mock robot status 의 TCP 가 +0.1m 이동
4. 손 yaw 90° → mock TCP rotation 90°
5. `p` 키 → pause + 손 새 위치 → resume → 자동 recalibrate
6. `Space` → estop, robot mock 측 estop 로그 + 정지

### Step 3 — 실 UR10e (저속, ur_rtde mode)

⚠️ **가장 위험한 단계**. 처음에는 **`--scale 0.3`** 으로 시작.

**로봇 PC**:
```bash
# UR10e RTDE
python3 -m robot.arm.admittance.main \
    --mode rtde --input unified --robot-ip 192.168.0.2
# 기대: ur_rtde 연결 OK, 메인 루프 시작

# DG-5F 별도 터미널
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72
python3 -m robot.hand.receiver --hand right
```

**조종 PC**:
```bash
adb reverse tcp:8013 tcp:8013

# 저속 통합 실행
python3 -m scripts.run_xr_teleop \
    --target-ip <ROBOT_PC_IP> \
    --scale 0.3 \
    --watchdog-timeout-s 0.2
```

**검증 시나리오** (10 분 운영):

1. **사전 준비**:
   - UR10e workspace 비어있는 상태
   - pendant freedrive 잠금 해제 / E-stop 해제
   - 사용자 손가락 / 다른 사람 robot 옆에 없음 확인

2. **시작**:
   - 헤드셋 → Enter VR
   - 손 들이밀기
   - PC 'r' 키 → calibrate

3. **느린 동작 검증** (5분):
   - 사용자 손 매우 천천히 이동 (~5cm/s)
   - UR10e 가 매끄럽게 따라가는지 확인 (jerky / overshoot 없어야)
   - 손가락 open / fist / pinch — DG-5F 추종 확인

4. **pause / recalibrate** (1분):
   - `p` 키 → pause → 손 새 위치 → `p` 키 → resume
   - jump 없이 새 위치에서 부드럽게 재개

5. **workspace 한계 검증** (1분):
   - 사용자 손을 base 뒤쪽 (y > 0) 으로 이동 시도
   - sender 콘솔: `WARN: target pos ... → clamped` 출력
   - UR10e 가 workspace 안에서만 동작

6. **stale watchdog 검증** (1분):
   - 헤드셋에서 Quit VR → 사용자 손 인식 끊김
   - sender 콘솔: `WARN: BridgePoseStore stale` 1초 안에 출력
   - UR10e 정지 (last target 유지)
   - 사용자 다시 손 들이밀기 → 자동 재개 (`'r'` 키 없이 — last user origin 그대로 사용)

7. **E-stop** (마지막):
   - `Space` 키 → robot 즉시 정지 (admittance safety_monitor.process_estop)
   - 메시지: `[xr_sender] ButtonState.estop=True` + `[safety] E-Stop processed`

## Gate E 통과 조건

| 항목 | 기준 |
|---|---|
| Step 1 (DG-5F only) 5분 무사고 | ✓ |
| Step 2 (UR sim) `r/p/c/Space` 모두 정상 | ✓ |
| Step 3 (UR real) 10분 무사고 | ✓ |
| latency 사용자 체감 | 200ms 이내 |
| `Space` E-stop response | 100ms 이내 robot 정지 |
| workspace clamp 정상 동작 | ✓ |
| stale watchdog 정상 동작 | ✓ |

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| UR rtde 연결 실패 | robot IP / 방화벽 | UR pendant 의 IP 확인. ping. ur_rtde 30001-30004 통과 |
| 'r' 키 calibrate 후 jerk | scale 1.0 + 사용자 손 origin 캡처 시 위치 부정확 | `--scale 0.3` 으로 시작, calibrate 직전 손을 안정 자세 유지 |
| 손가락 inversion (open ↔ fist) | convention | `--convention manus` 재시도 |
| latency 큰 (>500ms) | 헤드셋 USB / Chrome throttle | USB3 포트 사용. 다른 USB device 분리 |
| stale 빈번 발생 | 헤드셋 throttle | `--watchdog-timeout-s 0.5` 로 완화 (단 안전 ↓) |
| UR 가 overshoot / oscillate | admittance gain 너무 강함 | robot 측 admittance config M/K/D 튜닝 (별도 작업) |
| DG-5F 손가락 진동 | retarget jitter | receiver `--ema-alpha 0.1` (강한 smoothing) |

## 다음 단계 (운영 안정 후)

- **scale 1.0 정상속**: Gate E 통과 후 점진적으로 scale ↑ 1.0
- **imitation learning 데이터 수집**: receiver 측 episode writer 추가 (Phase 4)
- **양손 / 카메라 영상**: 별도 Phase
