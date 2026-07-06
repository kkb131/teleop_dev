# XR (Galaxy XR / Meta Quest 3) 원격조종 사용 가이드

> Samsung Galaxy XR (SM-I610) 또는 Meta Quest 3 headset 으로 UR10e + Tesollo DG-5F 를 원격조종한다. WebXR + USB-C (adb reverse) 기반 — WiFi / SteamVR 불요.

---

## 1. 전체 흐름

```
[헤드셋 Chrome (WebXR)]
    │ webxr_to_pose.html (자체 작성, ~500 lines)
    │ XR-RAF onFrame 매 frame head/hand pose JSON 송신 (WebSocket)
    ▼  USB-C (adb reverse tcp:8013)
[조종 PC — teleop_operator conda env]
    BridgePoseStore (Singleton, aiohttp HTTP+WS server, port 8013)
    │
    ├─ scripts/run_xr_teleop.py   (통합 launcher)
    │    │
    │    ├─ XR arm sender (UDP 9871)
    │    │    BridgePoseStore.right_arm_pose → XRRelativeFrameAligner →
    │    │    TeleopPosePacket (절대 target pose, base_link frame)
    │    │
    │    └─ XR hand sender (UDP 9872)
    │         BridgePoseStore.right_hand_positions → XRDexRetargeter (DexPilot) →
    │         retargeted=True 패킷 (DG-5F 20-vec joint angles)
    │
    │ UDP 9871 / 9872 (JSON)
    ▼
[로봇 PC — teleop_operator]
    robot/arm/admittance|impedance/main.py + UnifiedNetworkInput  (변경 없음)
    robot/hand/receiver.py + DG5FROS2Client                       (변경 없음)
    │ ur_rtde servoJ 125Hz / Modbus TCP 100Hz
    ▼
[UR10e + Tesollo DG-5F]
```

---

## 2. 사전 요건

### 2.1 헤드셋

| 항목 | Galaxy XR (SM-I610) | Meta Quest 3 |
|---|---|---|
| OS | Android 14 (XR) | Horizon OS |
| Vendor ID (USB) | `04e8` (Samsung) | `2833` (Meta) |
| Chrome 버전 | 최신 안정 | 최신 안정 (또는 Wolvic) |
| Cable | USB-C 데이터 (충전전용 케이블 불가) | USB-C 데이터 |
| WebXR + hand-tracking | ✓ | ✓ (xr_teleop Week 1~4 검증) |

### 2.2 조종 PC

| 항목 | 요구 |
|---|---|
| OS | Ubuntu 22.04 (또는 WSL2) |
| Python | 3.10 (conda env `teleop_operator`) |
| 의존성 | `aiohttp>=3.8`, `dex_retargeting`, `numpy<2`, `pinocchio` |
| USB | USB3 권장 (USB2 는 30 Hz 미달 가능) |

### 2.3 로봇 PC

XR 별도 추가 의존성 없음. 기존 teleop_dev 환경 그대로:
- `robot/arm/admittance/main.py` 또는 `robot/arm/impedance/main.py`
- `robot/hand/receiver.py`

---

## 3. 초기 설치 (조종 PC, 1회)

### 3.1 Google 공식 ADB platform-tools

> ⚠️ Ubuntu apt 패키지 (`android-tools-adb`) 는 glibc 2.35 호환성 문제 — 반드시 Google 공식 빌드 사용.

```bash
sudo apt remove --purge android-tools-adb android-tools-fastboot 2>/dev/null || true
cd /tmp
wget -q https://dl.google.com/android/repository/platform-tools-latest-linux.zip
unzip -q platform-tools-latest-linux.zip
sudo mv platform-tools /opt/

grep -q '/opt/platform-tools' ~/.bashrc || \
  echo 'export PATH=/opt/platform-tools:$PATH' >> ~/.bashrc
export PATH=/opt/platform-tools:$PATH

adb --version    # "Version 35.x.x" 표시되면 정상
```

### 3.2 udev rules

```bash
sudo usermod -aG plugdev "$USER"

# Galaxy XR (Samsung VID 04e8)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04e8", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/51-android-samsung.rules

# Meta Quest (VID 2833)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/51-android-meta.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
# 로그아웃 후 재로그인
```

### 3.3 conda env (XR 추가)

```bash
cd /path/to/teleop_dev
conda env update -f environment.yaml --prune
conda activate teleop_operator

# 확인
python3 -c "import aiohttp, dex_retargeting, numpy; print('OK')"
# numpy.__version__ 1.26.4 이어야 함 (pinocchio ABI)
```

### 3.4 PYTHONPATH

```bash
# teleop_dev 가 /workspaces/tamp_ws/src/teleop_dev 라면
export PYTHONPATH="/workspaces/tamp_ws/src:${PYTHONPATH}"
echo 'export PYTHONPATH="/workspaces/tamp_ws/src:${PYTHONPATH}"' >> ~/.bashrc
```

---

## 4. 매번 사용 절차

### 4.1 헤드셋 연결

```bash
# USB-C 연결 후
adb devices
# 결과 예: R3KYA01R62L  device
# (처음 연결 시 USB 디버깅 RSA 키 지문 팝업 → "이 컴퓨터에서 항상 허용" 체크)

# Port forwarding
adb reverse tcp:8013 tcp:8013
adb reverse --list
# 결과: (reverse) UsbFfs tcp:8013 tcp:8013
```

### 4.2 (선택) pose-only 동작 확인 (Gate A)

로봇 PC 미연결 상태에서 헤드셋 ↔ 조종 PC 통신 검증.

```bash
cd /path/to/teleop_dev
conda activate teleop_operator
python3 -m scripts.xr_pose_diag

# 헤드셋 Chrome → http://localhost:8013/ → Enter VR/AR → 손 들이밀기
# PC 콘솔에 1Hz 로그:
# [xr_pose_diag]  60.5 msg/s (head 30.1/s, hand 30.4/s)  head=OK rArm=OK rHand=OK
```

30 초 정량 측정:
```bash
python3 -m scripts.xr_pose_diag --measure 30 \
    --report docs/vr_teleop/xr_spike/A3_diag_result.md
```

### 4.3 통합 실행 (팔 + 손)

```bash
# 로봇 PC 측
python3 -m robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2
python3 -m robot.hand.receiver --hand right

# 조종 PC 측
adb reverse tcp:8013 tcp:8013
python3 -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --scale 0.3
```

헤드셋:
1. Chrome → `http://localhost:8013/` 접속
2. **Enter VR** (또는 **Enter AR** — pass-through 가능)
3. 손을 시야 안에 들이밀기
4. 조종 PC 터미널에서 **`r`** 키 → sync 시작

### 4.4 키 매핑 (조종 PC 터미널)

| 키 | 동작 |
|---|---|
| `r` | sync 시작 또는 recalibrate (사용자 + robot origin 동시 capture) |
| `p` | pause / resume — 손 새 위치로 옮길 때. resume 시 자동 recalibrate |
| `c` | immediate recalibrate (pause 없이, jump 가능) |
| `Space` | E-Stop |
| `+` / `-` | speed scale up/down |
| `x` / `Esc` / `q` | sender 종료 |
| Ctrl+C | 강제 종료 |

손 sender 는 별도 키 처리 없음 — 손동작 자체 (open / fist / pinch) 로 제어.

### 4.5 옵션

```bash
# 손만
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-arm

# 팔만
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-hand

# Left hand (B5, 검증 완료)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --hand left
# robot PC 측: receiver --hand left --port 9874 + dg5f_left_pid_all_controller.launch.py

# 양팔 + 양손 동시 (dual — 팔별 remap/scale/workspace 는 yaml 로)
python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \
    --target-ip 192.168.0.10
# 왼팔 첫 가동 전 필독: docs/xr_dual_arm_left_tuning_ko.md

# 정상속 (검증 후)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 1.0

# convention 변경 (open ↔ fist inversion 시)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --convention manus

# safety 완화 (디버그용 — 운영 시 비권장)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-workspace-clamp
```

전체 옵션: `python3 -m scripts.run_xr_teleop --help`

---

## 5. 카메라 영상 (현재 비활성)

현재 teleop_dev 의 로봇 PC 가 카메라 publisher 를 제공하지 않음. `sender/xr_common/config.yaml` 의 `webrtc.enabled: false` 가 default — 헤드셋 안에서 영상 plane 안 그려짐 (배경만 clear 또는 pass-through).

향후 WebRTC 카메라 publisher 추가 시:
1. `sender/xr_common/config.yaml` 의 `webrtc.enabled` → `true`
2. `webrtc.host` 를 카메라 publisher host 로 변경 (sim/loopback 시 `localhost`)
3. 헤드셋 Chrome 에서 `https://<host>:60001` (head), `60003` (right_wrist) 1회씩 cert 신뢰
4. HTML / Python 측 코드 변경 불필요 (구조 이미 준비됨)

---

## 6. 단계별 검증 (Phase A~F)

각 Phase 의 상세 가이드:

- **Gate A** (pose-only) — [`docs/vr_teleop/xr_spike/A2_test_guide.md`](vr_teleop/xr_spike/A2_test_guide.md)
- **Gate B** (XR 손) — [`docs/vr_teleop/xr_spike/B3_test_guide.md`](vr_teleop/xr_spike/B3_test_guide.md)
- **Gate C** (XR 팔) — [`docs/vr_teleop/xr_spike/C2_test_guide.md`](vr_teleop/xr_spike/C2_test_guide.md)
- **Gate D** (통합) — [`docs/vr_teleop/xr_spike/D1_unified.md`](vr_teleop/xr_spike/D1_unified.md)
- **Gate E** (실로봇) — [`docs/vr_teleop/xr_spike/E2_realrobot.md`](vr_teleop/xr_spike/E2_realrobot.md)

---

## 7. 트러블슈팅

### 7.1 헤드셋 ↔ 조종 PC

| 증상 | 원인 | 해결 |
|---|---|---|
| `adb devices` 가 unauthorized | RSA 지문 미허용 | 헤드셋 안의 USB 디버깅 팝업에서 "이 컴퓨터에서 항상 허용" |
| 헤드셋 Chrome 에서 `http://localhost:8013/` "이 페이지에 도달할 수 없습니다" | `adb reverse` 안 됨 | `adb reverse --list` 확인. 안 보이면 `adb kill-server && adb start-server` |
| Chrome 페이지 떴는데 "Enter VR" 비활성 | WebXR 미지원 (또는 flag 필요) | (Galaxy XR) `chrome://flags/#webxr-incubations` enable + 재시작 |
| 헤드셋 진입했는데 PC 콘솔 `msg_count=0` | ws 연결 실패 | 헤드셋 화면의 Live session log 확인. `ws closed; reconnect in 1s` 메시지 보이면 `adb reverse` 재설정 |
| 30Hz 미달 (15-20Hz) | USB 2.0 / Chrome throttle | USB 3 포트 사용. 다른 USB 디바이스 분리 |

### 7.2 손 (DG-5F)

| 증상 | 원인 | 해결 |
|---|---|---|
| `open hand 인데 fist 자세` | convention mismatch | `--convention manus` 재시도 |
| 손가락 정지 / `tracking=False` 만 | 헤드셋 hand 인식 안 됨 | 손 시야 안에 들이밀기. 옷소매 등 가림 확인. `inputsourceschange: hands=2` 로그 확인 |
| 손가락 진동 / jerky | retarget jitter | receiver `--ema-alpha 0.1` (강한 smoothing) 또는 yaml `low_pass_alpha 0.1` |
| `ImportError: dex_retargeting` | env 미갱신 | `conda env update -f environment.yaml --prune` |

### 7.3 팔 (UR10e)

| 증상 | 원인 | 해결 |
|---|---|---|
| `[Sender] WARNING: Pose query failed` | robot PC 측 미시작 | `python3 -m robot.arm.admittance.main --mode rtde --input unified` 실행 확인 |
| `r` 키 안 받음 | termios 가 키 못 받음 | 터미널 포커스 확인. SSH 세션 인 경우 `--no-keyboard` 옵션 시도 |
| user origin `[0,0,0]` 으로 calibrate | bridge 측 hand pose 미수신 | 손을 시야 안에 둔 채로 `r` 키 |
| 실 UR10e overshoot / oscillate | scale 너무 크거나 admittance gain 너무 강함 | `--scale 0.3` 로 시작. robot 측 admittance config 의 M/K/D 튜닝 |
| `WARN: target pos ... → clamped` 빈번 | workspace envelope 너무 좁음 | `sender/xr_common/watchdog.py` 의 WorkspaceLimits 수정 |
| `WARN: BridgePoseStore stale` 빈번 | 헤드셋 throttle | `--watchdog-timeout-s 0.5` 로 완화 (단 안전 ↓) |

### 7.4 환경 / 설치

| 증상 | 원인 | 해결 |
|---|---|---|
| `ImportError: cannot import name 'RetargetingConfig' from 'dex_retargeting'` | dex_retargeting 0.5.0+ API 변경 | 코드 이미 `from dex_retargeting.retargeting_config import RetargetingConfig` 사용 |
| `port 8013 already in use` | 이전 sender 가 daemon thread 로 남음 | `pkill -f bridge_pose_store` 또는 `--bridge-port 8014` |
| numpy 2.x 로 자동 upgrade | 다른 패키지 install 부작용 | `pip install 'numpy<2' --force-reinstall --no-deps` |

---

## 8. 관련 문서

- [`docs/setup_guide.md`](setup_guide.md) — 전체 환경 설치
- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) — 전체 시스템 구조
- [`docs/arm_input_guide.md`](arm_input_guide.md) — Vive / keyboard / joystick (기존 sender)
- [`docs/hand_manus_guide.md`](hand_manus_guide.md) — Manus / RealSense (기존 sender)
- [`docs/vr_teleop/xr_integration_plan.md`](vr_teleop/xr_integration_plan.md) — 통합 계획서 + 영향 범위
- [`docs/vr_teleop/xr_spike/`](vr_teleop/xr_spike/) — Phase A~E spike + test guide
