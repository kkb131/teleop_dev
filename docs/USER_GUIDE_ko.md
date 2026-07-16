# 통합 사용자 가이드 — Galaxy XR 양팔·양손 원격조종 시스템

> **이 문서 하나로**: 시스템이 무엇으로 구성되고, 어떻게 켜고, 어떻게 조종하고,
> 문제가 생기면 어느 문서를 보면 되는지.
> 메인 조종 수단은 **Galaxy XR (또는 Quest 3) 헤드셋**이며, Vive/키보드/Manus
> 등은 대안 입력이다 (§8).

---

## 1. 시스템 한눈에

```
        [Galaxy XR 헤드셋]  ← 메인 조종 (손 추적 + 손목 pose)
            │ USB-C (adb reverse 8013/8014)
            ▼
        [조종 PC]───────────────── 브라우저 → 로봇 웹 대시보드 (시작/정지/상태/E-STOP)
            │  BridgePoseStore(8013) → arm sender ×2 + hand sender ×2
            │
            │ UDP: 9871(오른팔) 9875(왼팔) 9872(오른손) 9874(왼손)
            │ ZMQ 9873 ← 카메라 영상 (robot → 조종)
            │ HTTP 9876 → 로봇 웹 대시보드 (launcher.robot.web)
            ▼
        [robot PC]
            ├─ arm 수신부 ×2 ──RTDE servoJ──→ UR10e 오른팔(192.168.0.2) / 왼팔(미정)
            ├─ hand 수신부 ×2 ─ROS2─→ dg5f_driver ─Modbus─→ DG5F 오른손(.72)/왼손(.73)
            └─ cam 서버 ── D405 카메라(들)
```

동작 원리 (팔): 'r' 캘리브레이션 순간의 손목 위치와 로봇 TCP 를 짝으로 잡고,
이후 **손목 이동량만** 로봇 target 으로 보낸다 (상대 매핑 — 절대 위치 아님).
손: WebXR 25관절 → DexPilot retarget → DG5F 20관절.

### 전체 IP / 포트 표

| 무엇 | 값 | 비고 |
|------|-----|------|
| robot PC | 192.168.0.10 | 조종측 xr_dual.yaml `network.robot_pc_ip` |
| 오른팔 UR10e | 192.168.0.2 | |
| 왼팔 UR10e | **미정** (placeholder 192.168.0.3) | 확정 시 [robot.yaml](../launcher/robot/config/robot.yaml) + [left.yaml](../robot/arm/admittance/config/left.yaml) 수정 |
| 오른손 DG5F | 169.254.186.72:502 | Modbus TCP |
| 왼손 DG5F | 169.254.186.73:502 | Modbus TCP |
| 헤드셋 pose 브리지 | 조종 PC 8013 (WS) | adb reverse |
| 카메라 뷰 (헤드셋/브라우저) | 조종 PC 8014 (WS) | adb reverse |
| 오른팔 / 왼팔 명령 | robot PC 9871 / 9875 (UDP) | |
| 오른손 / 왼손 명령 | robot PC 9872 / 9874 (UDP) | |
| 카메라 영상 | robot PC 9873 (ZMQ PUB) | robot → 조종 |
| 런처 웹 대시보드 | robot PC 9876 (HTTP) | 조종 PC 브라우저로 접속 |

---

## 2. 최초 1회 설치

| 대상 | 할 일 | 문서 |
|------|------|------|
| 조종 PC | conda env + 의존성, adb 설치 | [setup_guide.md](setup_guide.md) |
| 조종 PC ↔ 헤드셋 | USB-C 연결, `adb devices` 승인 | [xr_input_guide.md](xr_input_guide.md) §2 |
| robot PC | ROS2 humble + 워크스페이스 빌드, ur_rtde/pinocchio/pink | [setup_guide.md](setup_guide.md) |
| 현장 값 반영 | 왼팔 IP / DG5F IP / 경로 | [robot.yaml](../launcher/robot/config/robot.yaml) 상단 주석 |

---

## 3. 표준 기동 절차

> 로봇측 상세: [launcher_guide_ko.md](launcher_guide_ko.md)

```bash
# ① robot PC — 웹 대시보드 데몬 (tmux/systemd 상시 실행 권장)
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m launcher.robot.web
```

② 조종 PC **브라우저** → `http://<robot_pc_ip>:9876/` →
파츠 카드(팔/손/캠)의 HW dot 초록 확인 → **▶ 전체 시작**
(드라이버→수신부 순서는 런처가 보장. 필요 없는 파츠는 개별 정지).

```bash
# ③ 조종 PC — XR sender (터미널)
cd /workspaces/tamp_ws/src/teleop_dev
adb reverse tcp:8013 tcp:8013 && adb reverse tcp:8014 tcp:8014
python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \
    --target-ip <robot_pc_ip>
```

④ 헤드셋 Chrome → `http://localhost:8013/` → **Enter VR/AR** → 손을 시야에.

⑤ **캘리브레이션**: 표준 자세(§4)에서 sender 터미널에 **r**.
이 순간부터 손 이동이 로봇으로 전달된다.

⑥ 종료: sender 에서 `x` → 대시보드 **■ 전체 정지**.

비상 시: 대시보드 우상단 적색 **E-STOP** (양팔 즉시 정지 + 손 수신부
정지 — 소프트 스톱이며 물리 E-Stop 을 대체하지 않음).

---

## 4. 조종 방법 (Galaxy XR)

### ⚠️ 캘리브레이션 표준 자세 — 가장 중요한 운용 규칙

`r` 을 누르는 순간의 **헤드셋 정면 방향이 좌표계의 기준**이 된다.
항상 **오른팔 base_link 정면(+x)** 을 바라보는 같은 자세에서 Chrome 을 띄우고
`r` 을 누른다. 다른 방향을 보고 `r` 을 누르면 모든 손 동작이 그 각도만큼
돌아간 채로 로봇에 전달된다 (양팔 모두).

### 키 (조종 PC 의 xr-dual-teleop 터미널)

| 키 | 동작 |
|----|------|
| `r` | sync 시작 / 재캘리브레이션 — **양팔 동시에** origin 캡처 |
| `p` | pause ↔ resume (resume 시 자동 재캘리) — 손을 편하게 옮길 때 |
| `c` | 즉시 재캘리 (pause 없이 — 로봇 jump 가능, 주의) |
| `Space` | **E-Stop** (양팔 즉시 정지) |
| `+` / `-` | robot 측 속도 프리셋 증감 |
| `x` / `q` / Esc | 전체 종료 |

### 운용 요령

- 손이 카메라 시야를 벗어나면 sender 가 송신을 멈추고 로봇은 정지 상태를
  유지한다 (watchdog). 손을 다시 시야에 넣으면 이어진다.
- 큰 이동이 필요하면 `p` → 손 이동 → `p` (resume 재캘리) 패턴 사용.
- 카메라 뷰: 조종 PC 브라우저 `http://localhost:8014/`, 헤드셋에서는 VR
  화면 안 plane 으로 표시 ([cam_streaming_guide.md](cam_streaming_guide.md)).

---

## 5. 양팔 운용과 왼팔 첫 가동

- 손(양손)은 검증 완료. **왼팔은 미검증** — 미러/각도 장착이라 좌표
  remap 을 실측으로 확정해야 한다.
- **왼팔 첫 가동 전 필독**: [xr_dual_arm_left_tuning_ko.md](xr_dual_arm_left_tuning_ko.md)
  (안전 설정 → `--only-arm left` 축별 검증 → remap 확정 → 정상 운용 전환).
- 팔별 설정 위치:
  - sender 측 (scale/remap/workspace): [scripts/config/xr_dual.yaml](../scripts/config/xr_dual.yaml)
  - robot 측 (IP/포트/안전한계/home): [robot/arm/admittance/config/right.yaml](../robot/arm/admittance/config/right.yaml), [left.yaml](../robot/arm/admittance/config/left.yaml)
- 한쪽 팔만 쓰는 날: 대시보드에서 반대쪽 팔/손 카드를 정지, sender 는
  `xr_dual.yaml` 의 해당 `enabled: false` 또는 `--only-arm`.

---

## 6. 일상 종료 / 재시작

| 상황 | 조작 |
|------|------|
| 작업 종료 | sender `x` → 대시보드 ■ 전체 정지 (웹 데몬은 계속 떠 있어도 무방) |
| sender 만 재시작 | 터미널에서 `x` 종료 → 재실행 → 헤드셋 새로고침 → `r` |
| 로봇 급정지 | sender `Space` / 대시보드 **E-STOP** (소프트) 또는 티치펜던트 E-Stop (하드) |
| E-Stop 해제 후 | 대시보드에서 해당 팔 로그 확인 → sender 에서 `r` 재캘리 (또는 팔 카드 재시작) |
| 초기 위치 이동 | 대시보드 팔 카드 `초기 위치 이동` (수신부 정지 상태에서 — [launcher_guide_ko.md](launcher_guide_ko.md) §6) |

---

## 7. 문제가 생기면 (증상 → 문서)

| 증상 | 먼저 볼 곳 |
|------|-----------|
| 대시보드 접속 안 됨 / HW dot 빨강 | [launcher_guide_ko.md](launcher_guide_ko.md) §10 (데몬/방화벽/전원) |
| 파츠가 빨강(error) | 로그 확인 → [launcher_guide_ko.md](launcher_guide_ko.md) §10 |
| 헤드셋 접속/Enter VR 안 됨, pose 안 들어옴 | [xr_input_guide.md](xr_input_guide.md) (adb reverse, Chrome 플래그) |
| 로봇이 손과 다른 방향으로 움직임 (왼팔) | [xr_dual_arm_left_tuning_ko.md](xr_dual_arm_left_tuning_ko.md) §5 처방표 |
| 모든 동작이 일정 각도 돌아감 | §4 표준 자세로 `r` 재캘리 |
| 손가락 동작이 이상함 (오므림/벌림 반전 등) | [xr_input_guide.md](xr_input_guide.md) (convention), [hand_manus_guide.md](hand_manus_guide.md) |
| 카메라 영상 안 나옴 | [cam_streaming_guide.md](cam_streaming_guide.md) |
| 팔 제어 품질 (떨림/반응성/외력) | robot/arm/admittance/docs/, impedance/docs/ (튜닝) |

---

## 8. 대안 입력 (Galaxy XR 없이)

| 입력 | 용도 | 실행 | 문서 |
|------|------|------|------|
| 키보드 | 디버그/데모 | `python3 -m sender.arm.keyboard_sender --target-ip <IP>` | [arm_input_guide.md](arm_input_guide.md) |
| Vive Tracker | 팔 6DoF | `python3 -m sender.arm.vive_sender --target-ip <IP>` | [arm_input_guide.md](arm_input_guide.md) |
| 게임패드 | 팔 | `python3 -m sender.arm.joystick_sender --target-ip <IP>` | [arm_input_guide.md](arm_input_guide.md) |
| Manus 글러브 | 손 | `python3 -m sender.hand.manus_sender --target-ip <IP>` | [hand_manus_guide.md](hand_manus_guide.md) |
| RealSense D405 + MediaPipe | 손 (비착용) | `python3 -m sender.hand.realsense_sender --target-ip <IP>` | [hand_manus_guide.md](hand_manus_guide.md) |

robot PC 측 수신부는 입력과 무관하게 동일 — 런처로 띄운 상태에서 sender 만
바꿔치기하면 된다 (같은 포트 사용 시 XR sender 는 정지할 것).

---

## 9. 부록 — 런처 없이 수동 실행 (전체 명령)

기존 단일팔 플로우는 그대로 유지된다. 터미널별 수동 실행:

```bash
### robot PC ###
# 손 드라이버 (좌/우)
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72
ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py  delto_ip:=169.254.186.73
# 손 수신부 (좌/우)
python3 -m robot.hand.receiver --hand right --port 9872
python3 -m robot.hand.receiver --hand left  --port 9874
# 팔 수신부 (좌/우)
python3 -m robot.arm.admittance.main --mode rtde --input unified \
    --config robot/arm/admittance/config/right.yaml
python3 -m robot.arm.admittance.main --mode rtde --input unified \
    --config robot/arm/admittance/config/left.yaml
# 카메라
python3 -m robot.cam.main

### 조종 PC ###
adb reverse tcp:8013 tcp:8013 && adb reverse tcp:8014 tcp:8014
# 양팔+양손 (신규)
python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \
    --target-ip 192.168.0.10
# 단일팔+단일손 (기존 방식 — 변경 없음)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3
# 카메라 뷰어
python3 -m sender.cam.main --robot-ip 192.168.0.10
```

### 문서 지도

| 문서 | 내용 |
|------|------|
| **USER_GUIDE_ko.md (이 문서)** | 최상위 — 기동/조종/문제 분류 |
| [launcher_guide_ko.md](launcher_guide_ko.md) | 로봇 웹 대시보드/브링업 상세 |
| [xr_dual_arm_left_tuning_ko.md](xr_dual_arm_left_tuning_ko.md) | 왼팔 좌표계 튜닝/트러블슈팅 |
| [xr_input_guide.md](xr_input_guide.md) | XR 헤드셋 연결/조작 상세 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 개발자용 구조/프로토콜 |
| [setup_guide.md](setup_guide.md) | 환경 설치 |
| [cam_streaming_guide.md](cam_streaming_guide.md) | 카메라 스트리밍 |
| [arm_input_guide.md](arm_input_guide.md) / [hand_manus_guide.md](hand_manus_guide.md) | 대안 입력 |
