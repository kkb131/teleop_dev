# teleop_dev 아키텍처

UR10e 팔 + Tesollo DG5F 손의 원격조종(teleop) 시스템.
조종 PC와 로봇 PC 2-tier 구조로 분리되며, UDP로 통신한다.

---

## 시스템 개요

```
조종 PC (Operator)                         로봇 PC (AGX Orin)
┌──────────────────────┐                  ┌───────────────────────────┐
│  Vive Tracker 3.0    │                  │  UnifiedNetworkInput      │
│  Keyboard / Xbox     │                  │    ↓                      │
│  Galaxy XR / Quest 3 ┼── UDP 9871 ────→ │  ExpFilter → Pink IK      │
│  ┌────────────────┐  │                  │    ↓                      │
│  │ BridgePoseStore│  │                  │  Admittance / Impedance   │
│  │ (ws bridge)    │  │                  │    ↓                      │
│  └────────────────┘  │                  │  UR10e (servoJ / torque)  │
├──────────────────────┤                  ├───────────────────────────┤
│  Manus Gloves        │                  │  Receiver → Retarget     │
│  RealSense D405      │                  │    ↓                      │
│  Galaxy XR / Quest 3 ┼── UDP 9872 ────→ │  Tesollo DG5F (Modbus)   │
└──────────────────────┘                  └───────────────────────────┘
```

XR 헤드셋은 USB-C (`adb reverse`) 로 연결, 자체 ws bridge (port 8013) 가 헤드셋 Chrome → 조종 PC pose 송신. 같은 BridgePoseStore singleton 이 팔 (UDP 9871) 과 손 (UDP 9872) sender 양쪽에 데이터 공급.

---

## 디렉토리 구조

```
teleop_dev/
├── protocol/                          # 통신 프로토콜 (양쪽 PC 공유, stdlib+numpy만)
│   ├── arm_protocol.py                #   UDP 9871: TeleopPosePacket, ButtonState, query/response
│   ├── hand_protocol.py               #   UDP 9872: HandData (20 finger joints)
│   └── cam_protocol.py                #   TCP 9873 (ZMQ) + 8014 (WS): 카메라 JPEG 프레임
│
├── sender/                            # 조종 PC (Windows/Linux, ROS2 불필요)
│   ├── arm/                           #   팔 입력장치 → UDP 9871
│   │   ├── sender_base.py             #     모든 arm sender의 ABC (UDP 전송, 포즈 누적)
│   │   ├── vive_sender.py             #     Vive Tracker 3.0 sender (openvr)
│   │   ├── vive_tracker.py            #     OpenVR wrapper (행렬→포즈 변환)
│   │   ├── vive_config.py             #     Vive YAML 설정 로더
│   │   ├── calibrate.py               #     SteamVR → base_link 좌표 보정
│   │   ├── keyboard_sender.py         #     WASDQE 키보드 sender
│   │   ├── joystick_sender.py         #     Xbox/Logitech 게임패드 sender
│   │   ├── xr_sender.py               #     Galaxy XR / Quest 3 sender (BridgePoseStore 공유)
│   │   ├── xr_frame_align.py          #     WebXR wrist → base_link, relative motion + recalibrate
│   │   ├── monitor.py                 #     UDP 패킷 모니터 (디버깅)
│   │   ├── move_to_pose.py            #     단일 포즈 전송 유틸리티
│   │   ├── config/default.yaml        #     기본 설정
│   │   └── tests/                     #     단계별 테스트 (step1~step6)
│   ├── hand/                          #   손 입력장치 → UDP 9872
│   │   ├── manus_sender.py            #     Manus → UDP sender (60Hz, --retarget {none|ergo-direct|dex})
│   │   ├── manus_reader.py            #     Manus SDK subprocess wrapper (legacy)
│   │   ├── manus_reader_ros2.py       #     manus_data_publisher ROS2 subscriber (권장)
│   │   ├── manus_config.py            #     Manus YAML 설정 로더
│   │   ├── realsense_sender.py        #     RealSense D405 → MediaPipe → [3A] dex_retarget → UDP
│   │   ├── realsense/                 #     RealSense D405 + MediaPipe HandLandmarker 파이프라인
│   │   ├── calibrate.py               #     2-pose (open/fist) ROM 캘리브레이션
│   │   ├── hand_visualizer.py         #     손 관절 시각화 (pygame)
│   │   ├── keyboard_state.py          #     hand sender 공통 키보드 이벤트 (E-Stop/reset/quit/speed)
│   │   ├── core/                      #     세대 공통 인프라
│   │   │   ├── dg5f_config.py         #       DG5F 관절 이름/한계/링크 길이 상수
│   │   │   ├── filters.py             #       EMA 필터 (모든 retarget 세대 공유)
│   │   │   ├── mano_transform.py      #       MANO frame 변환 (manus/mediapipe convention)
│   │   │   └── retarget_base.py       #       HandRetargetBase ABC
│   │   ├── gen1a_ergo_direct/         #     [1A] Manus ergonomics → DG5F 직접 매핑 (단순 스케일)
│   │   ├── gen3a_dex_retarget/        #     [3A] DexPilot/Vector 옵티마이저 wrapper (dex_retargeting)
│   │   ├── xr_hand_sender.py          #     Galaxy XR / Quest 3 hand sender (DexPilot)
│   │   ├── xr_dex_retargeter.py       #     WebXR 25-joint → DG-5F 20-vec retarget wrapper
│   │   ├── xr_remap.py                #     WebXR 25 → wrist-local + palm-aligned MANO frame
│   │   ├── config/default.yaml        #     manus_sender 기본 설정
│   │   ├── config_xr/                 #     XR retarget URDF + meshes + DexPilot yml
│   │   │   ├── dg5f_xr.yml            #       right + left DexPilot config (B5)
│   │   │   ├── dg5f_right_retarget.urdf   #   right URDF (PIP/DIP lower=0)
│   │   │   ├── dg5f_left_retarget.urdf    #   left URDF (PIP/DIP lower=0, B5)
│   │   │   └── meshes/{visual,collision}/ #   rl_dg_* + ll_dg_* (56 + 56 files)
│   │   ├── sdk/                       #     Manus C++ SDK + ROS2 publisher 소스 (사전 빌드 .so)
│   │   └── tests/                     #     단계별 테스트 (step0~step4)
│   ├── cam/                           #   카메라 수신 + 브라우저/VR 서빙 (docs/cam_streaming_guide.md)
│   │   ├── config.py                  #     ZmqConfig / HttpConfig / VrConfig (YAML+CLI)
│   │   ├── config.yaml                #     robot_ip, cameras, VR plane 배치/고정모드
│   │   ├── receiver.py                #     ZMQ SUB (9873) → 카메라별 LatestSlot
│   │   ├── http_server.py             #     aiohttp 8014: /ws /config /snapshot /recenter
│   │   ├── assets/cam_view.html       #     브라우저 그리드 뷰 (WS-JPEG + canvas)
│   │   └── main.py                    #     python3 -m sender.cam.main --robot-ip <IP>
│   └── xr_common/                     #   XR (Galaxy XR / Quest 3) 공통 — arm + hand 가 공유
│       ├── bridge_pose_store.py       #     aiohttp HTTP+WS server, Singleton (port 8013)
│       ├── watchdog.py                #     StoreWatchdog (freshness) + WorkspaceLimits (envelope)
│       ├── config.yaml                #     ws.port / cam.enabled (영상 통합) / render plane size
│       └── assets/webxr_to_pose.html  #     headset Chrome WebXR client (pose 송신 + 카메라 plane 렌더)
│
├── robot/                             # 로봇 PC (AGX Orin, ROS2 + ur-rtde)
│   ├── config.py                      #   공유 상수: JOINT_NAMES, URDF_PATH, RTDE_FREQUENCY 등
│   ├── core/                          #   공유 인프라 (IK, 필터, 로봇 백엔드)
│   │   ├── robot_backend.py           #     RobotBackend ABC + create_backend("rtde"|"sim")
│   │   ├── ur_robot.py                #     RTDEBackend (실제 로봇, ur_rtde servoJ 125Hz)
│   │   ├── sim_robot.py               #     SimBackend (Mock hw / Isaac Sim, ROS2 토픽)
│   │   ├── trajectory_executor.py     #     궤적 리샘플링 (40Hz→125Hz) + 실시간 스트리밍
│   │   ├── controller_utils.py        #     ControllerSwitcher (ros2_control 전환)
│   │   ├── kinematics.py              #     PinocchioIK (FK/Jacobian/DLS IK)
│   │   ├── pink_ik.py                 #     Pink QP IK (관절 한계 보장, proxqp)
│   │   ├── exp_filter.py              #     지수 필터 (위치 EMA + 자세 slerp)
│   │   ├── input_handler.py           #     입력 추상화 (Keyboard/Xbox/UnifiedNetworkInput)
│   │   ├── ft_source.py               #     F/T 센서 추상화 (RTDE / Null)
│   │   └── compliant_control.py       #     어드미턴스 동역학 (M·ẍ + D·ẋ + K·x = F)
│   ├── arm/                           #   팔 제어 모드
│   │   ├── admittance/                #     모드: 어드미턴스 (servoJ + F/T 피드백)
│   │   │   ├── main.py                #       엔트리포인트, 메인 제어 루프
│   │   │   ├── admittance_layer.py    #       F/T → 변위 보정
│   │   │   ├── safety_monitor.py      #       4단계 안전 (E-Stop, 워크스페이스, 속도, 타임아웃)
│   │   │   ├── teleop_config.py       #       YAML 설정 로더 (dataclass)
│   │   │   ├── config/default.yaml    #       기본 설정
│   │   │   └── docs/                  #       이론, 설정 가이드, 매뉴얼
│   │   ├── impedance/                 #     모드: 임피던스 (URScript PD 토크 500Hz)
│   │   │   ├── main.py                #       엔트리포인트, 듀얼 루프 디스패치
│   │   │   ├── urscript_manager.py    #       URScript 업로드 + RTDE 레지스터 I/O
│   │   │   ├── impedance_gains.py     #       PD 게인 프리셋 (STIFF/MEDIUM/SOFT)
│   │   │   ├── torque_safety.py       #       토크 안전 검사 (위치 편차, 속도, 타임아웃)
│   │   │   ├── impedance_config.py    #       YAML 설정 로더 (dataclass)
│   │   │   ├── scripts/               #       URScript 파일 (impedance_pd.script)
│   │   │   ├── config/default.yaml    #       기본 설정
│   │   │   └── docs/                  #       이론, 구현 노트, 튜닝 가이드
│   │   └── servo/                     #     모드: 단순 DLS IK (로컬 전용, 네트워크 불필요)
│   │       ├── keyboard_cartesian.py  #       키보드 Cartesian 제어
│   │       ├── keyboard_forward.py    #       키보드 Joint-space 제어
│   │       ├── joystick_cartesian.py  #       Xbox Cartesian 제어
│   │       ├── keyboard_servo_admittance.py  # 키보드 + F/T 어드미턴스
│   │       └── docs/                  #       매뉴얼, 사용 가이드
│   ├── hand/                          #   손 제어 (UDP 수신 → ROS2 토픽 publish)
│   │   ├── dg5f_ros2_client.py        #     ROS2 MultiDOFCommand (real) / JointState (sim) publisher
│   │   ├── receiver.py                #     UDP 9872 수신 → EMA filter → ROS2 publish (sender 측에서 retarget 완료)
│   │   └── tests/                     #     ROS2 PID 튜닝, zero-pose, direct-pub, 프리셋 포즈 테스트
│   └── cam/                           #   D405 컬러 스트리밍 (docs/cam_streaming_guide.md)
│       ├── config.py                  #     StreamConfig / CameraEntry (YAML+CLI)
│       ├── config/default.yaml        #     카메라 개수/시리얼/해상도/fps/JPEG 품질
│       ├── rs_color_camera.py         #     RSColorCamera (pyrealsense2, 컬러 전용)
│       ├── synthetic_camera.py        #     합성 카메라 (D405 없이 테스트)
│       ├── publisher.py               #     CamZmqPublisher (PUB 9873) + CaptureWorker
│       ├── list_cameras.py            #     연결된 D405 시리얼 나열
│       └── main.py                    #     python3 -m robot.cam.main
│
├── scripts/                           # entry-point 스크립트
│   ├── run_xr_teleop.py               #   XR 통합 launcher (arm + hand 동시, BridgePoseStore 공유)
│   └── xr_pose_diag.py                #   XR pose-only diagnostic (mean_freq / lost / recovery / jitter)
│
├── docs/                              # 문서
│   ├── ARCHITECTURE.md                #   이 파일
│   ├── setup_guide.md                 #   conda env / Docker / 의존성 설치
│   ├── arm_input_guide.md             #   Vive / keyboard / joystick 사용 가이드
│   ├── hand_manus_guide.md            #   Manus glove / RealSense 사용 가이드
│   ├── xr_input_guide.md              #   Galaxy XR / Quest 3 사용 가이드 (XR sender)
│   ├── cam_streaming_guide.md         #   D405 카메라 스트리밍 가이드 (robot/cam + sender/cam)
│   └── vr_teleop/                     #   XR 통합 plan + phase 별 spike + test guide
│
└── environment.yaml                   # 조종 PC conda 환경
```

---

## 계층 구조 원칙

| 질문 | 답 | 해당 경로 |
|------|---|----------|
| 이 코드 어디서 실행? | `sender/` = 조종 PC, `robot/` = 로봇 PC | 1차 분류 |
| 어떤 서브시스템? | `arm/` = UR10e 팔, `hand/` = DG5F 손 | 2차 분류 |
| 통신 규격은? | `protocol/` = 양쪽 공유 wire format | 독립 모듈 |

---

## 통신 프로토콜

### arm_protocol (UDP 9871)

팔 원격조종용 통합 프로토콜. JSON over UDP.

**Operator → Robot** (`teleop_pose`):
```json
{
  "v": 1,
  "type": "teleop_pose",
  "pos": [0.1, -0.4, 0.3],
  "quat": [1.0, 0.0, 0.0, 0.0],
  "buttons": {"estop": false, "reset": false, "quit": false, ...},
  "gripper": 0.0,
  "timestamp": 1711180800.123
}
```

**초기 핸드셰이크**: Operator가 `{"type": "query_pose"}` 전송 → Robot이 현재 TCP 포즈 응답.

### hand_protocol (UDP 9872)

손 원격조종용 프로토콜. JSON over UDP.

**Operator → Robot**: Manus 글러브 20관절 각도 + 손목 포즈.

**쿼터니언 규약**: Operator(Vive)는 `wxyz`, Robot(Pinocchio)는 `xyzw` — `input_handler.py`에서 변환.

### cam_protocol (ZMQ TCP 9873 + WS TCP 8014)

D405 컬러 영상 스트리밍. 방향이 다른 프로토콜과 반대 (Robot → Operator).

**Robot → Operator** (ZMQ PUB-SUB, multipart): `[topic(카메라이름), header JSON, JPEG]`
**Operator → 브라우저/헤드셋** (WS binary): 12-byte 헤더 (version, cam index, seq, ts) + JPEG

상세: `docs/cam_streaming_guide.md`

---

## 팔 제어 모드 비교

| | Admittance | Impedance | Servo |
|---|---|---|---|
| 제어 변수 | 위치 (servoJ) | 토크 (direct_torque) | 위치 (servoJ) |
| IK | Pink QP | Pink QP | Pinocchio DLS |
| 제어 주기 | 125Hz (Python) | 500Hz (URScript) | 50Hz (Python) |
| 외력 대응 | F/T 센서 기반 | PD 게인 (물리적) | 없음 |
| 안전 시스템 | 4단계 Safety | Torque Safety | 없음 |
| 네트워크 입력 | UnifiedNetworkInput | UnifiedNetworkInput | 로컬 키보드 전용 |
| 용도 | 실전 teleop | 실전 teleop (접촉 작업) | 학습/디버깅 |

---

## 실행 명령

### 조종 PC (Operator)

```bash
# Vive 트래커 → 팔 teleop
python3 -m sender.arm.vive_sender --target-ip 10.0.0.5

# 키보드 → 팔 teleop
python3 -m sender.arm.keyboard_sender --target-ip 10.0.0.5

# Manus 글러브 → 손 teleop
python3 -m sender.hand.manus_sender --target-ip 10.0.0.5

# Galaxy XR / Quest 3 → 팔 + 손 teleop (통합)
python3 -m scripts.run_xr_teleop --target-ip 10.0.0.5 --scale 0.3

# Galaxy XR / Quest 3 → 팔만
python3 -m sender.arm.xr_sender --target-ip 10.0.0.5 --scale 0.3

# Galaxy XR / Quest 3 → 손만
python3 -m sender.hand.xr_hand_sender --target-ip 10.0.0.5

# 카메라 수신 + 브라우저/VR 서빙 (브라우저: http://localhost:8014/)
python3 -m sender.cam.main --robot-ip 10.0.0.5
```

### 로봇 PC (AGX Orin)

```bash
# 팔: 어드미턴스 모드 (네트워크 입력)
python3 -m robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2

# 팔: 임피던스 모드 (네트워크 입력, PolyScope 5.23.0+ 필수)
python3 -m robot.arm.impedance.main --mode rtde --input unified --robot-ip 192.168.0.2

# 팔: 단순 servo (로컬 키보드, 네트워크 불필요)
python3 -m robot.arm.servo.keyboard_cartesian --mode sim

# 손: Manus 수신 → DG5F 제어
python3 -m robot.hand.receiver --hand-ip 169.254.186.72

# 카메라: D405 1~3대 컬러 스트리밍 (ZMQ PUB 9873)
python3 -m robot.cam.list_cameras          # 시리얼 확인
python3 -m robot.cam.main                  # config/default.yaml 기준
```

---

## 의존성

### 조종 PC (sender/)

| 패키지 | 용도 |
|--------|------|
| numpy | 수치 연산, 포즈 변환 |
| openvr | Vive Tracker SteamVR API |
| pynput | 키보드 상태 읽기 |
| pygame (optional) | 게임패드 입력 |
| Manus SDK (.so) | 글러브 데이터 읽기 |
| aiohttp (XR) | Galaxy XR / Quest 3 ws bridge (xr_common) |
| dex_retargeting (XR / hand) | WebXR 25-joint → DG-5F retargeting |
| mediapipe / pyrealsense2 / opencv-python | RealSense + MediaPipe HandLandmarker (별도 sender) |

### 로봇 PC (robot/)

| 패키지 | 용도 |
|--------|------|
| numpy (<2.0) | 수치 연산 (pinocchio ABI 호환) |
| pinocchio | FK/Jacobian/IK (apt: ros-humble-pinocchio) |
| pin-pink | QP 기반 IK (proxqp 백엔드) |
| proxsuite | QP solver |
| ur-rtde | UR10e RTDE 통신 (servoJ / URScript 업로드) |
| rclpy | ROS2 (SimBackend, ControllerSwitcher, DG5F MultiDOFCommand) |
| PyYAML | YAML 설정 파싱 |

---

## import 규약

```python
# 프로토콜 (양쪽 PC)
from protocol.arm_protocol import TeleopPosePacket, ButtonState
from protocol.hand_protocol import HandData, NUM_JOINTS

# 조종 PC
from sender.arm.sender_base import TeleopSenderBase
from sender.arm.vive_tracker import ViveTracker

# 로봇 PC
from robot.config import JOINT_NAMES, URDF_PATH
from robot.core.robot_backend import create_backend
from robot.core.pink_ik import PinkIK
from robot.hand.dg5f_ros2_client import DG5FROS2Client
```
