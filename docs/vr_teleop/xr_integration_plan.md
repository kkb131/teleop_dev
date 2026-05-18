# XR 조종 통합 계획서 — Galaxy XR (or Quest 3) → teleop_dev (UR10e + DG-5F)

> 작성: 2026-05-18
> 대상: teleop_dev 시스템에 Samsung Galaxy XR / Meta Quest 3 헤드셋 기반 XR 조종 기능을 추가
> 출처 분석: [`src/teleop_dev/docs/`](.) (4개 문서) + [`src/xr_teleop/docs/`](../../xr_teleop/docs/) (19개 문서)

---

## 1. 통합 방향 결정 — 한 줄 요약

> **teleop_dev 의 분리 아키텍처(조종 PC ↔ 로봇 PC, UDP 9871/9872)를 그대로 유지하고, xr_teleop 의 검증된 XR 코드(televuer / BridgePoseStore / dex_retargeting / mano_transform)를 teleop_dev 의 새 sender 로 이식한다.**

robot PC 측 코드(ur_rtde admittance/impedance, DG-5F receiver)는 **수정 없음**. 새 작업 단위는 모두 `sender/` 안에서 발생.

### 1.1 양쪽 시스템 비교

| 항목 | teleop_dev | xr_teleop |
|---|---|---|
| 아키텍처 | 조종 PC ↔ 로봇 PC 분리 (UDP) | 단일 PC (DDS, sim 전용) |
| 팔 제어 | ur_rtde servoJ 125Hz + Pink IK | DDS rt/lowcmd publisher (sim 전용) |
| 손 제어 | UDP 9872 → Modbus TCP (실로봇) | DDS rt/dg5f/cmd (sim 전용) |
| 입력장치 | Vive / Keyboard / Joystick / Manus / RealSense | Quest 3 (vuer) / Galaxy XR (ws bridge) |
| **실로봇 검증** | ✅ 완료 | ❌ sim only |
| **XR 검증** | ❌ 없음 | ✅ Gate 4 통과 (Quest 3 + Galaxy XR sim) |

### 1.2 왜 "양방향 결합" 대신 "XR sender 단방향 이식" 인가

사용자가 제안한 **②안 (xr_teleop 기반으로 실로봇 코드 추가)** 도 가능은 하지만:

1. xr_teleop 의 `ur10e_arm_controller.py` 는 DDS rt/lowcmd 발행기 — 실로봇은 ur_rtde servoJ 125Hz 라 **재작성 필요**. teleop_dev 의 `RTDEBackend` 가 이미 그 일을 한다.
2. xr_teleop 의 `dg5f_controller.py` 는 DDS rt/dg5f/cmd 발행기 — 실로봇은 Modbus TCP. teleop_dev 의 `dg5f_ros2_client.py + receiver.py` 가 이미 그 일을 한다.
3. teleop_dev 의 `protocol/arm_protocol.py + hand_protocol.py` 는 wire format 추상화 계층 — XR sender 한 개만 더 만들면 robot PC 측에 **영향 없음**.
4. xr_teleop 의 [remote_teleop_office_to_field_analysis.md §3.2](../../xr_teleop/docs/remote_teleop_office_to_field_analysis.md) 가 권장하는 "Thick Relay" 구조 = teleop_dev 의 분리 구조.

### 1.3 통합 후 데이터 흐름

```
[Galaxy XR / Quest 3]
   │ WebXR (25-joint hand + wrist pose + head pose)
   ▼  USB adb reverse (tcp:8013 ws bridge or tcp:8012 vuer)
[조종 PC — teleop_dev/sender/]
   ├─ xr_pose_source.py        (BridgePoseStore wrapper, single source of truth)
   ├─ sender/arm/xr_sender.py  ─ UDP 9871 ─→ wrist (4×4) → query_pose 핸드셰이크 → 누적 pose
   └─ sender/hand/xr_hand_sender.py ─ UDP 9872 ─→ 25-joint → mano transform → DexPilot → 20-vec
       │ UDP (JSON, protocol/ 변경 없음)
       ▼
[로봇 PC — teleop_dev/robot/] (수정 없음)
   ├─ robot/arm/admittance|impedance/main.py + UnifiedNetworkInput
   └─ robot/hand/receiver.py + DG5FROS2Client
       │ ur_rtde servoJ 125Hz / Modbus TCP 100Hz
       ▼
[UR10e + Tesollo DG-5F]
```

---

## 2. 전체 작업 단위 (Phase A~F)

> 각 unit 은 **3시간~1일** 이내 끝나는 단위. unit 마다 `docs/xr_spike/<unit>.md` 작성 + commit.
> "단위 테스트" 와 "smoke test" 둘 다 추가, 실로봇 없이 단계 검증 가능하게.

### Phase A — XR 환경 준비 + pose-only 검증

조종 PC 에서 XR 헤드셋이 hand/wrist pose 를 안정적으로 발행하는지 확인. xr_teleop 의 검증된 단계 재현.

#### A1. teleop_dev 에 XR 환경 추가 (`environment.yaml` 갱신)
- **목적**: 기존 `teleop_operator` conda env 에 XR sender 의존성 추가
- **변경 파일**: [`src/teleop_dev/environment.yaml`](../environment.yaml)
- **추가 의존성**: `aiohttp` (BridgePoseStore ws server), `PyYAML` (이미 있음), `dex_retargeting` (이미 있음)
- **검증**: `conda env update -f environment.yaml` 후 `python -c "import aiohttp, dex_retargeting"` 성공
- **spike**: `docs/xr_spike/A1_env.md` — 환경 갱신 절차 + 의존성 충돌 검증

#### A2. xr_teleop 검증 자산 이식 (`sender/xr_common/`)
- **목적**: Galaxy XR ws bridge HTML + frame transform 코드를 teleop_dev 안으로 가져오기
- **새 파일**:
  - `sender/xr_common/__init__.py`
  - `sender/xr_common/bridge_pose_store.py` ← xr_teleop/scripts/bridge_pose_store.py 복사 (수정: TeleVuer mimick 인터페이스만 유지, vuer-specific 인자 노이즈 제거 가능)
  - `sender/xr_common/webxr_to_pose.html` ← xr_teleop/assets/webxr_to_pose.html 복사
  - `sender/xr_common/config.yaml` ← ws port + WebRTC port 설정
- **참조 (수정 없음)**: [xr_teleop/scripts/bridge_pose_store.py](../../xr_teleop/scripts/bridge_pose_store.py), [xr_teleop/assets/webxr_to_pose.html](../../xr_teleop/assets/webxr_to_pose.html)
- **검증 1 (단위)**: `python -m sender.xr_common.bridge_pose_store --selftest` → ws server 부팅 + smoke 메시지 수신
- **검증 2 (수동)**: 헤드셋에서 `http://localhost:8013/` 접속, "Enter VR" + 손 들이밀기 → 콘솔에 head/hand msg count 증가
- **spike**: `docs/xr_spike/A2_bridge.md` — port 설정 + adb reverse 명령 + 두 헤드셋 (Galaxy XR / Quest 3) 사용 방법 비교

#### A3. pose-only diagnostic 스크립트 (`scripts/xr_pose_diag.py`)
- **목적**: 헤드셋 ↔ 조종 PC 간 30Hz 안정 수신 정량 확인 (xr_teleop Gate 2 재현)
- **새 파일**: `src/teleop_dev/scripts/xr_pose_diag.py`
- **기능**:
  - `--measure 30` : 30초 측정, mean_freq_hz / nan_per_field / lost_per_field / recovery_latency_s / wrist_jitter_cm 출력
  - `--report <md>` : markdown 표 append
- **참조 패턴**: [xr_teleop/scripts/test_pose_only_ws.py](../../xr_teleop/scripts/test_pose_only_ws.py)
- **검증**: 30초 측정 후 mean_freq ≥ 30 Hz, lost_frames < 5%
- **spike**: `docs/xr_spike/A3_diag.md` — 측정 결과 + 헤드셋 종류별 차이 (Galaxy XR jitter ↑ 예상)

### Phase B — XR 손 sender (UDP 9872) — 로봇 PC 손만 먼저 검증

손은 retargeting 만으로 동작, 팔 IK 없어 안전. 손 동작 검증 후 팔로 진행.

#### B1. 25-joint → 21-joint 어댑터 + frame transform (`sender/hand/xr_remap.py`)
- **목적**: WebXR 25 → MANO 21 reshape 또는 25 그대로 + wrist-local frame transform
- **새 파일**: `src/teleop_dev/sender/hand/xr_remap.py`
- **내용**:
  - `webxr_to_wrist_local_mano(kp_25)` — [xr_teleop/scripts/dg5f_controller.py](../../xr_teleop/scripts/dg5f_controller.py#L107) 의 함수 그대로 복제
  - `_estimate_wrist_frame_webxr(kp_25)` — 같은 곳, palm-plane SVD 패턴
  - mediapipe / manus convention 선택 가능 (사용자가 visual 확인 후 결정)
- **참조 (변경 없음)**: [sender/hand/core/mano_transform.py](../sender/hand/core/mano_transform.py) — `apply_mano_transform` 의 21-joint 버전. 25→21 변환 헬퍼는 본 파일이 자체 구현
- **검증 1 (단위)**:
  - 손목 yaw 90° 회전한 dummy hand_pos vs 원래 hand_pos → 두 입력의 wrist_rot 적용 후 결과가 atol=1e-3 로 같음
  - 손가락 굽힘 변화한 dummy hand_pos → 결과의 finger keypoint 차이가 회전 입력보다 큼
- **spike**: `docs/xr_spike/B1_remap.md` — frame transform 패턴 출처 (retarget_dev) + WebXR 25 인덱스 매핑

#### B2. DexPilot retargeter wrapper (`sender/hand/xr_dex_retargeter.py`)
- **목적**: 25-joint hand_pos → DG-5F 20-vec joint angles. xr_teleop 의 retargeter 호출 흐름 복제
- **새 파일**: `src/teleop_dev/sender/hand/xr_dex_retargeter.py`
- **내용**:
  - `XRDexRetargeter(yml_path=..., assets_dir=..., convention="mediapipe")` class
  - `retarget(kp_25) -> np.ndarray (20,)` : webxr_to_wrist_local_mano → ref_value → DexPilot retarget → expand_retarget_to_dg5f_20 (target_joint_names 길이가 20이면 그대로)
  - 내부적으로 `dex_retargeting.RetargetingConfig.from_dict(cfg).build()`
- **변경 없음 참조**:
  - [xr_teleop/scripts/dg5f_controller.py](../../xr_teleop/scripts/dg5f_controller.py#L302) — `_control_process` 의 retarget 로직
  - [xr_teleop/assets/dg5f_hand/dg5f_right.yml](../../xr_teleop/assets/dg5f_hand/dg5f_right.yml) — DexPilot 설정
  - [xr_teleop/assets/dg5f_hand/dg5f_right_retarget.urdf](../../xr_teleop/assets/dg5f_hand/dg5f_right_retarget.urdf) — PIP/DIP lower=0 retarget URDF
- **새 자산**:
  - `src/teleop_dev/sender/hand/config/dg5f_right_xr.yml` ← `dg5f_right.yml` 복사 (urdf_path 만 상대 경로 조정)
  - `src/teleop_dev/sender/hand/config/dg5f_right_retarget.urdf` ← 복사
- **검증 1 (단위)**:
  - dummy "open hand" kp_25 → retarget 결과 q[5,9,13,17] (PIP joint) ≈ 0
  - dummy "fist" kp_25 → 같은 q ≈ joint upper limit
  - 손목 yaw 회전 robust: 같은 손가락 자세 + 회전만 다른 두 입력 → q 의 absolute difference < 0.1 rad
- **spike**: `docs/xr_spike/B2_retarget.md` — 단위 테스트 결과 + DexPilot vs vector 비교

#### B3. XR 손 sender entry (`sender/hand/xr_hand_sender.py`)
- **목적**: BridgePoseStore → XRDexRetargeter → UDP 9872 publish
- **새 파일**: `src/teleop_dev/sender/hand/xr_hand_sender.py`
- **내용**:
  - `BridgePoseStore` 인스턴스 (port 8013)
  - 60Hz 루프: `kp_25 = store.right_hand_positions` → `q_20 = retargeter.retarget(kp_25)` → UDP packet (`type:"xr", joint_angles: q_20, retargeted: true`)
  - 패킷 포맷: `protocol/hand_protocol.py` 의 HandData 와 호환되도록 (joint_angles + wrist_pos + wrist_quat + timestamp + buttons + retargeted: true)
  - Robot 측 `receiver.py` 가 `retargeted: true` 면 passthrough (코드 변경 불요)
- **참조 (변경 없음)**: [sender/hand/manus_sender.py](../sender/hand/manus_sender.py) — UDP 송신 + keyboard state 패턴
- **검증 1 (smoke, 로봇 없음)**:
  - 별도 터미널에서 `python -m sender.arm.monitor --port 9872` (UDP packet 모니터) 실행
  - `python -m sender.hand.xr_hand_sender --target-ip 127.0.0.1` 시작 → 헤드셋 손 들이밀기 → monitor 가 60Hz 근처에서 `joint_angles` 출력
- **검증 2 (실로봇, optional)**: `robot/hand/receiver.py --mode real --hand right` 실행 + XR sender 동작 → 손 동작이 DG-5F 에 전달되는지 시각 확인
- **spike**: `docs/xr_spike/B3_hand_sender.md` — 단위 테스트 + monitor 출력 캡쳐 + (가능 시) 실로봇 영상

### Phase C — XR 팔 sender (UDP 9871)

손 검증 후 팔. 팔은 위험도 ↑ (워크스페이스 충돌 가능) — sim 모드 먼저, 실로봇은 저속.

#### C1. WebXR wrist → 로봇 base_link 좌표 변환 (`sender/arm/xr_frame_align.py`)
- **목적**: WebXR `local-floor` 좌표계의 wrist 4×4 → 로봇 base_link 좌표계 의 wrist 4×4
- **새 파일**: `src/teleop_dev/sender/arm/xr_frame_align.py`
- **내용**:
  - `align_wrist_to_robot(wrist_4x4_webxr) -> wrist_4x4_robot` : WebXR (Y up, +Z toward user) → robot base (Z up, +X forward) basis 변환
  - 캘리브레이션 옵션: 사용자가 손을 로봇 base 원점에 놓고 capture
  - **상대 motion 모드** (기본): xr_teleop 의 [run_teleop_ur10e.py:340-358](../../xr_teleop/scripts/run_teleop_ur10e.py#L340) 패턴 — `r` 키로 origin capture, delta_p + R_delta 만 누적
- **참조 (변경 없음)**:
  - [xr_teleop/scripts/run_teleop_ur10e.py:340-358](../../xr_teleop/scripts/run_teleop_ur10e.py#L340) — relative motion 처방
  - [sender/arm/calibrate.py](../sender/arm/calibrate.py) — teleop_dev 의 Vive 3점 캘리브레이션 (참고용)
- **검증 1 (단위)**:
  - WebXR identity (눈높이 정면) 입력 → robot base 좌표계 의 expected pose (z=+1m, 정면) 매칭
  - 90° yaw 회전 입력 → 결과의 R_delta 가 90° yaw rotation matrix
- **spike**: `docs/xr_spike/C1_frame_align.md` — 좌표계 정의 다이어그램 + 캘리브레이션 절차

#### C2. XR 팔 sender entry (`sender/arm/xr_sender.py`)
- **목적**: BridgePoseStore → xr_frame_align → TeleopSenderBase.run()
- **새 파일**: `src/teleop_dev/sender/arm/xr_sender.py`
- **내용**:
  - `XRArmSender(TeleopSenderBase)` 상속
  - `__init__`: BridgePoseStore 인스턴스 (Phase A2 의 single source of truth — XR 손 sender 와 같은 인스턴스 공유)
  - `_read_input()`: store.right_arm_pose 읽기 → 상대 motion delta 계산 → InputResult 반환
  - 키 매핑:
    - `r` — origin recapture (sync 시작 + 재캘리)
    - `p` — pause/resume (resume 시 자동 recalibrate)
    - `c` — 즉시 recalibrate
    - `q` — quit
    - 상대 motion scale 옵션 `--scale 1.0`
- **참조 (변경 없음)**: [sender/arm/sender_base.py](../sender/arm/sender_base.py) — UDP 송신 + query_pose 핸드셰이크
- **검증 1 (smoke, 로봇 없음, sim 모드)**:
  - `python -m robot.arm.admittance.main --mode sim --input unified` (mock backend) 실행
  - `python -m sender.arm.xr_sender --target-ip 127.0.0.1` 실행 + 헤드셋 손 움직임 → mock robot 의 TCP pose 가 따라가는지 확인 (콘솔 로그)
- **검증 2 (실로봇, 저속)**: `--mode rtde --robot-ip 192.168.0.2 --max-vel 0.1`, scale=0.5 로 시작 → 헤드셋 손 천천히 움직이면 robot 도 천천히 따라감
- **spike**: `docs/xr_spike/C2_arm_sender.md` — 캘리브레이션 절차 + 키 매핑 + 실측 latency

### Phase D — 단일 BridgePoseStore 공유 (process 분리 문제 해결)

Phase B + C 두 sender 가 각각 BridgePoseStore 인스턴스를 만들면 port 충돌 (8013). 통합 launcher 필요.

#### D1. 통합 entry `run_xr_teleop.py`
- **목적**: 한 프로세스에서 BridgePoseStore + arm sender + hand sender 동시 실행
- **새 파일**: `src/teleop_dev/scripts/run_xr_teleop.py`
- **내용**:
  - argparse: `--target-ip`, `--scale`, `--hand|--arm|--both` (default both), `--mode hand|controller`
  - threading: BridgePoseStore (background ws server) + arm sender thread + hand sender thread
  - shutdown: 모든 thread join + UDP socket close
- **참조 (변경 없음)**: [xr_teleop/scripts/run_teleop_ur10e_ws.py](../../xr_teleop/scripts/run_teleop_ur10e_ws.py) — 단일 process 패턴
- **검증 1 (smoke)**:
  - `python -m scripts.run_xr_teleop --target-ip 127.0.0.1 --both` 시작 + 별도 터미널에서 `python -m sender.arm.monitor --port 9871` + `python -m sender.arm.monitor --port 9872` 두 monitor 실행
  - 헤드셋 손 + wrist 움직임 → 두 monitor 가 동시에 60Hz 근처 packet 수신
- **검증 2 (실로봇)**: Phase B3 + C2 두 검증을 동시에 진행
- **spike**: `docs/xr_spike/D1_unified.md` — thread 분리 vs 단일 thread 비교 + race condition 확인

### Phase E — 실로봇 통합 + 안전 (저속 → 정상속)

#### E1. 안전 layer 추가 (`scripts/run_xr_teleop.py` 보강)
- **목적**: 사용자 손 IK 실패 / 시야 밖 / 워크스페이스 초과 시 robot 정지
- **내용**:
  - hand recovery latency 측정 (NaN frame 검출 시 robot 측에 estop 전송하지 않고 sender 측에서 마지막 valid pose 유지)
  - watchdog: BridgePoseStore msg 가 200ms 이상 없으면 모든 sender 가 절대 pose 송신 중단 → robot 측 admittance 가 마지막 명령에서 stationary
  - workspace boundary: arm sender 가 base_link 좌표계 의 z<0.05 / xyz envelope 초과 시 clamp
- **참조 (변경 없음)**: [robot/arm/admittance/safety_monitor.py](../robot/arm/admittance/safety_monitor.py) — 로봇 PC 측 4단계 안전
- **검증**:
  - 단위: 200ms 강제 disconnect → arm sender 가 packet 송신 stop
  - 실로봇: 헤드셋 떼서 (헤드 IMU 만 inactive) → 손 detect 안 됨 → robot 정지
- **spike**: `docs/xr_spike/E1_safety.md`

#### E2. 실로봇 저속 운영 검증
- **목적**: 헤드셋 → 실 UR10e + DG-5F 동작 (저속, 안전 확인)
- **검증 절차**:
  1. 로봇 PC: `python -m robot.arm.admittance.main --mode rtde --input unified --robot-ip <UR_IP>`
  2. 로봇 PC: `python -m robot.hand.receiver --hand right --hand-ip <DG5F_IP>`
  3. 조종 PC: `python -m scripts.run_xr_teleop --target-ip <ROBOT_PC_IP> --scale 0.3 --both`
  4. 헤드셋 → r 키 (sync 시작) → 손 천천히 움직임 → UR10e 따라감
  5. 손가락 굽힘 → DG-5F 따라감
- **gate 조건**:
  - 10분 연속 동작 무사고
  - latency 200ms 이내 (head → DG-5F 시각 확인)
  - E-stop (Ctrl+C 또는 quit 키) 100ms 이내 robot 정지
- **spike**: `docs/xr_spike/E2_realrobot.md` — 측정값 + 영상 (가능하면)

#### E3. 정상속 운영 + recording (선택, Phase 4 사전 작업)
- **목적**: scale 1.0 직관적 매핑 + LeRobot 호환 episode 녹화
- **참조 (변경 없음)**: [xr_teleop/docs/week5_strategy.md](../../xr_teleop/docs/week5_strategy.md) Track B (Recording infra)
- **검증**: 5초 데모 동작 녹화 → HDF5 / npz 로딩 → key replay 도구 동작
- **spike**: `docs/xr_spike/E3_recording.md`

### Phase F — 문서화 + 통합 가이드

#### F1. 통합 setup guide 보강 (`docs/setup_guide.md`)
- **목적**: 조종 PC 에 XR 헤드셋 추가 절차 안내 — adb reverse, cert, Galaxy XR 트러블슈팅
- **참조 (변경 없음)**: [xr_teleop/README.md](../../xr_teleop/README.md) — Step A~H 의 XR 준비 절차
- **검증**: 새 PC 에서 가이드만 보고 1시간 안에 XR sender 동작 재현

#### F2. ARCHITECTURE.md 갱신
- **목적**: XR sender 추가 반영
- **변경**: 시스템 개요 다이어그램 (조종 PC 측에 [Galaxy XR / Quest 3] 추가) + 디렉토리 구조 + 의존성 표

#### F3. XR-specific 가이드 (`docs/xr_input_guide.md`)
- **목적**: arm_input_guide.md / hand_manus_guide.md 와 같은 형태의 XR 사용자 가이드
- **내용**: 헤드셋 설정 → adb reverse → calibration → 사용 절차 + 트러블슈팅 (cert, ws bridge, retargeting)

---

## 3. 영향 받는 파일 목록

### 새로 만드는 파일
- `src/teleop_dev/sender/xr_common/__init__.py` (A2)
- `src/teleop_dev/sender/xr_common/bridge_pose_store.py` (A2)
- `src/teleop_dev/sender/xr_common/webxr_to_pose.html` (A2)
- `src/teleop_dev/sender/xr_common/config.yaml` (A2)
- `src/teleop_dev/scripts/xr_pose_diag.py` (A3)
- `src/teleop_dev/sender/hand/xr_remap.py` (B1)
- `src/teleop_dev/sender/hand/xr_dex_retargeter.py` (B2)
- `src/teleop_dev/sender/hand/xr_hand_sender.py` (B3)
- `src/teleop_dev/sender/hand/config/dg5f_right_xr.yml` (B2)
- `src/teleop_dev/sender/hand/config/dg5f_right_retarget.urdf` (B2)
- `src/teleop_dev/sender/arm/xr_frame_align.py` (C1)
- `src/teleop_dev/sender/arm/xr_sender.py` (C2)
- `src/teleop_dev/scripts/run_xr_teleop.py` (D1)
- `src/teleop_dev/docs/xr_spike/A1_env.md` ~ `E3_recording.md` (각 unit 별)
- `src/teleop_dev/docs/xr_input_guide.md` (F3)

### 수정하는 파일
- `src/teleop_dev/environment.yaml` (A1) — aiohttp 추가
- `src/teleop_dev/docs/setup_guide.md` (F1) — XR 설치 절차 추가
- `src/teleop_dev/docs/ARCHITECTURE.md` (F2) — XR sender 반영

### 수정하지 않는 파일 (중요)
- `src/teleop_dev/protocol/*` — wire format 변경 없음
- `src/teleop_dev/robot/*` — 로봇 PC 측 코드 전체 변경 없음
- `src/teleop_dev/sender/arm/sender_base.py` — TeleopSenderBase 재사용
- `src/teleop_dev/sender/hand/manus_sender.py / realsense_sender.py` — 기존 sender 유지

### 참조 (출처 — 변경 없음)
- [xr_teleop/scripts/bridge_pose_store.py](../../xr_teleop/scripts/bridge_pose_store.py) — ws bridge 패턴 (A2)
- [xr_teleop/assets/webxr_to_pose.html](../../xr_teleop/assets/webxr_to_pose.html) — WebXR client (A2)
- [xr_teleop/scripts/dg5f_controller.py:107](../../xr_teleop/scripts/dg5f_controller.py#L107) — webxr_to_wrist_local_mano (B1)
- [xr_teleop/assets/dg5f_hand/dg5f_right.yml](../../xr_teleop/assets/dg5f_hand/dg5f_right.yml) — DexPilot 설정 (B2)
- [xr_teleop/scripts/run_teleop_ur10e_ws.py:307-358](../../xr_teleop/scripts/run_teleop_ur10e_ws.py#L307) — relative motion (C1)
- [xr_teleop/docs/galaxy_xr_ws_bridge_guide.md](../../xr_teleop/docs/galaxy_xr_ws_bridge_guide.md) — 우회 구조 (A2)
- [xr_teleop/docs/remote_teleop_office_to_field_analysis.md §3.2](../../xr_teleop/docs/remote_teleop_office_to_field_analysis.md) — Thick Relay 권장

---

## 4. 검증 / 통과 조건 (단계별)

| Phase | Gate | 통과 조건 |
|---|---|---|
| A | Gate A | 헤드셋 → 조종 PC 30Hz 이상 안정 pose 수신, lost frame < 5%, recovery < 1s |
| B | Gate B | 단위 테스트 (B1, B2) 전부 통과 + (가능시) 실 DG-5F 시각 동작 확인 |
| C | Gate C | sim 모드에서 헤드셋 → mock UR10e 추종 + 실로봇 저속 추종 |
| D | Gate D | 통합 launcher 에서 두 monitor 동시에 60Hz packet 관찰 |
| E | Gate E | 실 UR10e + DG-5F, 저속 10분 무사고 + E-stop 100ms |
| F | Gate F | 새 사용자가 가이드만 보고 1시간 내 재현 |

---

## 5. 위험 + Fallback

### R1: Galaxy XR Chrome 의 vuer freeze (이미 발견)
- **대응**: BridgePoseStore (ws bridge) 사용. Phase A2 에서 자체 ws server + webxr_to_pose.html 패턴 그대로.
- **fallback**: Quest 3 만 우선 검증 (vuer 정상 동작). Galaxy XR 은 Phase E 이후로 미룸.

### R2: DexPilot convention mismatch (mediapipe vs manus)
- **출처**: [xr_teleop/scripts/dg5f_controller.py:68](../../xr_teleop/scripts/dg5f_controller.py#L68) — WebXR HandLandmarker chirality 미확정
- **대응**: B2 단위 테스트에서 mediapipe 로 시작, 실 헤드셋 visual 검증 (fist↔spread) 후 결정. yml argparse 옵션 노출.
- **fallback**: manus matrix (row 1 sign flip) 로 재시도.

### R3: WebXR wrist → 로봇 base_link 좌표계 정합 어려움
- **대응**: C1 의 relative motion 모드 default. 절대 매핑은 캘리브레이션 후에만.
- **fallback**: 사용자에게 매 sync 시작 시 `r` 키 capture 의존 (xr_teleop 검증된 패턴).

### R4: 실로봇 운영 시 안전 사고
- **대응**: Phase E1 의 watchdog + workspace clamp + scale 0.3 default. E2 의 10분 저속 운영 검증 후 정상속 진입.
- **fallback**: 실로봇은 정지 모드에서 사용자가 손동작만 시각 확인 → 단계적 진입.

### R5: 두 sender 통합 시 shared BridgePoseStore singleton 이슈
- **출처**: [xr_teleop/scripts/bridge_pose_store.py:134-139](../../xr_teleop/scripts/bridge_pose_store.py#L134) — singleton 패턴
- **대응**: D1 단일 process 에서 한 인스턴스만 만들고 arm/hand sender 가 reference 공유.
- **fallback**: 두 process 분리 시에는 BridgePoseStore 만 별도 daemon 으로 운영 + sender 둘은 ws client.

---

## 6. Out of Scope (이 계획)

- **Vive Tracker → XR 교체가 아니라 추가**: 기존 Vive sender 유지. 사용자가 어떤 sender 를 쓸지 선택.
- **WAN 분리 운영**: [remote_teleop_office_to_field_analysis.md](../../xr_teleop/docs/remote_teleop_office_to_field_analysis.md) 의 Tailscale + Thick Relay 는 별도 Phase G+ (이 계획 후속).
- **imitation learning 데이터 수집**: Phase E3 (optional) 만 sketch. 본격 LeRobot 통합은 별도 작업.
- **F/T 시각화 AR overlay**: Phase 5+ (xr_teleop CLAUDE.md §7).
- **eye-tracking / gaze-guided manipulation**: Phase 6+.
- **dual-hand 양손**: DG-5F single right hand 환경. 좌측 hand sender 는 미구현.

---

## 7. 추정 작업 시간

| Phase | 작업 일 (단위 일) | 누적 |
|---|---|---|
| A (환경 + pose 검증) | 1.5 일 | 1.5 일 |
| B (XR 손 sender) | 2 일 | 3.5 일 |
| C (XR 팔 sender) | 2 일 | 5.5 일 |
| D (통합 launcher) | 0.5 일 | 6 일 |
| E (실로봇 + 안전) | 2 일 | 8 일 |
| F (문서화) | 0.5 일 | 8.5 일 |

총 **약 8-9 일** (1.5 ~ 2 주 작업). xr_teleop 의 검증된 코드 재사용 비중 ↑ 라 신규 개발 부담은 낮음.

---

## 8. 다음 단계

본 계획서 사용자 검토 후:
1. **Phase A 부터 plan mode** 로 진입해 unit-by-unit detailed plan 작성
2. 각 unit 마다 spike 작성 + commit
3. Phase B 완료 시 사용자 측 실 DG-5F 시각 확인 1회
4. Phase C 완료 시 사용자 측 실 UR10e sim 측 검증 1회
5. Phase E 진입 전 사용자 측 실로봇 안전 절차 합의

> **선택 사항**: Galaxy XR 우선 vs Quest 3 우선. Quest 3 가 검증 더 진행됨 (Gate 3/4 통과). Galaxy XR 은 ws bridge 로 우회 검증만 됨. **Quest 3 먼저 검증 후 Galaxy XR 적용 권장**.
