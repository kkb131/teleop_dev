# realsense_sender — 사용 가이드

`sender/hand/realsense_sender`로 Intel RealSense D405 카메라를 사용하여
DG5F 핸드를 원격 조종하는 실행 가이드.

알고리즘 / 설치 / 트러블슈팅 상세는 다음 문서 참조:
- 설치 + 하드웨어: [`sender/hand/realsense/docs/realsense_sender_setup.md`](../realsense/docs/realsense_sender_setup.md)
- dex_retargeting 알고리즘: [`sender/hand/gen3a_dex_retarget/docs/dex_retarget_setup.md`](../gen3a_dex_retarget/docs/dex_retarget_setup.md)
- Manus 버전 비교: [`dex_retarget_usage.md`](dex_retarget_usage.md)

---

## 1. 한 줄 요약

D405 (color + depth) → MediaPipe 21 landmarks → real depth deprojection →
MANO 21 keypoints → DexPilot 옵티마이저 → DG5F 20 관절각 → UDP → robot PC.

조종 PC에서 retargeting을 끝내고 robot PC에는 DG5F 각도만 전송 (receiver bypass).

---

## 2. 사전 조건

| 항목 | 위치 | 비고 |
|------|------|------|
| Intel RealSense D405 + USB 3.0 케이블 | 조종 PC | 펌웨어 ≥ 5.13 |
| pyrealsense2 + mediapipe==0.10.21 + opencv-python | 조종 PC | `environment.yaml` 또는 수동 설치 |
| dex_retargeting + numpy<2 | 조종 PC | gen3a_dex_retarget 의존성 |
| Tesollo DG5F + dg5f_driver | 로봇 PC | `pid_all_controller.launch.py` |
| `robot.hand.receiver` 가능 환경 | 로봇 PC | UDP 9872 listening |

설치 검증:
```bash
python3 -c "
import pyrealsense2 as rs, mediapipe as mp, cv2, numpy
print(f'pyrealsense2: {rs.__version__}, mediapipe: {mp.__version__}, opencv: {cv2.__version__}, numpy: {numpy.__version__}')
"
rs-enumerate-devices | grep -i "Intel RealSense D405"
```

---

## 3. 실행 — 4-터미널 시퀀스

### T1 — D405 연결 확인 (조종 PC)
```bash
rs-enumerate-devices | grep -A2 "Intel RealSense D405"
# Serial Number 출력 확인
```

### T2 — DG5F 드라이버 (로봇 PC)
```bash
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py \
  delto_ip:=169.254.186.72
```

`ros2 control list_controllers -c /dg5f_right/controller_manager`로
`rj_dg_pospid` active 상태인지 확인.

### T3 — receiver (로봇 PC)
```bash
python3 -m robot.hand.receiver --hand right
```

retargeted 패킷을 받으면 자동으로 retarget을 스킵하고 DG5F에 그대로 발행.

### T4 — realsense_sender (조종 PC)
```bash
# 기본 (DexPilot 옵티마이저, 30Hz, 자동 D405 검출)
python3 -m sender.hand.realsense_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right

# Vector 옵티마이저 + OpenCV 미리보기
python3 -m sender.hand.realsense_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right \
  --dex-optimizer vector \
  --viz

# 특정 D405 (다중 카메라 환경)
python3 -m sender.hand.realsense_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right \
  --rs-serial 1234ABCDEF
```

성공 시 첫 검출 직후 로그:
```
[RealSenseReader] Started (hand=right)
[Sender] Retarget: 3A-dex-retarget (right, optimizer=dexpilot)
[SEND] #150 mode=DEX retargeted=True angles[0:5]=[+0.05 -0.12 ...]
```

---

## 4. CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--hand` | right | `left` 또는 `right` (single hand only) |
| `--port` | 9872 | UDP 포트 |
| `--hz` | 30 | 송신율. D405는 30Hz native, 더 올려도 효과 없음 |
| `--rs-serial` | (자동) | D405 device serial. 다중 카메라 환경에서 명시 |
| `--resolution` | 640x480 | `WxH` 형식. D405 native 해상도 권장 |
| `--dex-optimizer` | dexpilot | `dexpilot` (thumb-pinch) 또는 `vector` (palm→tip + MCP→DIP) |
| `--viz` | false | OpenCV 미리보기 창 (ESC로 종료) |
| `--config` | default_config.yaml | YAML override |

manus_sender와 달리 `--retarget` 플래그 없음 — RealSense는 항상 [3A] dex 사용
(MediaPipe는 ergonomics를 제공하지 않으므로 [1A] ergo-direct 불가).

---

## 5. config 튜닝

### 5.1 카메라 / 송신율
`sender/hand/realsense/default_config.yaml`:
```yaml
camera:
  width: 640
  height: 480
  fps: 30          # D405 native; 더 올리면 detection 못 따라옴
network:
  hz: 30           # camera fps와 동일하게
```

### 5.2 dex_retargeting 옵티마이저 튜닝
config 파일은 `sender/hand/gen3a_dex_retarget/config/dg5f_right_{dexpilot,vector}.yml`
(manus와 동일한 config 사용).

| 파라미터 | 효과 | 권장 |
|---|---|---|
| `scaling_factor` | 인간→로봇 fingertip 스케일 | 1.2 (기본). 손가락 안 굽힘 → 1.4 |
| `low_pass_alpha` | 출력 smoothing | 0.2 (기본). 진동 → 0.1, 반응 느림 → 0.4 |

YAML 수정 후 sender 재시작 필요.

### 5.3 D405 카메라 위치
- 손이 카메라에서 **20cm ~ 80cm** 범위에 있어야 안정 (D405 stereo baseline)
- 손바닥이 카메라를 정면으로 향해야 MediaPipe 검출률 높음
- 손등 / 측면 view는 검출률 떨어짐 — 카메라 위치 재조정 권장

---

## 6. 데이터 흐름 (코드 레벨)

```
RealSense D405 (color + depth, 30Hz)
  └─ pyrealsense2 pipeline + rs.align(rs.stream.color)
      └─ aligned color (BGR) + depth (meters)
            ↓
RSCamera.read() → (ok, color, depth_m)
            ↓
HandDetector.detect(color, ts_ms)  [MediaPipe HandLandmarker VIDEO mode]
  └─ HandDetection
      ├─ landmarks_2d (21, 3) normalized image coords
      └─ world_landmarks (21, 3) MediaPipe estimated meters (fallback)
            ↓
DepthKeypointConverter.convert(detection, depth_m, w, h)
  ├─ For each of 21 keypoints:
  │   ├─ pixel coords from landmarks_2d
  │   ├─ neighborhood depth median (radius=4, valid range 0.02m~1.0m)
  │   └─ rs2_deproject_pixel_to_point(intrinsics, [px, py], d) → 3D xyz
  ├─ Shift to wrist origin: pts_3d -= pts_3d[0]
  └─ apply_mano_transform(pts_3d, hand_type)
            ↓
HandKeypoints (21, 3 wrist-relative MANO frame)
            ↓
RealSenseReader._latest_keypoints  ← background thread
            ↓
realsense_sender main loop
  └─ data = reader.get_data()  # non-blocking
      └─ if data is not None:
             dg5f_q = retarget.retarget(keypoints=data.keypoints_3d)
            ↓
DexRetargetWrapper.retarget(keypoints=...)
  ├─ kp = keypoints[:21, :3]
  ├─ kp = kp - kp[wrist_idx]   # idempotent (already wrist-centered)
  ├─ ref_value = _build_ref_value(kp)  # vector / dexpilot 분기
  └─ self._retargeting.retarget(ref_value) → q (20,)
            ↓
_build_packet(dg5f_q, hand_side, buttons) → "type":"realsense","retargeted":True
            ↓
UDP socket.sendto(<ROBOT_IP>, 9872)
            ↓
robot/hand/receiver.py
  └─ pkt["retargeted"] == True → retarget 스킵 → DG5F publish
```

핵심 불변식:
- `data.keypoints_3d` 는 `(21, 3)`, **wrist 원점, MANO 좌표계** (MANO transform 적용 후)
- 손이 검출 안 되면 `reader.get_data()` 가 `None` → `_build_null_packet()` 으로 전송
- `retargeted: True` 항상 — sender가 매 프레임 retarget

---

## 7. 동작 검증 체크리스트

순서대로 확인:

1. **D405 검출**
   ```bash
   rs-enumerate-devices | grep -i "Intel RealSense D405"
   # Serial Number 출력 확인
   ```

2. **카메라 단독 테스트**
   ```bash
   realsense-viewer  # GUI에 color + depth 스트림 보임
   ```

3. **`--viz` 모드로 검출 확인**
   ```bash
   python3 -m sender.hand.realsense_sender \
     --target-ip 127.0.0.1 --viz
   # OpenCV 창에 손 + 21 keypoints + bone connections 표시
   # ESC로 종료
   ```

4. **receiver가 retargeted 플래그 인식**
   receiver 터미널에서 `retargeted=True` 로 표시되면 retarget 스킵.

5. **DG5F 즉시 반응**
   조종 PC에서 손가락을 움직이면 ~30~50ms 지연으로 DG5F가 따라옴.

6. **자세 정확도**
   - 손을 쫙 폈을 때 DG5F도 펴짐 → OK
   - 주먹 쥘 때 손가락이 안 굽혀짐 → `scaling_factor` 1.4까지 증가
   - 검지/중지가 손등 방향으로 뒤집힘 → URDF가 PIP/DIP 음수 제한 버전인지 확인

---

## 8. 트러블슈팅 (사용 시)

### `RuntimeError: No device connected`
- `rs-enumerate-devices` 로 D405 검출 확인
- 다른 RealSense 앱 (`realsense-viewer`)이 D405 점유 중이면 종료
- USB 케이블 재연결

### MediaPipe 검출 안 됨
- 손이 카메라 view에 들어와 있는지 (`--viz` 로 확인)
- 조명 부족 (어두운 환경) → 조명 추가
- 손바닥이 카메라를 정면으로 향하는지

### `[Sender] Hand LOST` 가 계속 뜸
- 손이 카메라 view 밖
- depth 노이즈 (반사 표면, 너무 가까움)
- MediaPipe 검출 신뢰도 낮음 → 손 자세 / 거리 조정

### DG5F가 손가락을 충분히 안 굽힘
- `gen3a_dex_retarget/config/dg5f_right_dexpilot.yml`의 `scaling_factor` 1.2 → 1.4

### 검지/중지가 손등 방향으로 뒤집힘
- `dg5f_right_retarget.urdf` 가 PIP/DIP 음수 제한 버전인지 확인
- `gen3a_dex_retarget/config/` 의 사본은 이미 수정됨

### MediaPipe import 시 오류
```bash
pip install "mediapipe==0.10.21"  # 0.10.22+ 는 framework 모듈 제거
```

### dex_retargeting / pinocchio 관련 segfault
```bash
pip install "numpy<2"  # eigenpy ABI 호환
```

### `[Sender] Hand LOST` 가 너무 자주 (~매 5초)
detection rate가 카메라 fps보다 느려서 발생. detection ms가 30ms 이상이면
GPU 가속 없는 환경. 해결책:
- `--hz 15` 로 송신율 낮춤
- 더 빠른 CPU/GPU 환경으로 이전

---

## 9. manus_sender vs realsense_sender 비교

| | manus_sender | realsense_sender |
|---|---|---|
| 입력 장치 | Manus Quantum 글러브 + 동글 | Intel RealSense D405 |
| 도입 비용 | $$$$ (글러브 + SDK) | $300 (D405) |
| 갱신율 | 120 Hz native | 30 Hz native (depth bottleneck) |
| 정밀도 | 직접 측정 (sub-mm) | depth 추정 (~1-5mm) |
| Occlusion | 무관 (착용형) | 가려지면 검출 실패 |
| 카메라 시점 의존성 | 없음 | 있음 |
| 캘리브레이션 | 글러브 fitting | 카메라 위치 |
| retarget 모드 | none / ergo-direct / dex | dex 전용 |
| dependency | manus_ros2 + dex_retargeting | pyrealsense2 + mediapipe + dex_retargeting |
| 권장 용도 | 정밀 작업, 장시간 데이터 수집 | 데모, 빠른 프로토타입, 글러브 미보유 환경 |

선택 가이드:
- **글러브 보유 + 정밀 작업**: `manus_sender --retarget dex --sdk-mode ros2`
- **글러브 미보유**: `realsense_sender`
- **저사양 데모**: `realsense_sender` (D405는 USB 3 카메라만 있으면 됨)

---

## 10. 참고 파일

- 코드:
  - `sender/hand/realsense_sender.py` — CLI entry
  - `sender/hand/realsense/reader.py` — RealSenseReader (camera + detector + converter)
  - `sender/hand/realsense/depth_keypoint_converter.py` — depth deprojection
  - `sender/hand/realsense/mano_transform.py` — MANO 좌표계 변환
  - `sender/hand/realsense/hand_keypoints.py` — HandKeypoints dataclass
  - `sender/hand/gen3a_dex_retarget/dex_retarget.py` — DexRetargetWrapper (재사용)
- Config:
  - `sender/hand/realsense/default_config.yaml` — 기본 송신/카메라 설정
  - `sender/hand/gen3a_dex_retarget/config/dg5f_right_dexpilot.yml` — DexPilot YAML
  - `sender/hand/gen3a_dex_retarget/config/dg5f_right_vector.yml` — Vector YAML
- 외부:
  - dex_retargeting 라이브러리: <https://github.com/dexsuite/dex-retargeting>
  - AnyTeleop (RSS 2023): <https://yzqin.github.io/anyteleop/>
  - MediaPipe HandLandmarker: <https://developers.google.com/mediapipe/solutions/vision/hand_landmarker>
  - Intel RealSense D405: <https://www.intelrealsense.com/depth-camera-d405/>
