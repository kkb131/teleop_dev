# [3A] dex_retarget — 설치 / 실행 / 트러블슈팅

Manus 글러브의 raw skeleton (25 노드) 을 DexPilot/Vector 옵티마이저로
DG5F 20 관절각으로 retarget. AnyTeleop (RSS 2023) 의 dex_retargeting 라이브러리
기반 fingertip 위치 매칭 + 다중 cost 최적화.

```
manus_data_publisher (ROS2 120Hz)
  → /manus_glove_*  ManusGlove msg
    → ManusReaderROS2 (callback)
      → 25 → 21 MANO 리매핑 (chain_type, joint_type 메타데이터)
        → HandData.skeleton (21, 7)
          → DexRetargetWrapper.retarget()
            → DG5F angles (20,)
              → manus_sender → UDP → robot/hand/receiver → DG5F
```

## 1. 사전 설치 (조종 PC)

`environment.yaml`을 사용하면 자동 설치됨. 수동 설치 시:

```bash
pip install dex_retargeting
pip install "numpy<2"             # 필수: pinocchio / eigenpy ABI 호환
pip install mediapipe==0.10.21    # 필수: dex_retargeting examples 의존
```

> **참고: `git clone` 불필요.** retarget_dev의 setup.md는 dex-retargeting
> GitHub 레포 + git submodule을 요구하지만, 그건 라이브러리 내장 URDF
> (Allegro/Shadow/LEAP)를 쓰는 경우입니다. **이 모듈은 DG5F URDF
> (`gen3a_dex_retarget/config/dg5f_right_retarget.urdf`)를 자체 보관**하고
> `DexRetargetWrapper.__init__`이 `RetargetingConfig.set_default_urdf_dir()`로
> 그 디렉터리를 지정하므로, dex_retargeting 패키지의 `assets/robots/hands/`는
> 사용하지 않습니다. `pip install dex_retargeting` 한 줄이면 충분.

### 버전 제약 (중요)

| 패키지 | 제약 | 이유 |
|---|---|---|
| `numpy` | < 2.0 | dex_retargeting / pinocchio / eigenpy 모두 numpy 1.x ABI 바이너리 |
| `mediapipe` | == 0.10.21 | 0.10.22+ 에서 `mediapipe.framework` 모듈 제거 |
| `pinocchio` | ≥ 3.9.0 | dex_retargeting 라이브러리 의존 |

dex_retargeting 자체가 `numpy>=2`를 요구해도 무시. 실제 동작에 문제 없음.

### 검증

```bash
python3 -c "
from dex_retargeting.retargeting_config import RetargetingConfig
import pinocchio, numpy
print(f'numpy: {numpy.__version__}')
print(f'pinocchio: {pinocchio.__version__}')
print('dex_retargeting: OK')
"
```

## 2. ROS2 / manus_data_publisher

`--retarget dex` 모드는 **ROS2 모드 전용**이다 (subprocess 모드는 skeleton의
`chain_type`/`joint_type` 메타데이터를 출력하지 않음).

```bash
# T1 (조종 PC): manus_data_publisher
source /opt/ros/humble/setup.bash
source ~/manus_ws/install/setup.bash
ros2 run manus_ros2 manus_data_publisher

# T2 (조종 PC): 토픽 확인
ros2 topic echo /manus_glove_0 --once
# raw_node_count: 25 가 보여야 정상
```

## 3. 실행

```bash
# T1 (로봇 PC): DG5F 드라이버
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py delto_ip:=169.254.186.72

# T2 (로봇 PC): receiver
python3 -m robot.hand.receiver --hand right

# T3 (조종 PC): manus_data_publisher (위 §2)

# T4 (조종 PC): sender [3A] 모드
python3 -m sender.hand.manus_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right \
  --retarget dex \
  --sdk-mode ros2
```

성공 시 첫 메시지 수신 직후 로그:
```
[Sender] Retarget: 3A-dex-retarget (right)
[ManusROS2] Manus skeleton: 25 raw nodes → MANO 21 (remap OK)
```

## 4. 25 → 21 MANO 리매핑 (배경)

Manus SDK는 손당 **25 노드** publish (1 wrist + thumb 4 + 4 finger × 5),
dex_retargeting은 **21 MANO keypoint** 가정. 비-thumb 4 개의 Metacarpal 노드를
드롭해야 함.

리매핑 표는 `gen3a_dex_retarget/manus_remap.py` 의 `_MANO_REMAP` 참조.
함정: publisher의 string label은 해부학적 의미와 한 단계 어긋남:

| SDK enum | publisher string | 해부학적 의미 |
|---|---|---|
| Metacarpal | `"MCP"` | CMC / metacarpal-base |
| Proximal | `"PIP"` | MCP (knuckle) |
| Intermediate | `"IP"` | PIP |
| Distal | `"DIP"` | DIP |
| Tip | `"TIP"` | fingertip |

→ `("Index", "PIP"): 5` 는 "MANO Index PIP (idx 6)" 가 아니라 **MANO Index MCP
(idx 5)** 에 매핑됨. publisher 문자열 기준으로 매칭하기 때문.

## 5. config 파일

`gen3a_dex_retarget/config/` 에 로컬 복사본:
- `dg5f_right_retarget.urdf` — DG5F 우측 손 URDF (PIP/DIP 음수 제한 추가)
- `dg5f_right_dexpilot.yml` — **권장** DexPilot 옵티마이저 config
- `dg5f_right_vector.yml` — Vector 옵티마이저 config (다중 task)

YAML의 `urdf_path`는 파일명만 (`dg5f_right_retarget.urdf`) 적혀 있고,
`DexRetargetWrapper.__init__` 가 YAML 디렉터리 기준으로 절대화함
(`RetargetingConfig.set_default_urdf_dir`).

### 튜닝

| 파라미터 | 효과 |
|---|---|
| `scaling_factor` (default 1.2) | 인간 → 로봇 손 크기 비율. 손가락이 충분히 안 굽혀지면 1.4 까지 증가 |
| `low_pass_alpha` (default 0.2) | 0=완전 smoothing, 1=raw. 진동 시 낮춤 |

YAML 수정 후 sender 재시작 필요.

## 6. 트러블슈팅

### `ModuleNotFoundError: dex_retargeting`
→ `pip install dex_retargeting`

### `ModuleNotFoundError: mediapipe.framework`
→ `pip install mediapipe==0.10.21`

### `_ARRAY_API not found` 또는 import 시 segfault
→ `pip install "numpy<2"`

### `[ERROR] --retarget dex requires --sdk-mode ros2`
→ subprocess 모드는 skeleton 메타데이터 부재. `--sdk-mode ros2` 사용 + ROS2
   manus_data_publisher 실행.

### `[ManusROS2] WARNING: Manus skeleton remap incomplete`
출력된 `(chain, joint)` 키 set 을 확인하여:
- 모든 손가락 5(또는 thumb는 4)개 들어와야 함
- `("Pinky", "TIP")` 같은 키가 빠지면 글러브 센서 dropout
- 처음 보는 string이 있으면 `manus_remap.py` 의 `_MANO_REMAP` 에 추가

일시적 dropout은 자동 skip + warm start. 폭주 없음.

### DG5F가 손가락을 충분히 안 굽힘
→ `dg5f_right_dexpilot.yml` 의 `scaling_factor` 를 1.2 → 1.4 로 증가

### 검지/중지가 손등 방향으로 뒤집힘
→ `dg5f_right_retarget.urdf` 가 PIP/DIP 음수 제한 버전인지 확인 (이 폴더의
   사본은 이미 수정된 버전)

### `No data received after 10s. Is manus_data_publisher running?`
- T1에서 `ros2 run manus_ros2 manus_data_publisher` 가 실행 중인지
- `ros2 topic list | grep manus_glove`
- 같은 ROS_DOMAIN_ID 인지 확인

## 7. 성능 참고치

- Manus → ROS2 publisher: **120 Hz** native
- ROS2 callback + 25→21 remap: ~0.5ms / frame
- DexRetargetWrapper.retarget (DexPilot): ~5–10ms
- UDP 송신 + receiver → DG5F PID: ~5–10ms
- **Total round-trip**: ~30–40ms → 안정 30Hz

dex_retargeting 옵티마이저 warm-start가 안정화되는 데 ~5 frames 필요.
sender `--hz 30` 권장.

## 8. 참고

- 원본 구현: `retarget_dev/models/dex_retarget/`
- 알고리즘 개요: `retarget_dev/docs/retargeting_research.md`
- AnyTeleop 논문: <https://yzqin.github.io/anyteleop/>
- dex_retargeting 라이브러리: <https://github.com/dexsuite/dex-retargeting>
