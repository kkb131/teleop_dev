# [3A] dex_retarget — 사용 가이드

`sender/hand/gen3a_dex_retarget` 모듈을 이용해 Manus 글러브 raw skeleton을
DG5F 핸드로 retargeting하는 실행 가이드.

알고리즘 / 설치 / 트러블슈팅 상세는 다음 문서 참조:
- 설치 + 의존성: [`gen3a_dex_retarget/docs/dex_retarget_setup.md`](../gen3a_dex_retarget/docs/dex_retarget_setup.md)
- 전체 핸드 시스템 개요: [`docs/hand_manus_guide.md`](../../../docs/hand_manus_guide.md)
- 1A vs 3A 비교 / 아키텍처: [`hand_retarget_1a_impl.md`](hand_retarget_1a_impl.md)

---

## 1. 한 줄 요약

Manus skeleton (25 raw nodes) → 21 MANO keypoint → DexPilot 옵티마이저 → DG5F 20관절각.
조종 PC에서 retargeting을 끝내고 robot PC에는 DG5F 각도만 UDP로 전송 (receiver는 bypass).

---

## 2. 사전 조건

| 항목 | 위치 | 비고 |
|------|------|------|
| Manus 글러브 + 동글 | 조종 PC | Manus Core 라이선스 필요 |
| `manus_ros2` + `manus_ros2_msgs` | 조종 PC | colcon build → source |
| dex_retargeting + numpy<2 + mediapipe==0.10.21 | 조종 PC | `environment.yaml` 또는 수동 설치 |
| Tesollo DG5F + dg5f_driver | 로봇 PC | `pid_all_controller.launch.py` |
| `robot.hand.receiver` 가능 환경 | 로봇 PC | UDP 9872 listening |

설치 검증:
```bash
python3 -c "
from sender.hand.gen3a_dex_retarget import DexRetargetWrapper
import numpy as np
rt = DexRetargetWrapper(hand_side='right')
fake = np.zeros((21, 7), dtype=np.float32); fake[:, 3] = 1.0  # qw=1
q = rt.retarget(skeleton=fake)
assert q.shape == (20,), q.shape
print('[OK] DexRetargetWrapper basic')
"
```

---

## 3. 실행 — 4-터미널 시퀀스

### T1 — Manus ROS2 publisher (조종 PC)
```bash
source /opt/ros/humble/setup.bash
source ~/manus_ws/install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

확인:
```bash
ros2 topic list | grep manus_glove   # /manus_glove_0 등 보여야 함
ros2 topic echo /manus_glove_0 --once  # raw_node_count: 25
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

### T4 — sender [3A] 모드 (조종 PC)
```bash
# 기본 (DexPilot 옵티마이저)
python3 -m sender.hand.manus_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right \
  --retarget dex \
  --sdk-mode ros2

# Vector 옵티마이저 (다중 task: palm→tip + MCP→DIP)
python3 -m sender.hand.manus_sender \
  --target-ip <ROBOT_PC_IP> \
  --hand right \
  --retarget dex \
  --dex-optimizer vector \
  --sdk-mode ros2
```

`--dex-optimizer` 기본값은 `dexpilot`. 둘 다 동일 URDF를 쓰며 코드 변경 없이
런타임에 교체 가능 (config 파일은 `gen3a_dex_retarget/config/dg5f_right_{optimizer}.yml`).

성공 시 첫 메시지 수신 직후 로그:
```
[Sender] Retarget: 3A-dex-retarget (right, optimizer=dexpilot)
[ManusROS2] Manus skeleton: 25 raw nodes → MANO 21 (remap OK)
```

---

## 4. CLI 옵션 (3A 모드 관련)

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--target-ip` | (필수) | 로봇 PC IP |
| `--hand` | right | `left`, `right`, `both` |
| `--retarget` | none | **`dex` 로 설정해야 [3A] 활성화** |
| `--dex-optimizer` | dexpilot | `dexpilot` (thumb-pinch 특화) 또는 `vector` (palm→tip + MCP→DIP 다중 task) |
| `--sdk-mode` | (config에서) | **`ros2` 강제** — subprocess는 skeleton 메타데이터 부재로 거부됨 |
| `--hz` | 60 | 송신율 (Hz). dex_retargeting warm-start 안정화를 위해 30~60 권장 |
| `--port` | 9872 | UDP 포트 |
| `--config` | default.yaml | yaml에서 일부 옵션 override 가능 |

`--calibrate`는 [1A] 전용 (2포즈 ergonomics 캘리브레이션). dex_retarget는
URDF에 인코딩된 DG5F 한계와 `scaling_factor`만으로 동작하므로 별도 캘리 단계 없음.

### 잘못된 조합
```bash
python3 -m sender.hand.manus_sender --retarget dex --sdk-mode subprocess
# → [ERROR] --retarget dex requires --sdk-mode ros2 ...
```

---

## 5. config 튜닝

`gen3a_dex_retarget/config/dg5f_right_dexpilot.yml` (기본):
```yaml
retargeting:
  type: DexPilot
  urdf_path: dg5f_right_retarget.urdf
  wrist_link_name: "rl_dg_palm"
  finger_tip_link_names: [rl_dg_1_tip, rl_dg_2_tip, ..., rl_dg_5_tip]
  scaling_factor: 1.2
  low_pass_alpha: 0.2
```

| 파라미터 | 효과 | 조정 가이드 |
|---|---|---|
| `scaling_factor` | 인간→로봇 fingertip 거리 비율 | 손가락이 충분히 안 굽힘 → 1.4까지 증가. 너무 굽힘 → 1.0으로 감소 |
| `low_pass_alpha` | 출력 smoothing (0=완전, 1=raw) | 진동 발생 → 낮춤 (0.1). 반응 느림 → 높임 (0.4) |
| `urdf_path` | DG5F 모델 경로 | 상대경로(파일명만) — `DexRetargetWrapper`가 yaml 디렉터리 기준으로 절대화 |

YAML 수정 후 sender 재시작 필요. 두 옵티마이저는 sender CLI로 즉시 교체:
```bash
# DexPilot (기본)
python3 -m sender.hand.manus_sender --retarget dex --dex-optimizer dexpilot ...

# Vector (다중 task)
python3 -m sender.hand.manus_sender --retarget dex --dex-optimizer vector ...
```

또는 직접 임의 YAML 경로 지정 (라이브러리 API):
```python
from sender.hand.gen3a_dex_retarget import DexRetargetWrapper
rt = DexRetargetWrapper(
    hand_side='right',
    config_path='my/custom/dg5f.yml',  # optimizer 인자 무시됨
)
```

---

## 6. 데이터 흐름 (코드 레벨)

```
manus_data_publisher (C++, ROS2 120Hz)
  └─ ManusGlove msg
     ├─ ergonomics[20]   ← _ERGO_TYPE_MAP (1A 모드용, 3A는 무시)
     └─ raw_nodes[25]    ← _MANO_REMAP (3A 모드용)
            ↓
ManusReaderROS2._glove_callback
  └─ manus_remap.remap_to_mano_21(raw_nodes) → (21, 7) 또는 None
      └─ HandData.skeleton, HandData.has_skeleton
            ↓
manus_sender main loop
  └─ _apply_retarget(data)
      ├─ method = retarget.get_method_name()  # "3A-dex-retarget"
      └─ if data.has_skeleton:
             dg5f_q = retarget.retarget(skeleton=data.skeleton)
            ↓
DexRetargetWrapper.retarget(skeleton)
  ├─ kp = skeleton[:21, :3] - skeleton[0, :3]   # wrist 원점화
  ├─ ref_value = _build_ref_value(kp)            # (vector / dexpilot 분기)
  └─ self._retargeting.retarget(ref_value)       # SeqRetargeting → 20D
            ↓
data.joint_angles ← dg5f_q  (HandData에 덮어쓰기)
            ↓
_build_packet → "retargeted": True
            ↓
UDP socket.sendto(<ROBOT_IP>, 9872)
            ↓
robot/hand/receiver.py
  └─ pkt["retargeted"] == True → retarget 스킵 → 그대로 DG5F publish
```

핵심 불변식:
- `data.has_skeleton == True` 일 때만 [3A] 가 retarget 수행
- 부분 dropout (일부 노드 누락) → `remap_to_mano_21()` None 반환 → frame skip,
  마지막 자세 유지 (warm start)

---

## 7. 동작 검증 체크리스트

순서대로 확인:

1. **manus_data_publisher 정상**
   ```bash
   ros2 topic hz /manus_glove_0
   # ~120 Hz 가 나와야 함
   ```

2. **sender가 skeleton 파싱 성공**
   sender 실행 후 첫 1초 이내:
   ```
   [ManusROS2] Manus skeleton: 25 raw nodes → MANO 21 (remap OK)
   ```
   `WARNING ... remap incomplete` 메시지가 뜨면 §8 트러블슈팅 참조.

3. **receiver가 retargeted 플래그 인식**
   receiver 터미널에 `retargeted=True` 로 표시되면 receiver는 retarget 스킵.

4. **DG5F 즉시 반응**
   조종 PC에서 손가락을 움직이면 ~30~50ms 지연으로 DG5F가 따라옴.
   너무 느리면 sender `--hz`를 60으로 올림. 진동하면 yaml `low_pass_alpha`
   감소 또는 `scaling_factor` 감소.

5. **자세 정확도**
   - 손을 쫙 폈을 때 DG5F도 펴짐 → OK
   - 주먹 쥘 때 손가락이 안 굽혀짐 → `scaling_factor` 1.4까지 증가
   - 손가락이 손등 방향으로 뒤집힘 → URDF가 PIP/DIP 음수 제한 버전인지 확인
     (`gen3a_dex_retarget/config/dg5f_right_retarget.urdf`의 사본은 이미 수정됨)

---

## 8. 트러블슈팅 (사용 시)

### `[ERROR] --retarget dex requires --sdk-mode ros2`
SDK subprocess 모드는 skeleton 메타데이터를 출력하지 않음.
`--sdk-mode ros2` + manus_data_publisher 실행 필수.

### `[ManusROS2] WARNING: Manus skeleton remap incomplete`
출력된 `(chain, joint)` 키 set 을 확인:
- 모든 손가락 5개씩 (thumb 4개) 들어와야 함
- `("Pinky", "TIP")` 같은 키가 빠지면 글러브 센서 dropout
- 처음 보는 string이 있으면 `gen3a_dex_retarget/manus_remap.py`의
  `_MANO_REMAP` 표 업데이트 필요

일시적 dropout이면 frame skip + warm start로 복구되므로 계속 진행. 지속되면
글러브 펌웨어 / 동글 연결 / 캘리브레이션 점검.

### `ModuleNotFoundError: dex_retargeting`
```bash
pip install dex_retargeting "numpy<2" mediapipe==0.10.21
```

### `_ARRAY_API not found` 또는 import 시 segfault
numpy 2.x 설치되어 있음. `pip install "numpy<2"` 강제.

### DG5F가 조용함
- T1~T4 모두 살아있는지 확인
- sender 출력에 `[SEND] ... retargeted=True` 표시되는지
- receiver 출력에 패킷 카운터 증가하는지
- robot PC `ros2 topic echo /dg5f_right/rj_dg_pospid/reference` 로 명령 발행 확인

### dex_retargeting 옵티마이저가 매우 느림
`--hz`를 30으로 낮춤. SLSQP 솔버 워밍업에 ~5 frames 필요하므로 첫 100ms는
정상보다 느릴 수 있음.

---

## 9. 다른 모드와의 비교

| | [none] raw | [1A] ergo-direct | [3A] dex_retarget |
|---|---|---|---|
| 입력 | Manus ergonomics | Manus ergonomics | Manus skeleton (21 MANO) |
| retarget 위치 | 로봇 PC | 조종 PC | 조종 PC |
| 캘리브레이션 | 없음 | 2-pose (open + fist) | 없음 (URDF 한계 + scaling_factor) |
| 손가락 형태 보정 | 없음 | direction × scale | DexPilot fingertip optimization |
| 의존성 | 없음 | numpy | dex_retargeting + pinocchio + numpy<2 |
| 응답성 | ★★★ | ★★★ | ★★ (옵티마이저 ~5-10ms) |
| 정확도 (파지) | ★ | ★★★ | ★★★★ |
| ROS2 필요 | 선택 | 선택 | **필수** (manus_data_publisher) |

선택 가이드:
- 데모 / 빠른 검증: `[1A] ergo-direct --calibrate` (캘리만 하면 즉시 동작)
- 정밀한 파지 / 객체 조작: `[3A] dex` (추가 셋업 비용 있지만 fingertip 정확)
- 데이터 수집 (raw 보존): `[none]` (로봇 PC에서 후처리)

---

## 10. 참고 파일

- 코드:
  - `sender/hand/gen3a_dex_retarget/dex_retarget.py` — `DexRetargetWrapper`
  - `sender/hand/gen3a_dex_retarget/manus_remap.py` — `_MANO_REMAP`, `remap_to_mano_21`
  - `sender/hand/manus_reader_ros2.py` — skeleton 파싱 + `_diagnose_remap`
  - `sender/hand/manus_sender.py` — `--retarget dex` 분기 + `_apply_retarget`
- Config:
  - `sender/hand/gen3a_dex_retarget/config/dg5f_right_dexpilot.yml` (기본)
  - `sender/hand/gen3a_dex_retarget/config/dg5f_right_vector.yml`
  - `sender/hand/gen3a_dex_retarget/config/dg5f_right_retarget.urdf`
- 외부:
  - dex_retargeting 라이브러리: <https://github.com/dexsuite/dex-retargeting>
  - AnyTeleop (RSS 2023): <https://yzqin.github.io/anyteleop/>
