# [3A] dex_retarget 세대 추가 — gen3a_dex_retarget

## Context
[1A] Ergo-Direct만으로는 손가락 형태 차이로 인한 fingertip 위치 부정확 문제
해결 어려움. dex-retargeting 라이브러리(DexPilot/Vector 옵티마이저, AnyTeleop
RSS 2023)를 도입해 fingertip 위치 매칭 + 다중 cost 최적화로 정확도를 올린다.

retarget_dev에서 검증된 코드 + DG5F URDF/config + Manus 25→21 MANO 리매핑
로직을 teleop_dev/sender/hand에 이식한다. 이 모듈이 동작하면 [1A]와 함께
런타임에 `--retarget` 인자로 선택 가능.

핵심 파이프라인:
```
manus_data_publisher (ROS2) → /manus_glove_*
  → ManusReaderROS2 (skeleton 25 nodes 파싱 복원)
  → 25→21 MANO 리매핑 (chain_type, joint_type 메타데이터 사용)
  → wrist 원점 이동 → HandKeypoints (21, 3)
  → DexPilotOptimizer (pinocchio FK + 다중 cost 최적화)
  → DG5F joint angles (20,) → UDP → robot/hand/receiver → DG5F
```

## 1. 새 폴더: sender/hand/gen3a_dex_retarget/

### 1.1 dex_retarget.py — `DexRetargetWrapper` 클래스 (신규)
`HandRetargetBase` 상속. retarget_dev의 `DexRetargetModel`을 teleop_dev
인터페이스에 맞게 래핑.

```python
class DexRetargetWrapper(HandRetargetBase):
    def __init__(self, hand_side="right", config_path=None):
        super().__init__(hand_side)
        from dex_retargeting.retargeting_config import RetargetingConfig
        # 기본 config: gen3a_dex_retarget/config/dg5f_{side}_dexpilot.yml
        if config_path is None:
            here = Path(__file__).parent
            config_path = here / "config" / f"dg5f_{hand_side}_dexpilot.yml"
        cfg = RetargetingConfig.load_from_file(str(config_path))
        self._retargeting = cfg.build()
        opt = self._retargeting.optimizer
        self._retargeting_type = opt.retargeting_type
        self._human_indices = opt.target_link_human_indices
        self._wrist_idx = 0  # MANO 21 wrist 인덱스

    def retarget(self, skeleton: np.ndarray = None, **kwargs) -> np.ndarray:
        """skeleton: (21, 7) MANO 21-node array.
        Returns: (20,) DG5F joint angles in radians."""
        if skeleton is None or skeleton.shape[0] < 21:
            return np.zeros(20)
        # Extract xyz, shift to wrist origin
        kp = skeleton[:21, :3].astype(np.float32)
        kp = kp - kp[self._wrist_idx]
        ref_value = self._build_ref_value(kp)
        return self._retargeting.retarget(ref_value)

    def _build_ref_value(self, kp):
        idx = self._human_indices
        if self._retargeting_type == "POSITION":
            return kp[idx, :]
        return kp[idx[1, :], :] - kp[idx[0, :], :]

    def get_method_name(self) -> str:
        return "3A-dex-retarget"
```

`retarget_dev/models/dex_retarget/dex_retarget_model.py`의 `_build_ref_value`
와 동일한 패턴 — vector / dexpilot / position 모두 동일 코드 경로 사용.

### 1.2 manus_remap.py — `_remap_to_mano_21()` 함수 (신규)
`retarget_dev/sensing/manus/ros2_provider.py:51-123`에서 발췌.
- `_MANO_REMAP` dict (Manus chain/joint string → MANO 21 인덱스)
- `_remap_to_mano_21(raw_nodes) -> Optional[np.ndarray]` 함수 (21, 7) 반환
- 25개 raw 노드를 chain_type/joint_type 메타데이터 기반으로 21 슬롯에 배치
- 일부 누락 시 None 반환 → 호출자가 retarget skip

### 1.3 __init__.py
```python
from sender.hand.gen3a_dex_retarget.dex_retarget import DexRetargetWrapper
__all__ = ["DexRetargetWrapper"]
```

### 1.4 config/ — 로컬 복사
- `dg5f_right_retarget.urdf` (745 라인) ← retarget_dev/models/dex_retarget/config/
- `dg5f_right_dexpilot.yml` (14 라인)
- `dg5f_right_vector.yml` (66 라인)

YAML 파일의 `urdf_path`는 동적으로 절대 경로를 채워야 함. 옵션:
- A) YAML에 절대 경로 하드코딩 (배포 시 환경 의존)
- B) **택일** — `DexRetargetWrapper.__init__`에서 YAML 로드 후 urdf_path를
  현재 파일 기준 상대 경로로 패치 (코드에서 처리)

선택: B. 코드 측면에서 처리하여 YAML 자체는 상대 경로(`./dg5f_right_retarget.urdf`)
로 두고 로드 시 절대화. 이것이 가장 이식성 높음.

### 1.5 docs/dex_retarget_setup.md — 신규
설치/실행 가이드. retarget_dev/models/dex_retarget/docs/setup.md +
manus_realtime.md를 teleop_dev 컨텍스트로 정리.

핵심 내용:
- pip 설치: `pip install dex_retargeting`, `numpy<2`, `mediapipe==0.10.21`
- 실행 명령:
  ```bash
  python3 -m sender.hand.manus_sender --target-ip <IP> --hand right \
    --retarget dex --sdk-mode ros2
  ```
- 25→21 리매핑 이슈 설명 + 트러블슈팅 (성공/실패 로그 형태)

## 2. manus_reader_ros2.py 수정

skeleton 파싱이 이전 정리에서 제거되었음. 다시 추가하되 25→21 리매핑까지 포함.

### 2.1 manus_reader.py: HandData 필드 추가
경로: `sender/hand/manus_reader.py` L61-67
```python
@dataclass
class HandData:
    joint_angles: np.ndarray = ...
    finger_spread: np.ndarray = ...
    wrist_pos: np.ndarray = ...
    wrist_quat: np.ndarray = ...
    hand_side: str = "right"
    timestamp: float = 0.0
    # 신규 — dex_retarget 등 skeleton 기반 mode 지원
    skeleton: Optional[np.ndarray] = None  # (21, 7) MANO 21-node, x,y,z,qw,qx,qy,qz
    has_skeleton: bool = False
```

### 2.2 manus_reader_ros2.py: skeleton 파싱 추가
경로: `sender/hand/manus_reader_ros2.py`
- import: `from sender.hand.gen3a_dex_retarget.manus_remap import _remap_to_mano_21`
  - 또는 더 깔끔하게: 리매핑 헬퍼는 새 모듈이 로드된 경우에만 사용 (선택적 import)
- 더 안전한 방식: `_glove_callback`에서 try/except import:
  ```python
  try:
      from sender.hand.gen3a_dex_retarget.manus_remap import _remap_to_mano_21
      _SKELETON_REMAP_OK = True
  except ImportError:
      _remap_to_mano_21 = None
      _SKELETON_REMAP_OK = False
  ```
- `_glove_callback` 내부:
  ```python
  skeleton = None
  has_skeleton = False
  if _SKELETON_REMAP_OK and msg.raw_nodes and len(msg.raw_nodes) > 0:
      skeleton = _remap_to_mano_21(msg.raw_nodes)
      has_skeleton = skeleton is not None
  hd = HandData(..., skeleton=skeleton, has_skeleton=has_skeleton)
  ```
- 진단 로그 1회성: `_remap_logged_ok` / `_remap_logged_fail` 패턴 (retarget_dev
  와 동일).
- standalone `main()`의 `num_lines`/출력 라인은 수정 불필요 (skeleton 표시는
  retarget_dev test가 별도로 처리).

### 2.3 manus_reader.py (subprocess 모드): 변경 없음
SDK subprocess 모드는 skeleton 메타데이터를 출력하지 않으므로 25→21 매핑
불가. dex_retarget 모드는 ros2 mode 전제. 사용자가 subprocess 모드 + dex
조합을 시도하면 sender에서 명확히 거부.

## 3. manus_sender.py 수정

### 3.1 argparse — `--retarget` choices 확장
경로: L170-172
```python
parser.add_argument("--retarget", default="none",
                    choices=["none", "ergo-direct", "dex"],
                    help="Retarget mode: none=raw, ergo-direct=[1A], dex=[3A] dex_retargeting")
```

### 3.2 conditional import 블록 추가
L250 직후 (1A 블록 이후, keyboard listener 시작 전):
```python
elif args.retarget == "dex":
    if sdk_mode != "ros2":
        print("[ERROR] --retarget dex requires --sdk-mode ros2 (skeleton metadata)")
        return
    import numpy as np
    from sender.hand.gen3a_dex_retarget import DexRetargetWrapper
    retarget = DexRetargetWrapper(
        hand_side=hand_side if hand_side != "both" else "right",
    )
    print(f"[Sender] Retarget: 3A-dex-retarget ({hand_side})")
```

### 3.3 `_apply_retarget()` 갱신
L279-285:
```python
def _apply_retarget(data):
    if retarget is None:
        return False
    method = retarget.get_method_name()
    if method.startswith("1A"):
        dg5f_q = retarget.retarget(ergonomics=data.joint_angles)
    elif method.startswith("3A"):
        if not data.has_skeleton:
            return False
        dg5f_q = retarget.retarget(skeleton=data.skeleton)
    else:
        return False
    data.joint_angles = dg5f_q.astype(np.float32)
    return True
```

### 3.4 docstring usage 예시 추가
L11-22 영역:
```
# [3A] dex_retargeting (skeleton-based, fingertip optimization)
python3 -m sender.hand.manus_sender --target-ip <ROBOT_IP> --hand right \
    --retarget dex --sdk-mode ros2
```

## 4. core/retarget_base.py docstring 보강
경로: `sender/hand/core/retarget_base.py` L29-37
```python
서브클래스마다 kwargs가 다름:
- 1A: retarget(ergonomics=ndarray[20])
- 3A: retarget(skeleton=ndarray[21,7])
```
(실제 인터페이스 변경 없음 — `**kwargs`이므로 ABC 변경 불필요)

## 5. environment.yaml 갱신
경로: `sender/environment.yaml`
```yaml
  - pip:
    - openvr>=1.26.701
    - "numpy<2"             # dex_retargeting / pinocchio ABI 호환
    - pynput>=1.7.6
    - pyyaml>=6.0
    - pygame>=2.0.0
    - dex_retargeting       # [3A] dex_retarget mode
    - "mediapipe==0.10.21"  # dex_retargeting examples 호환 (framework 모듈)
```

## 6. 문서 업데이트

### 6.1 docs/hand_manus_guide.md
- 데이터 스트림 표에 "Raw Skeleton (25→21 MANO)" 행 다시 추가
- "Mode B" 옆에 "Mode C: [3A] dex_retarget" 섹션 추가 (실행 명령 + 흐름도)
- 트러블슈팅에 "remap incomplete" 경고 항목 추가

### 6.2 sender/hand/docs/hand_retarget_1a_impl.md
- "(향후 추가 예정)" 트리에서 `gen3a_multi_cost/` 옆에 `gen3a_dex_retarget/`
  실현 표시
- 1B/1C는 아직 미구현 그대로

### 6.3 docs/setup_guide.md
- 조종 PC sender 패키지 표에 `dex_retargeting`, `mediapipe==0.10.21`,
  `numpy<2` 행 추가
- "Manus sender" 실행 예시에 `--retarget dex --sdk-mode ros2` 한 줄 추가

### 6.4 sender/hand/docs/dex_retarget_setup.md (신규)
설치/실행/트러블슈팅 단일 페이지 — pip 명령, manus_data_publisher 실행 순서,
remap 로그 해석, scaling_factor 튜닝.

## 7. 보존 (touch 안 함)
- `sender/hand/sdk/SDKClient_Linux/` — 빌드 산출물 그대로
- `sender/hand/sdk/ROS2/manus_ros2/` — Manus 공식 ROS2 패키지
- `sender/hand/gen1a_ergo_direct/` — 기존 [1A] 그대로
- `core/retarget_base.py`, `core/dg5f_config.py`, `core/filters.py` — 변경 없음
- robot/hand/receiver.py — `retargeted` 플래그로 자동 우회 동작 그대로

## 8. 의존 파일 인용
| 출처 | 사용처 |
|---|---|
| `retarget_dev/models/dex_retarget/dex_retarget_model.py` | DexRetargetWrapper 패턴 (특히 `_build_ref_value`) |
| `retarget_dev/sensing/manus/ros2_provider.py:51-123` | _MANO_REMAP + _remap_to_mano_21 그대로 이식 |
| `retarget_dev/sensing/manus/ros2_provider.py:282-302` | 1회성 진단 로그 (`_diagnose_remap`) 패턴 |
| `retarget_dev/sensing/manus/manus_sensing.py:74-90` | wrist 원점화 (DexRetargetWrapper.retarget 내부) |
| `retarget_dev/models/dex_retarget/main.py:97-175` | 실시간 루프 패턴 참고 (sender는 다른 구조이지만 호출 순서) |
| `retarget_dev/models/dex_retarget/config/dg5f_right_*.{yml,urdf}` | 로컬 복사 후 urdf_path 동적 절대화 |
| `sender/hand/gen1a_ergo_direct/ergo_direct.py` | 동일 폴더/파일 패턴, HandRetargetBase 상속, __init__.py 구조 |

## 9. 검증

### 9.1 사전 조건
- 조종 PC: `conda activate teleop_operator` (environment.yaml 갱신본 적용)
- `pip install dex_retargeting "numpy<2" mediapipe==0.10.21`
- ROS2 + manus_data_publisher 실행 중

### 9.2 단위 검증
```bash
# 1. import + 객체 생성
python3 -c "
from sender.hand.gen3a_dex_retarget import DexRetargetWrapper
import numpy as np
rt = DexRetargetWrapper(hand_side='right')
fake_skel = np.zeros((21, 7), dtype=np.float32)
fake_skel[:, 3] = 1.0  # qw=1
q = rt.retarget(skeleton=fake_skel)
assert q.shape == (20,), q.shape
print('[OK] DexRetargetWrapper basic')
"

# 2. manus_remap 단위 테스트
python3 -c "
from sender.hand.gen3a_dex_retarget.manus_remap import _remap_to_mano_21
print('[OK] manus_remap import')
"

# 3. argparse choices
python3 -m sender.hand.manus_sender --help | grep -A2 retarget
# → none, ergo-direct, dex 모두 표시
```

### 9.3 통합 실행
```bash
# T1: manus_data_publisher
ros2 run manus_ros2 manus_data_publisher

# T2: sender (dex 모드)
python3 -m sender.hand.manus_sender --target-ip <ROBOT_IP> --hand right \
  --retarget dex --sdk-mode ros2

# 첫 메시지 수신 후 로그:
# [Sender] Retarget: 3A-dex-retarget (right)
# (manus_reader_ros2 1회성) Manus skeleton: 25 raw → MANO 21 (remap OK)

# T3 (robot PC): receiver
python3 -m robot.hand.receiver --hand right
```

### 9.4 회귀
- `--retarget none` 그대로 동작
- `--retarget ergo-direct` 그대로 동작
- `--retarget dex --sdk-mode subprocess` → 명확한 에러 메시지

## 10. 커밋 전략
의미 단위 분할:
1. `feat: copy dg5f URDF + dexpilot/vector configs from retarget_dev`
2. `feat: add 25→21 MANO remap helper for Manus skeleton`
3. `feat: re-add HandData.skeleton + ROS2 skeleton parsing`
4. `feat: add gen3a_dex_retarget (DexRetargetWrapper)`
5. `feat: register dex retarget mode in manus_sender`
6. `chore: update environment.yaml + docs for [3A] dex_retarget`

또는 단일 커밋 `feat: add gen3a_dex_retarget — Manus skeleton + DexPilot`.
사용자의 자동 commit 워크플로우에 따라 plan 완료 후 결정.
