# [1A] Manus Ergonomics Direct Mapping 구현 계획

> 작성일: 2026-04-08
> 기반: `src/retarget_dev/docs/retarget_history.md` Part II, Section 9 [1A]
> 목표: 다른 터미널에서 이 문서만으로 구현 가능하도록 작성

---

## 1. 현재 코드 vs 계획 분석

### 1.1 현재 코드 구조 (변경 전)

```
src/teleop_dev/
├── sender/hand/
│   ├── manus_sender.py         # Manus 읽기 + retarget + UDP 전송
│   ├── manus_reader.py         # SDKClient subprocess → HandData
│   ├── manus_reader_ros2.py    # ROS2 topic → HandData (대안)
│   ├── manus_config.py         # YAML 설정 로더
│   ├── hand_visualizer.py      # 시각화
│   ├── calibrate.py            # 캘리브레이션 CLI
│   ├── dg5f_fk.py              # DEPRECATED (redirect)
│   ├── vector_retarget.py      # DEPRECATED (redirect)
│   ├── sdk/                    # Manus SDK binary + ROS2 bridge
│   └── retarget/               # ★ 기존 리타게팅 (삭제 대상)
│       ├── __init__.py         # create_retarget() factory
│       ├── base.py             # RetargetBase ABC (skeleton 기반)
│       ├── angle_extractor.py  # 3D position → 20 angles
│       ├── direct_mapping.py   # skeleton → angles → linear map
│       ├── vector_retarget.py  # skeleton → fingertip vector opt (2.5gen)
│       ├── dg5f_fk.py          # Pinocchio FK
│       └── config/direct_mapping.yaml
│
├── robot/hand/
│   ├── receiver.py             # UDP 수신 + retarget + ROS2 publish
│   ├── retarget.py             # ★ ManusToD5FRetarget (기존 Tesollo 알고리즘)
│   ├── dg5f_ros2_client.py     # ROS2 MultiDOFCommand publisher
│   ├── tesollo_config.py       # YAML config
│   └── bak/                    # Deprecated Modbus client
│
└── protocol/
    └── hand_protocol.py        # HandData dataclass, UDP format
```

### 1.2 기존 코드의 문제점

| 문제 | 설명 |
|------|------|
| **입력 혼재** | `sender/hand/retarget/`의 base.py는 skeleton(N,7)을 기대하는데, 1A는 ergonomics(20 float)만 사용 |
| **리타게팅 위치 분산** | sender에 vector_retarget/direct_mapping, robot에 ManusToD5FRetarget → 같은 역할이 두 곳에 |
| **base 인터페이스 부적합** | RetargetBase.retarget(skeleton)은 skeleton 전용 → ergonomics, MediaPipe 등 다른 입력 지원 불가 |
| **확장성 부족** | [1B], [2A], [3A] 등 다른 세대 방법론을 깔끔하게 추가할 구조가 아님 |
| **DG5F FK 중복** | sender/hand/retarget/dg5f_fk.py와 retarget_dev/models/fingertip_ik/dg5f_fk.py 중복 |

### 1.3 계획하는 새 구조

```
src/teleop_dev/
├── sender/hand/
│   ├── manus_sender.py            # 수정: retarget 호출 경로 변경
│   ├── manus_reader.py            # 재사용 (변경 없음)
│   ├── manus_reader_ros2.py       # 재사용 (변경 없음)
│   ├── manus_config.py            # 재사용 (변경 없음)
│   ├── hand_visualizer.py         # 재사용
│   ├── calibrate.py               # 재사용
│   ├── sdk/                       # 재사용
│   │
│   ├── core/                      # ★ 새로 생성: 세대 공통 인프라
│   │   ├── __init__.py
│   │   ├── retarget_base.py       # 새 ABC: 입력 타입별 분리
│   │   ├── dg5f_config.py         # Joint names, limits, link lengths
│   │   └── filters.py             # EMA filter
│   │
│   ├── gen1a_ergo_direct/         # ★ 새로 생성: [1A] Ergonomics Direct
│   │   ├── __init__.py
│   │   └── ergo_direct.py         # ErgoDirectRetarget 클래스
│   │
│   └── retarget/                  # ★ 삭제 대상 (기존 코드)
│       └── (기존 파일들 전부)
│
├── robot/hand/
│   ├── receiver.py                # 수정: 새 retarget 사용
│   ├── dg5f_ros2_client.py        # 재사용 (변경 없음)
│   ├── tesollo_config.py          # 재사용
│   ├── retarget.py                # ★ 삭제 대상 (ManusToD5FRetarget)
│   │
│   ├── core/                      # ★ 새로 생성: robot hand 공통
│   │   ├── __init__.py
│   │   ├── dg5f_commander.py      # DG5F 명령 래퍼 (ROS2 client + safety)
│   │   └── dg5f_config.py         # sender/hand/core/dg5f_config.py 재사용 or import
│   │
│   └── bak/                       # 유지 (deprecated)
│
└── protocol/
    └── hand_protocol.py           # 재사용 (변경 없음)
```

---

## 2. 기존 코드 처분 분류

### 2.1 재사용 (변경 없음)

| 파일 | 이유 |
|------|------|
| `sender/hand/manus_reader.py` | SDKClient 파싱 로직. ergonomics 데이터 읽기에 그대로 사용 |
| `sender/hand/manus_reader_ros2.py` | ROS2 대안 입력. 동일 |
| `sender/hand/manus_config.py` | YAML 설정 로더. 그대로 |
| `sender/hand/sdk/` | Manus SDK binary. 변경 불필요 |
| `sender/hand/hand_visualizer.py` | 시각화. 독립 모듈 |
| `robot/hand/dg5f_ros2_client.py` | ROS2 MultiDOFCommand publisher. 완벽 재사용 |
| `robot/hand/tesollo_config.py` | 설정 로더. 그대로 |
| `protocol/hand_protocol.py` | UDP 프로토콜. 변경 없음 |

### 2.2 삭제 대상 (새 구현으로 대체)

| 파일 | 이유 |
|------|------|
| `sender/hand/retarget/__init__.py` | 기존 factory → 새 구조로 대체 |
| `sender/hand/retarget/base.py` | skeleton 전용 ABC → 새 retarget_base.py로 대체 |
| `sender/hand/retarget/angle_extractor.py` | 3D→angle 추출기. 1A는 ergonomics 직접 사용이므로 불필요 (1C/2A에서 재구현 예정) |
| `sender/hand/retarget/direct_mapping.py` | 기존 1.5세대. 새 gen1a로 대체 |
| `sender/hand/retarget/vector_retarget.py` | 기존 2.5세대. 추후 gen2a/gen3a로 재구현 예정 |
| `sender/hand/retarget/dg5f_fk.py` | 추후 core/에 통합 (2A 이후 필요 시) |
| `sender/hand/retarget/config/` | 기존 calibration yaml |
| `sender/hand/dg5f_fk.py` | DEPRECATED redirect |
| `sender/hand/vector_retarget.py` | DEPRECATED redirect |
| `robot/hand/retarget.py` | ManusToD5FRetarget → 새 구조로 대체 |

> **주의**: 삭제 전 git commit 으로 백업 권장. 또는 `bak/` 폴더로 이동.

### 2.3 수정 대상

| 파일 | 수정 내용 |
|------|----------|
| `sender/hand/manus_sender.py` | retarget import 경로 변경: `retarget/` → `gen1a_ergo_direct/` |
| `robot/hand/receiver.py` | retarget import 변경: `retarget.ManusToD5FRetarget` → 새 경로 or 없앰 |

### 2.4 참고용 보존 (로직 추출)

| 파일 | 추출할 로직 |
|------|-----------|
| `robot/hand/retarget.py` (ManusToD5FRetarget) | **direction 배열, calibration factors, posture constraints, Thumb swap/offset 로직** → 이것이 Tesollo 공식 알고리즘이므로 새 ergo_direct.py의 기반 |

---

## 3. 새 파일 구조 상세

### 3.1 디렉토리 레이아웃

```
src/teleop_dev/sender/hand/
├── core/                          # 세대 공통 인프라 (1A, 1B, 2A 등 모두 사용)
│   ├── __init__.py
│   ├── retarget_base.py           # 새 ABC
│   ├── dg5f_config.py             # DG5F 관절 정보 (limits, names, link lengths)
│   └── filters.py                 # EMA filter, One Euro Filter
│
├── gen1a_ergo_direct/             # [1A] Manus Ergonomics → Direct Mapping
│   ├── __init__.py
│   └── ergo_direct.py             # ErgoDirectRetarget 클래스
│
├── (향후 추가 예정)
│   gen1b_skeleton_quat/           # [1B] Manus Skeleton → Quat Decomposition
│   gen1c_mediapipe_angles/        # [1C] MediaPipe → Vector Angles
│   gen2a_fingertip_ik/            # [2A] Manus Skeleton → Fingertip IK
│   gen3a_multi_cost/              # [3A] Multi-Cost Optimization
│   ...
```

```
src/teleop_dev/robot/hand/
├── core/                          # robot hand 공통
│   ├── __init__.py
│   └── dg5f_commander.py          # DG5F ROS2 명령 래퍼
│
├── dg5f_ros2_client.py            # 기존 유지
├── receiver.py                    # 수정
└── tesollo_config.py              # 기존 유지
```

---

## 4. 파일별 구현 상세

### 4.1 `sender/hand/core/retarget_base.py` — 새 ABC

```python
"""세대 공통 리타게팅 베이스 클래스.

1A, 1B, 2A 등 모든 세대의 retarget 클래스가 이 ABC를 상속.
기존 RetargetBase(skeleton 전용)와 달리, 입력 타입이 자유로움.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np


class HandRetargetBase(ABC):
    """인간 손 데이터 → DG5F 20-joint 각도 변환 ABC.

    Attributes
    ----------
    hand_side : str
        "left" or "right"
    """

    def __init__(self, hand_side: str = "right"):
        self._hand_side = hand_side

    @property
    def hand_side(self) -> str:
        return self._hand_side

    @abstractmethod
    def retarget(self, **kwargs) -> np.ndarray:
        """입력 데이터 → DG5F joint angles (20,) radians.

        서브클래스마다 kwargs가 다름:
        - 1A: retarget(ergonomics=ndarray[20])
        - 1B: retarget(skeleton=ndarray[25,7])
        - 2A: retarget(skeleton=ndarray[25,7], ergonomics=ndarray[20])
        - 1C: retarget(landmarks=ndarray[21,3])
        """
        ...

    @abstractmethod
    def get_method_name(self) -> str:
        """리타게팅 방법 이름 (로그용). e.g., '1A-ergo-direct'"""
        ...

    def get_debug_info(self) -> Optional[dict]:
        """선택적 디버그 정보."""
        return None
```

### 4.2 `sender/hand/core/dg5f_config.py` — DG5F 관절 정보

```python
"""DG5F 관절 이름, 한계, 링크 길이 등 상수.

URDF(`dg5f_right.urdf`)에서 추출한 값. 모든 세대에서 공통 사용.
"""

import math
import numpy as np
from dataclasses import dataclass


NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",  # Thumb
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",  # Index
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",  # Middle
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",  # Ring
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",  # Pinky
]

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]


@dataclass
class JointLimit:
    lower: float  # radians
    upper: float  # radians


# URDF에서 직접 추출한 값
RIGHT_JOINT_LIMITS = [
    # Thumb
    JointLimit(-0.384, 0.890),   # rj_dg_1_1 X abd
    JointLimit(-math.pi, 0.0),   # rj_dg_1_2 Z opp
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_1_3 X flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_1_4 X flex
    # Index
    JointLimit(-0.419, 0.611),   # rj_dg_2_1 X abd
    JointLimit(0.0, 2.007),      # rj_dg_2_2 Y MCP flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_2_3 Y PIP flex
    JointLimit(-math.pi/2, math.pi/2),  # rj_dg_2_4 Y DIP flex
    # Middle
    JointLimit(-0.611, 0.611),
    JointLimit(0.0, 1.955),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Ring
    JointLimit(-0.611, 0.419),
    JointLimit(0.0, 1.902),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Pinky
    JointLimit(-0.017, 1.047),   # rj_dg_5_1 Z abd
    JointLimit(-0.419, 0.611),   # rj_dg_5_2 X spread
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
]

LEFT_JOINT_LIMITS = [
    # Thumb (mirrored)
    JointLimit(-0.890, 0.384),
    JointLimit(0.0, math.pi),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Index
    JointLimit(-0.611, 0.419),
    JointLimit(0.0, 2.007),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Middle
    JointLimit(-0.611, 0.611),
    JointLimit(0.0, 1.955),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Ring
    JointLimit(-0.419, 0.611),
    JointLimit(0.0, 1.902),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
    # Pinky
    JointLimit(-1.047, 0.017),
    JointLimit(-0.611, 0.419),
    JointLimit(-math.pi/2, math.pi/2),
    JointLimit(-math.pi/2, math.pi/2),
]


def get_joint_names(hand_side: str) -> list[str]:
    return RIGHT_JOINT_NAMES if hand_side == "right" else LEFT_JOINT_NAMES

def get_joint_limits(hand_side: str) -> list[JointLimit]:
    return RIGHT_JOINT_LIMITS if hand_side == "right" else LEFT_JOINT_LIMITS

def get_limits_arrays(hand_side: str) -> tuple[np.ndarray, np.ndarray]:
    """(lower[20], upper[20]) numpy 배열 반환."""
    limits = get_joint_limits(hand_side)
    lower = np.array([lim.lower for lim in limits])
    upper = np.array([lim.upper for lim in limits])
    return lower, upper
```

### 4.3 `sender/hand/core/filters.py` — EMA Filter

```python
"""시간축 필터. 모든 세대에서 공통 사용."""

import numpy as np


class EMAFilter:
    """Exponential Moving Average filter for joint angles.

    Parameters
    ----------
    alpha : float
        0~1. 1에 가까울수록 새 값 반영 (반응 빠름, 떨림 많음).
    size : int
        필터 대상 벡터 크기 (DG5F: 20).
    """

    def __init__(self, alpha: float = 0.4, size: int = 20):
        self._alpha = alpha
        self._prev = None
        self._size = size

    def filter(self, value: np.ndarray) -> np.ndarray:
        if self._prev is None:
            self._prev = value.copy()
            return value.copy()
        filtered = self._alpha * value + (1.0 - self._alpha) * self._prev
        self._prev = filtered.copy()
        return filtered

    def reset(self):
        self._prev = None
```

### 4.4 `sender/hand/gen1a_ergo_direct/ergo_direct.py` — [1A] 핵심 구현

```python
"""[1A] Manus Ergonomics → DG5F Direct Mapping.

Manus glove의 ergonomics 각도(20 floats, radians)를
DG5F 관절 각도(20 floats, radians)로 직접 변환.

알고리즘 기반: Tesollo 공식 manus_retarget + retarget_history.md Section 9 [1A].

파이프라인:
    Manus Ergo (20, rad)
    → direction * calibration 적용
    → per-finger 변환 (Thumb swap/offset, Index/Middle/Ring/Pinky offset)
    → posture constraint
    → joint limit clamp
    → EMA filter
    → DG5F (20, rad)
"""

import math
import numpy as np
from typing import Optional

# core는 sender/hand/core에서 import
from sender.hand.core.retarget_base import HandRetargetBase
from sender.hand.core.dg5f_config import (
    NUM_JOINTS, get_limits_arrays, get_joint_names,
)
from sender.hand.core.filters import EMAFilter


# ─── 부호 방향 (Manus → DG5F 축 방향 보정) ───
# 기존 ManusToD5FRetarget의 direction 배열 그대로 가져옴 (실기 검증 완료된 값)
_RIGHT_DIRECTIONS = np.array([
     1, -1,  1,  1,   # Thumb
    -1,  1,  1,  1,   # Index
    -1,  1,  1,  1,   # Middle
    -1,  1,  1,  1,   # Ring
     1, -1,  1,  1,   # Pinky
], dtype=np.float64)

_LEFT_DIRECTIONS = np.array([
    -1,  1, -1, -1,
     1,  1,  1,  1,
     1,  1,  1,  1,
     1,  1,  1,  1,
    -1,  1,  1,  1,
], dtype=np.float64)

# ─── 기본 캘리브레이션 팩터 (관절 반응성 조절) ───
# Tesollo 공식 값. 실기에서 검증된 default.
_DEFAULT_CAL_FACTORS = np.array([
    1.0, 1.6, 1.3, 1.3,   # Thumb
    1.0, 1.0, 1.3, 1.7,   # Index
    1.0, 1.0, 1.3, 1.7,   # Middle
    1.0, 1.0, 1.3, 1.7,   # Ring
    1.0, 1.0, 1.0, 1.0,   # Pinky
], dtype=np.float64)

DEG2RAD = math.pi / 180.0


class ErgoDirectRetarget(HandRetargetBase):
    """[1A] Manus Ergonomics → DG5F Direct Mapping.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    calibration_factors : ndarray[20] or None
        Per-joint scaling. None → Tesollo 기본값 사용.
    ema_alpha : float
        EMA 필터 강도 (0.0=필터없음, 1.0=필터없음). 0.3~0.5 권장.
    """

    def __init__(self, hand_side: str = "right",
                 calibration_factors: np.ndarray | None = None,
                 ema_alpha: float = 0.4):
        super().__init__(hand_side)

        self._is_right = (hand_side == "right")
        self._directions = _RIGHT_DIRECTIONS if self._is_right else _LEFT_DIRECTIONS
        self._cal = calibration_factors if calibration_factors is not None else _DEFAULT_CAL_FACTORS.copy()
        self._lower, self._upper = get_limits_arrays(hand_side)
        self._joint_names = get_joint_names(hand_side)
        self._ema = EMAFilter(alpha=ema_alpha, size=NUM_JOINTS)

        # 디버그용
        self._last_raw = np.zeros(NUM_JOINTS)
        self._last_transformed = np.zeros(NUM_JOINTS)

    def retarget(self, ergonomics: np.ndarray, **kwargs) -> np.ndarray:
        """Manus ergonomics (20, radians) → DG5F joint angles (20, radians).

        Parameters
        ----------
        ergonomics : ndarray[20]
            Manus 글러브 ergonomics 값 (radians).
            Layout: [Thumb(4), Index(4), Middle(4), Ring(4), Pinky(4)]
            각 finger: [Spread, MCP_Flex, PIP_Flex, DIP_Flex]

        Returns
        -------
        ndarray[20] — DG5F 목표 관절 각도 (radians), joint limit 내로 clamp됨.
        """
        assert ergonomics.shape == (NUM_JOINTS,), f"Expected (20,), got {ergonomics.shape}"
        self._last_raw = ergonomics.copy()

        # 1) Degree로 변환 (Tesollo 변환식이 degree 기반)
        q_deg = np.degrees(ergonomics)
        qd = np.zeros(NUM_JOINTS, dtype=np.float64)

        # 2) Per-finger 변환
        #    Thumb: spread/flex swap + offset (Tesollo 공식)
        qd[0] = (58.5 - q_deg[1]) * DEG2RAD   # ThumbMCPStretch → rj_1_1, inverted
        qd[1] = (q_deg[0] + 20.0) * DEG2RAD   # ThumbMCPSpread → rj_1_2, offset
        qd[2] = q_deg[2] * DEG2RAD             # ThumbPIPStretch → rj_1_3
        qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * DEG2RAD  # DIP averaged

        #    Index
        qd[4] = q_deg[4] * DEG2RAD
        qd[5] = q_deg[5] * DEG2RAD
        qd[6] = (q_deg[6] - 40.0) * DEG2RAD   # PIP -40° offset
        qd[7] = 0.5 * (q_deg[6] + q_deg[7]) * DEG2RAD

        #    Middle
        qd[8] = q_deg[8] * DEG2RAD
        qd[9] = q_deg[9] * DEG2RAD
        qd[10] = (q_deg[10] - 30.0) * DEG2RAD  # PIP -30° offset
        qd[11] = 0.5 * (q_deg[10] + q_deg[11]) * DEG2RAD

        #    Ring
        qd[12] = q_deg[12] * DEG2RAD
        qd[13] = q_deg[13] * DEG2RAD
        qd[14] = q_deg[14] * DEG2RAD
        qd[15] = q_deg[15] * DEG2RAD

        #    Pinky (conditional spread)
        if q_deg[17] > 55.0 and q_deg[18] > 25.0 and q_deg[19] > 20.0:
            spread_mult = 2.0
        else:
            spread_mult = 1.0 / 1.5
        qd[16] = q_deg[16] * spread_mult * DEG2RAD
        qd[17] = q_deg[17] * DEG2RAD
        qd[18] = q_deg[18] * DEG2RAD
        qd[19] = q_deg[19] * DEG2RAD

        # 3) Direction + calibration
        qd *= self._cal * self._directions

        # 4) Posture constraints
        self._apply_posture_constraints(qd)

        # 5) Joint limit clamp
        qd = np.clip(qd, self._lower, self._upper)

        # 6) EMA filter
        qd = self._ema.filter(qd)

        self._last_transformed = qd.copy()
        return qd

    def _apply_posture_constraints(self, qd: np.ndarray):
        """해부학적으로 불가능한 관절값 보정 (in-place)."""
        if self._is_right:
            if qd[0] < 0: qd[0] = 0.0    # Thumb base >= 0
            if qd[2] < 0: qd[2] = 0.0    # Thumb PIP >= 0
            if qd[3] < 0: qd[3] = 0.0    # Thumb DIP >= 0
            for i in [4, 8, 12]:          # Index/Middle/Ring spread <= 0
                if qd[i] > 0: qd[i] = 0.0
            if qd[16] > 0: qd[16] = 0.0  # Pinky spread <= 0
        else:
            if qd[0] > 0: qd[0] = 0.0
            if qd[2] > 0: qd[2] = 0.0
            if qd[3] > 0: qd[3] = 0.0
            for i in [4, 8, 12]:
                if qd[i] < 0: qd[i] = 0.0
            if qd[16] < 0: qd[16] = 0.0

    def get_method_name(self) -> str:
        return "1A-ergo-direct"

    def get_debug_info(self) -> dict:
        return {
            "method": self.get_method_name(),
            "raw_ergo_deg": np.degrees(self._last_raw).tolist(),
            "dg5f_deg": np.degrees(self._last_transformed).tolist(),
        }

    def set_calibration_factors(self, factors: np.ndarray):
        """런타임 캘리브레이션 팩터 업데이트."""
        assert factors.shape == (NUM_JOINTS,)
        self._cal = factors.copy()

    def reset_filter(self):
        """EMA 필터 초기화."""
        self._ema.reset()
```

### 4.5 `robot/hand/core/dg5f_commander.py` — DG5F 명령 래퍼

```python
"""DG5F 명령 래퍼.

dg5f_ros2_client.py를 감싸서 safety + logging 추가.
모든 세대의 robot/hand 코드에서 공통 사용.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from robot.hand.dg5f_ros2_client import DG5FROS2Client
from sender.hand.core.dg5f_config import get_limits_arrays, NUM_JOINTS


class DG5FCommander:
    """DG5F 명령 전송 래퍼 with safety clamp.

    Parameters
    ----------
    hand_side : str
        "left" or "right"
    motion_time_ms : int
        모션 시간 (ms)
    """

    def __init__(self, hand_side: str = "right", motion_time_ms: int = 50):
        self._hand_side = hand_side
        self._lower, self._upper = get_limits_arrays(hand_side)

        # ROS2 client 생성
        self._client = DG5FROS2Client(hand_side=hand_side,
                                       motion_time_ms=motion_time_ms)

    @property
    def node(self) -> Node:
        """rclpy spin에 사용할 ROS2 노드."""
        return self._client

    def send(self, angles_rad: np.ndarray):
        """Safety clamp 후 DG5F에 명령 전송.

        Parameters
        ----------
        angles_rad : ndarray[20]
            목표 관절 각도 (radians).
        """
        assert angles_rad.shape == (NUM_JOINTS,)
        safe = np.clip(angles_rad, self._lower, self._upper)
        self._client.set_positions(safe)

    def get_feedback(self) -> np.ndarray:
        """현재 관절 위치 피드백."""
        return self._client.get_positions()

    @property
    def has_feedback(self) -> bool:
        return self._client.has_feedback
```

---

## 5. manus_sender.py 수정 사항

기존 `manus_sender.py`에서 retarget 부분만 변경:

```python
# 변경 전:
from sender.hand.retarget import create_retarget
retarget = create_retarget(mode="direct", hand_side="right")
dg5f_q = retarget.retarget(hand_data.skeleton)

# 변경 후:
from sender.hand.gen1a_ergo_direct.ergo_direct import ErgoDirectRetarget
retarget = ErgoDirectRetarget(hand_side="right", ema_alpha=0.4)
dg5f_q = retarget.retarget(ergonomics=hand_data.joint_angles)
```

핵심 변경: `skeleton (N,7)` 대신 `joint_angles (20,)` (= ergonomics)를 직접 전달.

---

## 6. receiver.py 수정 사항

기존 receiver에서 ManusToD5FRetarget를 사용하던 부분을 제거하거나 통과(passthrough)로 변경:

```python
# 변경 전:
from robot.hand.retarget import ManusToD5FRetarget
self._retarget = ManusToD5FRetarget(hand_side=hand_side)
if not pkt.get("retargeted", False):
    angles = self._retarget.retarget(np.array(pkt["joint_angles"]))

# 변경 후:
# sender에서 이미 retarget 완료 → receiver는 passthrough
# pkt["retargeted"] == True일 때 그대로 사용
angles = np.array(pkt["joint_angles"])  # 이미 DG5F 각도
```

또는 receiver에서도 retarget 가능하게 유지하려면:
```python
from sender.hand.gen1a_ergo_direct.ergo_direct import ErgoDirectRetarget
self._retarget = ErgoDirectRetarget(hand_side=hand_side)
```

---

## 7. 구현 순서 (Step-by-step)

### Step 1: core/ 생성 (공통 인프라)
```bash
# 파일 생성
mkdir -p src/teleop_dev/sender/hand/core
touch src/teleop_dev/sender/hand/core/__init__.py
# 작성: retarget_base.py, dg5f_config.py, filters.py
# 내용: Section 4.1, 4.2, 4.3 참조
```

### Step 2: gen1a_ergo_direct/ 생성
```bash
mkdir -p src/teleop_dev/sender/hand/gen1a_ergo_direct
touch src/teleop_dev/sender/hand/gen1a_ergo_direct/__init__.py
# 작성: ergo_direct.py
# 내용: Section 4.4 참조
# 핵심: ManusToD5FRetarget(robot/hand/retarget.py)의 변환 로직을
#        HandRetargetBase 구조로 재구성
```

### Step 3: robot/hand/core/ 생성
```bash
mkdir -p src/teleop_dev/robot/hand/core
touch src/teleop_dev/robot/hand/core/__init__.py
# 작성: dg5f_commander.py
# 내용: Section 4.5 참조
```

### Step 4: manus_sender.py 수정
```
- retarget import 경로 변경
- --retarget 옵션에 "ergo-direct" (또는 "1a") 추가
- retarget.retarget(ergonomics=hand_data.joint_angles) 호출
```

### Step 5: receiver.py 수정
```
- ManusToD5FRetarget import 제거
- sender에서 retarget 완료 시 passthrough 처리
- 또는 ErgoDirectRetarget으로 교체
```

### Step 6: 기존 코드 정리
```bash
# 백업 후 삭제
git add -A && git commit -m "backup before hand retarget restructure"

# 삭제 대상:
rm -rf src/teleop_dev/sender/hand/retarget/
rm src/teleop_dev/sender/hand/dg5f_fk.py
rm src/teleop_dev/sender/hand/vector_retarget.py
rm src/teleop_dev/robot/hand/retarget.py
```

### Step 7: 테스트
```bash
# 단위 테스트: ErgoDirectRetarget 동작 확인
python3 -c "
from sender.hand.gen1a_ergo_direct.ergo_direct import ErgoDirectRetarget
import numpy as np
r = ErgoDirectRetarget('right')
mock = np.array([0.1,0.5,0.6,0.4, 0.05,0.8,0.9,0.5, 0.0,0.7,0.8,0.4,
                 -0.05,0.6,0.7,0.3, -0.1,0.5,0.6,0.25])
print(np.degrees(r.retarget(ergonomics=mock)))
"

# 통합 테스트: sender → UDP → receiver → DG5F
# (기존 tests/ 참조하여 업데이트)
```

---

## 8. 향후 확장 시 추가할 폴더

| 폴더 | 세대 | 추가 시점 | core/ 의존 |
|------|------|----------|-----------|
| `gen1b_skeleton_quat/` | [1B] Skeleton Quat | 1A 검증 후 | retarget_base.py, dg5f_config.py, filters.py |
| `gen1c_mediapipe_angles/` | [1C] MediaPipe | 1A 병렬 | retarget_base.py, dg5f_config.py, filters.py |
| `gen2a_fingertip_ik/` | [2A] Fingertip IK | Phase 2 | 위 + dg5f_fk.py (core/에 추가) |
| `gen3a_multi_cost/` | [3A] Multi-Cost Opt | Phase 3 | 위 + cost_functions.py, optimizer.py (core/에 추가) |

core/에 추가될 예정 파일:
- `core/dg5f_fk.py` — Pinocchio FK (2A부터 필요)
- `core/cost_functions.py` — 다중 cost (3A)
- `core/optimizer.py` — scipy wrapper (3A)

---

## 9. 핵심 구현 포인트 (실수 방지 체크리스트)

- [ ] `manus_reader.py`의 `joint_angles`가 이미 radians인지 확인 (SDK에서 degrees로 오면 변환 필요)
- [ ] Direction 배열 부호: 기존 ManusToD5FRetarget에서 실기 검증된 값 그대로 사용
- [ ] Thumb swap: Manus [Spread, Stretch, PIP, DIP] → DG5F [rj_1_1, rj_1_2, rj_1_3, rj_1_4] 매핑에서 index 0↔1 swap + offset 적용
- [ ] Pinky 조건부 spread: curl 상태에 따라 spread multiplier 변경
- [ ] EMA filter는 retarget 후, clamp 후에 적용 (순서 중요)
- [ ] `protocol/hand_protocol.py`의 `retargeted` 플래그를 True로 설정하여 receiver에서 중복 변환 방지
