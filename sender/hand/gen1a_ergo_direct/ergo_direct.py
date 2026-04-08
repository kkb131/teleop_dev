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

from sender.hand.core.retarget_base import HandRetargetBase
from sender.hand.core.dg5f_config import (
    NUM_JOINTS, get_limits_arrays, get_joint_names,
)
from sender.hand.core.filters import EMAFilter


# ─── 부호 방향 (Manus → DG5F 축 방향 보정) ───
# 기존 ManusToD5FRetarget의 direction 배열 그대로 (실기 검증 완료된 값)
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
        EMA 필터 강도 (0.0=이전값유지, 1.0=필터없음). 0.3~0.5 권장.
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

        # 2포즈 캘리브레이션 (rest/fist min-max normalization)
        self._rest_values: Optional[np.ndarray] = None   # open hand ergo (20,)
        self._fist_values: Optional[np.ndarray] = None    # fist ergo (20,)
        self._rom_scale: Optional[np.ndarray] = None      # dg5f_range / human_rom
        self._rom_offset: Optional[np.ndarray] = None     # -rest * scale
        self._is_rom_calibrated = False

        # 디버그용
        self._last_raw = np.zeros(NUM_JOINTS)
        self._last_transformed = np.zeros(NUM_JOINTS)

    def calibrate_rest(self, ergonomics: np.ndarray):
        """Record open-hand ergonomics as rest pose (Step 1 of 2).

        Parameters
        ----------
        ergonomics : ndarray[20]
            Averaged ergonomics from open-hand pose (radians).
        """
        self._rest_values = ergonomics.copy()
        print(f"[1A-Cal] Rest pose recorded. "
              f"MCP values: {[f'{np.degrees(ergonomics[f*4+1]):.1f}°' for f in range(5)]}")

    def calibrate_fist(self, ergonomics: np.ndarray):
        """Record fist ergonomics and compute ROM scale (Step 2 of 2).

        Requires calibrate_rest() to be called first.
        Computes: scale[i] = dg5f_range[i] / |human_rom[i]|

        Parameters
        ----------
        ergonomics : ndarray[20]
            Averaged ergonomics from fist pose (radians).
        """
        if self._rest_values is None:
            print("[1A-Cal] ERROR: call calibrate_rest() first!")
            return

        self._fist_values = ergonomics.copy()
        human_rom = self._fist_values - self._rest_values
        min_rom = np.radians(5.0)  # ignore ROM < 5° (noise)

        self._rom_scale = np.ones(NUM_JOINTS, dtype=np.float64)
        self._rom_offset = np.zeros(NUM_JOINTS, dtype=np.float64)

        for i in range(NUM_JOINTS):
            dg5f_range = self._upper[i] - self._lower[i]
            if abs(human_rom[i]) > min_rom:
                self._rom_scale[i] = dg5f_range / abs(human_rom[i])
                self._rom_offset[i] = -self._rest_values[i] * self._rom_scale[i]

        self._is_rom_calibrated = True
        print(f"[1A-Cal] Fist calibrated! ROM scale MCP: "
              f"{[f'{self._rom_scale[f*4+1]:.2f}' for f in range(5)]}")
        print(f"[1A-Cal] Human ROM MCP (deg): "
              f"{[f'{np.degrees(human_rom[f*4+1]):.1f}' for f in range(5)]}")

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

        # 0) ROM normalization (if calibrated via 2-pose)
        #    normalized[i] = (ergo[i] - rest[i]) * rom_scale[i]
        #    → rest pose → 0, fist pose → dg5f_range
        if self._is_rom_calibrated:
            ergonomics = ergonomics * self._rom_scale + self._rom_offset

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
