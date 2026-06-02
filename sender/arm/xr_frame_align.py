"""WebXR wrist pose → 로봇 base_link frame 변환 + relative motion 누적.

xr_teleop 의 [run_teleop_ur10e_ws.py:307-358](../../../xr_teleop/scripts/run_teleop_ur10e_ws.py#L307)
의 relative motion 패턴을 teleop_dev sender 측 구조로 이식.

핵심 아이디어 (xr_teleop Week 4 U5+ 검증):
    init pose (robot 의 시작 자세) 와 사용자 손목 시작 위치 가 무관하므로 절대
    매핑은 캘리브레이션 어렵고 위험. 대신:
      1. 'r' (또는 reset) 시점에 robot 의 현재 TCP pose 와 사용자 손목 pose 를
         동시 capture → origin pair
      2. 그 이후로는 사용자 손목 delta 만 robot origin 에 더해서 target 송신
         delta_p = (curr_user - origin_user) * scale
         R_delta = curr_user_R @ origin_user_R.T
         target_pos = origin_robot_pos + R_remap @ delta_p
         target_R   = R_delta @ origin_robot_R
      3. 'c' 키 (immediate recalibrate) 또는 'p' resume 시 origin 재캡처
         → jump 회피

WebXR → robot base_link 좌표축 매핑 (R_remap):
    WebXR (local-floor: +x right, +y up, -z forward) 의 손 변위를 robot
    base_link 의 [x,y,z] 매핑. 사용자 실측 (init pose HOME_JOINTS) 으로
    다음 정렬 도출:
        손 +x (오른쪽)     → robot +x
        손 +y (위)         → robot +z (위)
        손 +z (사용자 뒤)  → robot -y
    즉 robot base_link 의 x 축은 WebXR x 축과 일치하고, WebXR y/z 축이
    robot z/-y 와 swap. 이는 robot frame 의 x 축 기준 +90° 회전:
        R_REMAP = R_x(+90°) =
            [[1,  0,  0],
             [0,  0, -1],
             [0,  1,  0]]
    proper rotation (det = +1). position delta 와 rotation delta 양쪽에
    conjugation 으로 일관 적용.

teleop_dev 의 TeleopSenderBase 와의 통합:
    sender 측은 robot 의 query_pose 핸드셰이크 로 origin_robot_pose 확보.
    BridgePoseStore.right_arm_pose 로 origin_user_pose 확보 (sync 시작 'r' 시).
    매 loop 에서 delta 계산 → virtual_pos / virtual_quat 직접 set
    (sender_base 의 _apply_delta 누적 모델 대신 절대 set).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


# ─────────────────────────────────────────────────────────────────────────
# Translation remap (실측 확정 — 건들지 말 것).
# WebXR → robot base_link 좌표축 정렬 (R_x(+90°)).
# 손 +x (오른쪽) → robot +x, 손 +y (위) → robot +z, 손 +z (뒤) → robot -y.
# ─────────────────────────────────────────────────────────────────────────
_R_REMAP_TRANS = np.array([
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0, -1.0],
    [ 0.0,  1.0,  0.0],
], dtype=np.float64)

# ─────────────────────────────────────────────────────────────────────────
# Rotation remap — 실로봇에서 인터랙티브 튜닝.
#
# 회전은 translation 과 독립적으로 매핑이 달라질 수 있어 별도 매트릭스로 분리.
# 실로봇 'r' 캘리브레이션 후 손목을 한 축씩만 단순 회전시키며 robot EE 의
# 회전축 / 방향을 관찰 → 의도와 다르면 아래 표의 해당 column 만 수정.
#
# 매트릭스 의미:
#   각 column = WebXR 한 회전축이 robot frame 에서 가리키는 단위 벡터.
#     col 0 (WebXR x 회전 = 손바닥 roll)    → robot frame 의 어느 축?
#     col 1 (WebXR y 회전 = 손가락 좌우 yaw) → robot frame 의 어느 축?
#     col 2 (WebXR z 회전 = 손목 위/아래 pitch) → robot frame 의 어느 축?
#   값은 robot ±x/y/z 단위 벡터 6 종류 중 하나:
#     [+1,0,0]=+x  [-1,0,0]=-x  [0,+1,0]=+y  [0,-1,0]=-y  [0,0,+1]=+z  [0,0,-1]=-z
#
# 진단 절차 (각 회전 단독 테스트):
#   사용자 손목 동작           기본값 거동(현재 매트릭스)   의도와 다르면
#   ─────────────────────────  ──────────────────────────  ───────────────────
#   손바닥 roll (WebXR x 회전)  robot +x 축 회전           col 0 = [robot 의도축 부호단위]
#   손가락 yaw (WebXR y 회전)   robot +z 축 회전           col 1 = [robot 의도축 부호단위]
#   손목 pitch (WebXR z 회전)   robot -y 축 회전           col 2 = [robot 의도축 부호단위]
#
# 조치 패턴:
#   (a) 회전축은 OK, 방향만 반대  → 해당 column 전체 부호 반전
#       예: col 1 = [0, 0, 1] → [0, 0, -1]
#   (b) 회전축 자체가 잘못        → 해당 column 을 다른 축의 단위 벡터로
#       예: col 0 = [1, 0, 0] → [0, 0, 1]  (WebXR x 회전이 robot z 회전 되도록)
#   (c) 두 column 이 같은 축에 매핑됐다면 → invalid rotation. 다른 column 도 같이 swap.
#
# 사용자 보고 패턴 → 조치 예시:
#   "손목을 위로 들었더니 (pitch +, WebXR z 회전) robot 이 왼쪽을 바라봄 (yaw, robot z 회전)"
#     → col 2 가 robot z 축 [0,0,1] 또는 [0,0,-1] 로 가 있음.
#     → 의도가 'robot pitch (-y 축 회전)' 였다면 col 2 = [0, -1, 0]. 현재 [0,-1,0] 그대로면 부호 반전 [0,1,0].
#   "손바닥 roll 시 robot 이 반대 방향으로 roll"
#     → col 0 (현재 [1,0,0]) 의 부호만 반전 → [-1, 0, 0].
#
# Valid rotation matrix 제약 (변경 후 매트릭스가 깨지지 않게):
#   • 세 column 이 robot ±x/y/z 의 서로 다른 축 (nonzero entry 위치가 모두 달라야)
#   • 각 column 은 정확히 ±1 entry 하나만 (단위 벡터)
#   • det == +1 (proper rotation) 또는 -1 (reflection — 작동은 하지만 일관성 ↓)
#   • det 확인은 sender 시작 로그에서 자동 출력 (아래 _check_remap_rot 참조).
#
# 기본값 = _R_REMAP_TRANS (translation 매핑과 동일 시작점, R_x(+90°)):
# ─────────────────────────────────────────────────────────────────────────
_R_REMAP_ROT = np.array([
    #  col 0           col 1           col 2
    #  (WebXR x roll)  (WebXR y yaw)   (WebXR z pitch)
    [   1.0,            0.0,            0.0],   # row → robot x
    [   0.0,            0.0,           -1.0],   # row → robot y
    [   0.0,            1.0,            0.0],   # row → robot z
], dtype=np.float64)

# ── 회전 방향 부호 (axis 매핑과 독립적으로 조정) ──────────────────────────
#
# WHY 별도 매트릭스: _R_REMAP_ROT 의 column 한 개만 부호 반전하면 det=-1
# (reflection). conjugation R · R_delta(axis,θ) · R^T 에서 reflection 은
# axis 와 angle 부호를 둘 다 반전 → R(axis,+θ) = R(-axis,-θ) 라서 결과 동일.
# 즉 단순 column 부호 반전만으로는 "회전 방향만 반대로" 효과가 절대 안 남.
#
# 해결: R_delta 를 quaternion (w, x, y, z) 으로 풀고 vector 부분의 i 성분에
# _ROT_SIGN[i] 를 곱한 뒤 다시 rotation matrix 로 변환. 이는 axis-angle 의
# i 성분만 부호 반전 → 해당 WebXR 축의 회전 성분만 단독 부호 반전 (다른
# 축의 회전 성분은 영향 없음).
#
# 사용법:
#   - 각 entry (1, 1, 1) 은 WebXR 축 회전 (x roll / y yaw / z pitch) 의 방향.
#   - "손목 X 축 회전 시 robot 도 같은 axis 인데 방향만 반대" → 해당 entry +1 → -1.
#   - 예: WebXR x roll (손바닥 회전) 방향만 반대로 보이면 _ROT_SIGN[0] = -1.
#   - 모두 +1 이면 효과 없음 (현재 기본).
#
# 진단 절차:
#   1. _R_REMAP_ROT 으로 회전축 매핑부터 맞추기 (어느 WebXR 축이 어느 robot 축).
#   2. 축은 맞고 방향만 반대인 케이스가 남으면 _ROT_SIGN 의 해당 entry 부호 반전.
#   3. 두 단계 조합으로 모든 회전 거동 fine tune 가능.
# ─────────────────────────────────────────────────────────────────────────
_ROT_SIGN = np.array([1.0, 1.0, 1.0], dtype=np.float64)
#                     ↑    ↑    ↑
#                     x    y    z   ← WebXR axis (roll / yaw / pitch)


def _apply_rot_sign(R_webxr: np.ndarray) -> np.ndarray:
    """WebXR R_delta 의 각 axis 회전 성분 부호를 _ROT_SIGN 에 따라 반전.

    axis-angle 의 i 성분에 _ROT_SIGN[i] 를 곱하는 것과 동치 — quaternion
    (w, x, y, z) 의 vector 성분을 element-wise 로 _ROT_SIGN 과 곱하면 정확히
    axis 의 그 성분만 부호 반전.
    """
    if np.allclose(_ROT_SIGN, 1.0):
        return R_webxr
    q = _rotmat_to_quat_wxyz(R_webxr)
    q[1:] = q[1:] * _ROT_SIGN
    return _quat_wxyz_to_rotmat(q)


def _check_remap_rot() -> None:
    """Sender 시작 시 1회 호출 — _R_REMAP_ROT / _ROT_SIGN 진단 print."""
    det = float(np.linalg.det(_R_REMAP_ROT))
    print(f"[xr_frame_align] _R_REMAP_ROT det = {det:+.3f} "
          f"({'proper rotation' if det > 0.5 else 'reflection' if det < -0.5 else 'INVALID — fix columns!'})")
    print(f"  col 0 (WebXR x roll) → {_R_REMAP_ROT[:, 0].tolist()}")
    print(f"  col 1 (WebXR y yaw)  → {_R_REMAP_ROT[:, 1].tolist()}")
    print(f"  col 2 (WebXR z pitch)→ {_R_REMAP_ROT[:, 2].tolist()}")
    print(f"  _ROT_SIGN = {_ROT_SIGN.tolist()}  (WebXR x roll / y yaw / z pitch 회전 방향)")




@dataclass
class FrameOrigin:
    """Calibration origin — 'r' (또는 'c' / 'p' resume) 시점에 캡처된 짝."""
    user_pose: np.ndarray   # (4, 4) WebXR local-floor 좌표계 의 wrist
    robot_pos: np.ndarray   # (3,)   robot base_link 좌표계 의 TCP position
    robot_quat: np.ndarray  # (4,)   wxyz, robot base_link 의 TCP orientation


class XRRelativeFrameAligner:
    """WebXR wrist pose → robot base_link target 매핑 (relative motion).

    동작:
        1. `calibrate(user_pose, robot_pos, robot_quat)` — origin 캡처
        2. `apply(user_pose, scale)` — 현재 사용자 wrist → target pos / quat 반환

    캘리브레이션 전에 apply() 호출하면 robot origin 그대로 반환 (안전 hold).
    """

    def __init__(self, scale: float = 1.0):
        self.scale = scale
        self._origin: Optional[FrameOrigin] = None
        self._last_target_pos: Optional[np.ndarray] = None
        self._last_target_quat: Optional[np.ndarray] = None
        _check_remap_rot()

    def calibrate(
        self,
        user_pose: np.ndarray,
        robot_pos: np.ndarray,
        robot_quat: np.ndarray,
    ) -> None:
        """origin 캡처. user_pose / robot_pos 가 invalid 면 calibrate 실패 → False 반환 대신 무시.

        Parameters
        ----------
        user_pose : (4, 4) — BridgePoseStore.right_arm_pose. zeros / NaN 이면 skip.
        robot_pos : (3,)   — robot 의 현재 TCP position (base_link frame).
        robot_quat : (4,)  — wxyz.
        """
        if not _is_valid_pose(user_pose):
            print("[XRRelativeFrameAligner] WARN: user_pose invalid — skip calibration")
            return
        self._origin = FrameOrigin(
            user_pose=user_pose.copy(),
            robot_pos=np.asarray(robot_pos, dtype=np.float64).copy(),
            robot_quat=np.asarray(robot_quat, dtype=np.float64).copy(),
        )
        # 캘리 직후 target = origin (jump 회피)
        self._last_target_pos = self._origin.robot_pos.copy()
        self._last_target_quat = self._origin.robot_quat.copy()
        print(
            f"[XRRelativeFrameAligner] calibrated.\n"
            f"  user origin p = {self._origin.user_pose[:3, 3]}\n"
            f"  robot origin p = {self._origin.robot_pos}\n"
            f"  scale = {self.scale}"
        )

    def apply(self, user_pose: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """현재 사용자 wrist → robot target (pos, quat_wxyz).

        Parameters
        ----------
        user_pose : (4, 4) — BridgePoseStore.right_arm_pose.

        Returns
        -------
        target_pos : (3,) base_link 좌표계
        target_quat : (4,) wxyz

        Notes
        -----
        - calibrate 전: last_target (robot origin) 또는 zeros 반환 (안전).
        - user_pose invalid: last_target 유지 (jump 회피).
        - 평행이동: target_p = origin_robot_p + R_remap @ (curr_user_p - origin_user_p) * scale
        - 회전: target_R = (R_remap @ R_delta @ R_remap.T) @ origin_robot_R
        """
        if self._origin is None:
            if self._last_target_pos is None:
                return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
            return self._last_target_pos.copy(), self._last_target_quat.copy()

        if not _is_valid_pose(user_pose):
            return self._last_target_pos.copy(), self._last_target_quat.copy()

        # delta translation (scaled) — apply WebXR → robot axis remap (translation 전용 매트릭스)
        curr_p = user_pose[:3, 3]
        origin_p = self._origin.user_pose[:3, 3]
        webxr_delta_p = (curr_p - origin_p) * self.scale
        delta_p = _R_REMAP_TRANS @ webxr_delta_p
        target_pos = self._origin.robot_pos + delta_p

        # delta rotation: R_delta = curr_user_R @ origin_user_R^T (in WebXR frame)
        curr_R = user_pose[:3, :3]
        origin_R = self._origin.user_pose[:3, :3]
        R_delta_webxr = curr_R @ origin_R.T

        # axis-angle level sign flip — _ROT_SIGN[i] 가 WebXR i 축 회전 성분 부호.
        R_delta_webxr = _apply_rot_sign(R_delta_webxr)

        # WebXR-frame R_delta → robot-frame R_delta via conjugation.
        # 회전 매핑은 별도 _R_REMAP_ROT (translation 과 독립적 fine tune 가능).
        R_delta = _R_REMAP_ROT @ R_delta_webxr @ _R_REMAP_ROT.T

        # target_R = R_delta @ origin_robot_R
        origin_robot_R = _quat_wxyz_to_rotmat(self._origin.robot_quat)
        target_R = R_delta @ origin_robot_R
        target_quat = _rotmat_to_quat_wxyz(target_R)

        self._last_target_pos = target_pos.copy()
        self._last_target_quat = target_quat.copy()
        return target_pos, target_quat

    @property
    def calibrated(self) -> bool:
        return self._origin is not None

    def reset(self) -> None:
        """origin clear. next apply() 가 last_target 반환 (또는 zeros if 첫 호출)."""
        self._origin = None


# ── helpers ─────────────────────────────────────────────────────────────

def _is_valid_pose(pose: np.ndarray) -> bool:
    if pose is None or pose.shape != (4, 4):
        return False
    if not np.isfinite(pose).all():
        return False
    if np.allclose(pose, 0):
        return False
    if abs(pose[3, 3] - 1.0) > 1e-6:
        return False
    return True


def _quat_wxyz_to_rotmat(q: np.ndarray) -> np.ndarray:
    """wxyz → 3×3 rotation matrix."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def _rotmat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → wxyz quaternion (branch-free Shepperd)."""
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float64)
