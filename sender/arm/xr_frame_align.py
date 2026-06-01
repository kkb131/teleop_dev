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


# WebXR → robot base_link 좌표축 정렬 (사용자 실측 매핑, R_x(+90°)).
# 손 +x (오른쪽) → robot +x, 손 +y (위) → robot +z, 손 +z (뒤) → robot -y.
# 위치 delta 와 회전 delta 양쪽에 일관 적용 (rotation 은 R @ R_delta @ R.T conjugation).
_R_REMAP = np.array([
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0, -1.0],
    [ 0.0,  1.0,  0.0],
], dtype=np.float64)


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

        # delta translation (scaled) — apply WebXR → robot axis remap
        curr_p = user_pose[:3, 3]
        origin_p = self._origin.user_pose[:3, 3]
        webxr_delta_p = (curr_p - origin_p) * self.scale
        delta_p = _R_REMAP @ webxr_delta_p
        target_pos = self._origin.robot_pos + delta_p

        # delta rotation: R_delta = curr_user_R @ origin_user_R^T (in WebXR frame)
        curr_R = user_pose[:3, :3]
        origin_R = self._origin.user_pose[:3, :3]
        R_delta_webxr = curr_R @ origin_R.T

        # WebXR-frame R_delta → robot-frame R_delta via conjugation
        # (consistent with translation remap, so EE rotates intuitively too).
        R_delta = _R_REMAP @ R_delta_webxr @ _R_REMAP.T

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
