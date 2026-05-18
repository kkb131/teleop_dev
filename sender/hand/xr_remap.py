"""WebXR 25-joint → wrist-local + palm-aligned MANO-frame transform.

xr_teleop 의 webxr_to_wrist_local_mano 패턴 그대로 (origin commit
`02199d7+` U5++) — retarget_dev/sensing/core/mano_transform.py 의
apply_mano_transform 을 WebXR 25-joint 인덱스로 적응시킨 변형. 이 변환을
거치면 사용자 손목 회전 시 손가락 자세가 동일하면 retargeter 출력이 동일
하게 유지 (회전 robust).

WebXR 25-joint vs MANO 21-joint
--------------------------------
WebXR 25 = wrist + 5 finger × (metacarpal + proximal + intermediate + distal + tip)
    인덱스: wrist=0, thumb=1..4, index=5..9, middle=10..14, ring=15..19, pinky=20..24
    각 finger 5 keypoint (metacarpal 포함).
MANO 21 = wrist + 5 finger × 4 (no separate metacarpal for non-thumb)
    엄지만 4점 (CMC, MCP, IP, tip), 나머지 4점 (MCP, PIP, DIP, tip).

palm-plane SVD 의 3 점 인덱스만 다름:
    MANO 21: [0, 5, 9]   = wrist, index_MCP, middle_MCP
    WebXR 25: [0, 5, 10] = wrist, index-metacarpal, middle-metacarpal
metacarpal 이 palm 안쪽이라 palm-plane fit 정확도 더 좋음.

operator → MANO rotation matrix 는 sender/hand/core/mano_transform.py 의
MEDIAPIPE / MANUS RIGHT 그대로 — WebXR HandLandmarker 가 image-derived
MediaPipe 와 같은 chirality 라고 가정 (mediapipe default). visual 검증 후
fist↔spread inversion 보이면 convention="manus" 로 toggle.

Cross-reference:
    xr_teleop/scripts/dg5f_controller.py 의 webxr_to_wrist_local_mano +
    _estimate_wrist_frame_webxr.
    sender/hand/core/mano_transform.py — MEDIAPIPE_/MANUS_OPERATOR2MANO 매트릭스.
"""

from __future__ import annotations

import numpy as np

from sender.hand.core.mano_transform import (
    MEDIAPIPE_OPERATOR2MANO_LEFT,
    MEDIAPIPE_OPERATOR2MANO_RIGHT,
    MANUS_OPERATOR2MANO_LEFT,
    MANUS_OPERATOR2MANO_RIGHT,
)

# WebXR 25 인덱스 — palm-plane fit 3 점
_WEBXR_PALM_PLANE_IDX = [0, 5, 10]   # wrist, index-metacarpal, middle-metacarpal


def estimate_wrist_frame_webxr(kp_25: np.ndarray) -> np.ndarray:
    """WebXR 25-joint 의 wrist frame 추정 (SVD palm-plane fit).

    Parameters
    ----------
    kp_25 : (25, 3) np.ndarray
        WebXR 25-joint xyz. wrist 가 origin 일 필요 없음 (centered 안에서 처리).

    Returns
    -------
    (3, 3) np.ndarray
        columns = x/y/z basis vectors of wrist frame.
        - x: palm → middle-metacarpal 방향
        - y: palm plane normal (pinky→index 양수 방향 disambiguation)
        - z: x × normal (≈ pinky → index)
    """
    if kp_25.shape != (25, 3):
        raise ValueError(f"kp_25 must be (25, 3), got {kp_25.shape}")

    points = kp_25[_WEBXR_PALM_PLANE_IDX, :]
    # x direction: wrist → middle_metacarpal 의 반대 (palm 안쪽 방향)
    x_vector = points[0] - points[2]
    # SVD palm-plane normal
    centered = points - points.mean(axis=0, keepdims=True)
    _u, _s, v = np.linalg.svd(centered)
    normal = v[2, :]
    # Gram-Schmidt: x ⊥ normal
    x = x_vector - np.dot(x_vector, normal) * normal
    x = x / (np.linalg.norm(x) + 1e-10)
    z = np.cross(x, normal)
    # disambiguation: z 는 pinky → index 방향
    if np.dot(z, (points[1] - points[2])) < 0:
        normal = -normal
        z = -z
    return np.stack([x, normal, z], axis=1).astype(np.float64)


def webxr_to_wrist_local_mano(
    kp_25: np.ndarray,
    hand_side: str = "right",
    convention: str = "mediapipe",
) -> np.ndarray:
    """WebXR 25-joint world frame → wrist-local + palm-aligned MANO frame.

    Pipeline (retarget_dev/sensing/core/mano_transform.py:apply_mano_transform 그대로):
        1. wrist-center: pos - pos[0]
        2. SVD palm-plane fit → wrist rotation
        3. kp @ wrist_rot @ operator2mano

    Parameters
    ----------
    kp_25 : (25, 3) np.ndarray
        WebXR 25-joint xyz in world frame (BridgePoseStore 의 hand_positions).
    hand_side : "right" or "left"
    convention : "mediapipe" or "manus"
        WebXR HandLandmarker 가 image-derived MediaPipe 와 같은 chirality 라
        가정 (default mediapipe). visual 검증 후 fist↔spread inversion 보이면
        manus 로 toggle (row 1 sign flip).

    Returns
    -------
    (25, 3) np.ndarray
        wrist-centered + MANO-aligned. retargeter 가 wrist orientation 무관하게
        동일 손가락 자세에 대해 동일 출력을 내도록 함.
    """
    centered = kp_25 - kp_25[0]
    wrist_rot = estimate_wrist_frame_webxr(centered)
    is_left = hand_side.lower() == "left"
    if convention == "manus":
        op2mano = MANUS_OPERATOR2MANO_LEFT if is_left else MANUS_OPERATOR2MANO_RIGHT
    elif convention == "mediapipe":
        op2mano = (
            MEDIAPIPE_OPERATOR2MANO_LEFT if is_left else MEDIAPIPE_OPERATOR2MANO_RIGHT
        )
    else:
        raise ValueError(
            f"unknown convention: {convention!r} (use 'mediapipe' or 'manus')"
        )
    return centered @ wrist_rot @ op2mano.astype(np.float64)


def is_kp25_valid(kp_25: np.ndarray) -> bool:
    """BridgePoseStore 의 hand_positions 가 채워졌는지 확인.

    초기값 zeros 또는 NaN 이면 invalid. 손이 시야 안에 들어오기 전 retargeter
    호출하면 잘못된 결과 — sender 가 이 check 후 valid 일 때만 retarget.
    """
    if kp_25 is None:
        return False
    if kp_25.shape != (25, 3):
        return False
    if not np.isfinite(kp_25).all():
        return False
    if np.allclose(kp_25, 0):
        return False
    return True
