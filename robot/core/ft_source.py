"""Force/Torque sensor source abstraction.

Provides a common interface for reading F/T data from different backends:
  - RTDEFTSource: real UR10e via ur_rtde (with bias correction)
  - NullFTSource: returns zeros (sim mode or no sensor)
"""

from typing import Protocol

import numpy as np


class FTSource(Protocol):
    """Protocol for F/T sensor data sources."""

    def get_wrench(self) -> np.ndarray:
        """Return [fx, fy, fz, tx, ty, tz] in sensor (tool) frame."""
        ...

    def zero_sensor(self) -> None:
        """Bias-correct by subtracting the current reading."""
        ...


class RTDEFTSource:
    """F/T readings from UR10e via ur_rtde with bias correction."""

    def __init__(self, backend):
        self._backend = backend
        self._bias = np.zeros(6)

    def get_wrench(self) -> np.ndarray:
        raw = np.array(self._backend.get_tcp_force())
        return raw - self._bias

    def zero_sensor(self) -> None:
        self._bias = np.array(self._backend.get_tcp_force())


class BaseFrameFTSource:
    """Wraps an FTSource and transforms wrench from TCP frame to base frame.

    UR10e getActualTCPForce() returns wrench in the TCP frame. With the
    EE connector facing down, TCP X,Y are inverted relative to base X,Y.
    This wrapper negates X,Y components to convert to base frame.

    현재 UR10e에서는 "negate X,Y" 방식(#4)이 실험적으로 확인됨.
    로봇/마운트/IK 변경 시 아래 대안을 참고하여 교체할 것.
    (test_wrench_frame.py --servo 로 각 변환의 실시간 동작 검증 가능)

    --- 변환 후보 (test_wrench_frame.py 기준) ---

    #1 raw: 변환 없음 (센서 프레임 == 베이스 프레임인 경우)
        return w

    #2 R_ur @ w: UR TCP pose rotvec으로 회전 (센서→UR 베이스)
        R = rotvec_to_matrix(rotvec)   # rotvec from backend.get_tcp_pose()[3:6]
        f, t = R @ w[:3], R @ w[3:]
        return np.concatenate([f, t])

    #3 R_ur.T @ w: 역회전 (UR 베이스→센서)
        R = rotvec_to_matrix(rotvec)
        f, t = R.T @ w[:3], R.T @ w[3:]
        return np.concatenate([f, t])

    #4 negate X,Y: X,Y 부호 반전 (현재 사용 중) ← UR10e EE-down 확인됨
        return np.array([-w[0], -w[1], w[2], -w[3], -w[4], w[5]])

    #5 R_ur @ w + negate X,Y: 회전 후 X,Y 반전
        R = rotvec_to_matrix(rotvec)
        f, t = R @ w[:3], R @ w[3:]
        f, t = np.array([-f[0], -f[1], f[2]]), np.array([-t[0], -t[1], t[2]])
        return np.concatenate([f, t])

    #6 R_fk @ w: Pinocchio FK 회전행렬 사용 (센서→base_link)
        R_fk = quat_to_matrix(ee_quat)  # from PinkIK.get_ee_pose()
        f, t = R_fk @ w[:3], R_fk @ w[3:]
        return np.concatenate([f, t])
    """

    def __init__(self, source):
        self._source = source

    def get_wrench(self) -> np.ndarray:
        w = self._source.get_wrench()
        # #4: negate X,Y — UR10e EE-down 마운트에서 실험 확인됨
        return np.array([-w[0], -w[1], w[2], -w[3], -w[4], w[5]])

    def zero_sensor(self) -> None:
        self._source.zero_sensor()


class NullFTSource:
    """Always returns zero wrench. Used in sim mode or when no sensor."""

    def get_wrench(self) -> np.ndarray:
        return np.zeros(6)

    def zero_sensor(self) -> None:
        pass


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Convert rotation vector (axis-angle) to 3x3 rotation matrix (Rodrigues)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-10:
        return np.eye(3)
    axis = rotvec / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
