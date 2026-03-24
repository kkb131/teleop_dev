"""Pink-based task-level IK controller for teleop servo."""

from typing import Optional, Tuple

import numpy as np
import pinocchio as pin
import pink


class PinkIK:
    """Inverse kinematics using Pink's QP-based task solver.

    Uses FrameTask for end-effector tracking and PostureTask for
    joint centering. Joint limits are handled as QP constraints.
    """

    def __init__(
        self,
        urdf_path: str,
        ee_frame: str = "tool0",
        position_cost: float = 1.0,
        orientation_cost: float = 0.5,
        posture_cost: float = 1e-3,
        damping: float = 1e-12,
    ):
        self._ee_frame = ee_frame
        self._damping = damping

        # Build pinocchio model
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._nq = self._model.nq

        # Pink tasks
        self._ee_task = pink.FrameTask(
            ee_frame,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
        )
        self._posture_task = pink.PostureTask(cost=posture_cost)
        self._tasks = [self._ee_task, self._posture_task]

        # Configuration (initialized later)
        self._config: Optional[pink.Configuration] = None

    @property
    def nq(self) -> int:
        return self._nq

    def initialize(self, q_current: np.ndarray):
        """Set initial configuration from actual robot joints."""
        q = q_current.copy()
        self._config = pink.Configuration(
            self._model, self._data, q, forward_kinematics=True
        )
        # Set posture reference to current
        self._posture_task.set_target(q)

    def solve(
        self, target_pos: np.ndarray, target_quat: np.ndarray, dt: float
    ) -> Optional[np.ndarray]:
        """Compute joint positions for a Cartesian target.

        Args:
            target_pos: [x, y, z] in base frame
            target_quat: [x, y, z, w] quaternion
            dt: time step

        Returns:
            Joint positions array, or None if IK fails.
        """
        if self._config is None:
            return None

        # Build target SE3
        quat_pin = pin.Quaternion(target_quat[3], target_quat[0],
                                  target_quat[1], target_quat[2])
        target_se3 = pin.SE3(quat_pin.matrix(), target_pos)
        self._ee_task.set_target(target_se3)

        try:
            velocity = pink.solve_ik(
                self._config,
                self._tasks,
                dt,
                solver="proxqp",
                damping=self._damping,
            )
            self._config.integrate_inplace(velocity, dt)
            return self._config.q.copy()
        except Exception:
            return None

    def get_ee_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Forward kinematics: return (position[3], quaternion[4] as xyzw).

        Uses a temporary data to avoid disturbing the IK config.
        """
        data = self._model.createData()
        pin.forwardKinematics(self._model, data, q)
        pin.updateFramePlacements(self._model, data)

        frame_id = self._model.getFrameId(self._ee_frame)
        oMf = data.oMf[frame_id]

        pos = oMf.translation.copy()
        quat = pin.Quaternion(oMf.rotation)
        quat_xyzw = np.array([quat.x, quat.y, quat.z, quat.w])

        return pos, quat_xyzw

    def get_ee_rpy(self, q: np.ndarray) -> np.ndarray:
        """Return EE orientation as roll-pitch-yaw [rad]."""
        data = self._model.createData()
        pin.forwardKinematics(self._model, data, q)
        pin.updateFramePlacements(self._model, data)

        frame_id = self._model.getFrameId(self._ee_frame)
        R = data.oMf[frame_id].rotation
        return np.array(pin.rpy.matrixToRpy(R))

    def soft_sync(self, q_actual: np.ndarray, alpha: float = 0.05):
        """Blend internal config toward actual state to prevent drift.

        Each step, nudges the IK's internal q toward the real robot q.
        Prevents unbounded divergence while preserving IK lead-ahead.
        """
        if self._config is None:
            return
        q_blend = (1.0 - alpha) * self._config.q + alpha * q_actual
        self._config = pink.Configuration(
            self._model, self._data, q_blend, forward_kinematics=True
        )

    def sync_configuration(self, q: np.ndarray):
        """Sync internal pink configuration to actual joint state.

        Call this when the robot has moved externally (e.g., after e-stop reset).
        """
        self._config = pink.Configuration(
            self._model, self._data, q.copy(), forward_kinematics=True
        )
