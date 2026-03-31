"""UR10e communication via ur_rtde (RTDE + servoJ) — RTDEBackend."""

from typing import List, Optional

try:
    import rtde_control
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False

from robot.config import RTDE_FREQUENCY, SERVOJ_DT, SERVOJ_GAIN, SERVOJ_LOOKAHEAD
from robot.core.robot_backend import RobotBackend


class RTDEBackend(RobotBackend):
    """UR10e robot backend using ur_rtde library."""

    def __init__(self, robot_ip: str, frequency: int = RTDE_FREQUENCY):
        if not RTDE_AVAILABLE:
            raise ImportError(
                "ur_rtde is not installed. Install with: pip install ur-rtde"
            )
        self._ip = robot_ip
        self._frequency = frequency
        self._recv: Optional[rtde_receive.RTDEReceiveInterface] = None
        self._ctrl: Optional[rtde_control.RTDEControlInterface] = None

    def connect(self):
        print(f"[RTDEBackend] Connecting to {self._ip}...")
        self._recv = rtde_receive.RTDEReceiveInterface(self._ip)
        self._ctrl = rtde_control.RTDEControlInterface(self._ip)
        print(f"[RTDEBackend] Connected (RTDE {self._frequency}Hz)")

    def disconnect(self):
        if self._ctrl is not None:
            self._ctrl.stopScript()
            self._ctrl.disconnect()
            self._ctrl = None
        if self._recv is not None:
            self._recv.disconnect()
            self._recv = None
        print("[RTDEBackend] Disconnected")

    def get_joint_positions(self) -> List[float]:
        return list(self._recv.getActualQ())

    def get_joint_velocities(self) -> List[float]:
        return list(self._recv.getActualQd())

    def get_tcp_pose(self) -> List[float]:
        """Read current TCP pose [x,y,z,rx,ry,rz] (meters, axis-angle)."""
        return list(self._recv.getActualTCPPose())

    def get_tcp_force(self) -> List[float]:
        """Read current TCP force/torque [fx,fy,fz,tx,ty,tz] (N, Nm)."""
        return list(self._recv.getActualTCPForce())

    def send_joint_command(self, positions: List[float]):
        self._ctrl.servoJ(positions, 0, 0, SERVOJ_DT, SERVOJ_LOOKAHEAD, SERVOJ_GAIN)

    def on_trajectory_done(self):
        self._ctrl.servoStop()

    def stop_script(self):
        """Emergency stop — terminates any running URScript."""
        self._ctrl.stopScript()

    def emergency_stop(self):
        """E-Stop: immediately stop all robot motion."""
        self._ctrl.stopScript()

    def speed_stop(self, deceleration: float = 2.0):
        """Decelerate to stop smoothly."""
        self._ctrl.servoStop(deceleration)
