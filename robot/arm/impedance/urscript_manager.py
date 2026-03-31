"""RTDE interface for impedance torque control via custom URScript.

Communication architecture:
  - RTDEReceiveInterface: read robot state (actual_q, actual_qd, etc.)
  - RTDEIOInterface (lower [18-22]): write torques + mode via mixed register types
  - TCP socket (port 30002): upload URScript to UR Secondary Interface
  - Pinocchio: compute Coriolis/centrifugal torques locally

Note: RTDEControlInterface hangs on connection with our UR10e setup.
We bypass it entirely by using the UR Secondary Interface (port 30002)
for script upload and Pinocchio for dynamics computation.

Register allocation (all on lower range [18-22]):
  Double 18..22: tau[0..4]      (joint torques, Nm)
  Int 18:        tau[5] * 1000  (wrist3 millitorque, int32)
  Int 19:        mode           (0=idle, 1=active, -1=stop)
"""

import socket
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import pinocchio as pin

try:
    import rtde_receive
    import rtde_io
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False

from robot.config import URDF_PATH

_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"
_N_JOINTS = 6
_UR_SECONDARY_PORT = 30002

# RTDE register mapping (all lower range [18-22])
_REG_TAU_LOWER = 18       # double registers 18..22 for tau[0..4]
_REG_TAU5_INT = 18        # integer register 18 for tau[5] (millitorque)
_REG_MODE_INT = 19        # integer register 19 for mode
_MILLITORQUE_SCALE = 1000 # tau[5] * 1000 -> int32 (0.001 Nm precision)

# UR10e torque limits [Nm] per joint
TORQUE_LIMITS = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]


class URScriptManager:
    """Manages RTDE communication for impedance torque control.

    PD torque is computed Python-side. Computed torques are written to RTDE
    input registers via RTDEIOInterface. A custom URScript reads these
    registers and calls direct_torque() at 500Hz on the robot controller.

    URScript is uploaded via TCP socket to UR Secondary Interface (port 30002).
    Coriolis/centrifugal torques are computed locally via Pinocchio.
    """

    def __init__(self, robot_ip: str, frequency: float = 500.0):
        if not RTDE_AVAILABLE:
            raise ImportError(
                "ur_rtde is not installed. Install with: pip install ur-rtde"
            )
        self._ip = robot_ip
        self._frequency = frequency
        self._recv: Optional[rtde_receive.RTDEReceiveInterface] = None
        self._io: Optional[rtde_io.RTDEIOInterface] = None
        # Pinocchio model for Coriolis computation
        self._pin_model = pin.buildModelFromUrdf(URDF_PATH)
        self._pin_data = self._pin_model.createData()

    def connect(self):
        """Connect RTDE interfaces and upload torque relay URScript."""
        print(f"[URScriptMgr] Connecting to {self._ip} @ {self._frequency}Hz...")

        # 1. Receive interface for robot state
        self._recv = rtde_receive.RTDEReceiveInterface(self._ip)
        print("[URScriptMgr] RTDEReceiveInterface connected")

        # 2. IO interface for register writes (lower range [18-22] only)
        self._io = rtde_io.RTDEIOInterface(
            self._ip, use_upper_range_registers=False
        )
        print("[URScriptMgr] RTDEIOInterface connected (lower range)")

        # Initialize registers BEFORE upload (critical: prevents race condition)
        # Previous disconnect() may have left mode=-1 in register 19
        for i in range(18, 23):
            self._io.setInputDoubleRegister(i, 0.0)
        self._io.setInputIntRegister(_REG_TAU5_INT, 0)
        self._io.setInputIntRegister(_REG_MODE_INT, 0)
        time.sleep(0.5)  # propagation delay

        # 3. Upload URScript via UR Secondary Interface (port 30002)
        self._upload_script()
        time.sleep(1.0)  # wait for script startup

        # Re-initialize registers after upload (safety: in case upload resets them)
        for i in range(18, 23):
            self._io.setInputDoubleRegister(i, 0.0)
        self._io.setInputIntRegister(_REG_TAU5_INT, 0)
        self._io.setInputIntRegister(_REG_MODE_INT, 0)
        time.sleep(0.1)

    def _upload_script(self):
        """Upload torque relay URScript via TCP socket to port 30002."""
        if not _SCRIPT_PATH.exists():
            raise FileNotFoundError(f"URScript not found: {_SCRIPT_PATH}")

        script_text = _SCRIPT_PATH.read_text()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        try:
            sock.connect((self._ip, _UR_SECONDARY_PORT))
            sock.sendall(script_text.encode("utf-8") + b"\n")
            print(f"[URScriptMgr] URScript uploaded via port {_UR_SECONDARY_PORT}: "
                  f"{_SCRIPT_PATH.name}")
        finally:
            sock.close()

    def send_torque(self, torque: List[float]):
        """Write computed torques to RTDE input registers.

        tau[0..4] -> double registers 18..22
        tau[5]    -> integer register 18 (millitorque: value * 1000)
        """
        for i in range(5):
            self._io.setInputDoubleRegister(_REG_TAU_LOWER + i, torque[i])
        self._io.setInputIntRegister(
            _REG_TAU5_INT, int(torque[5] * _MILLITORQUE_SCALE)
        )

    def set_mode(self, mode: int):
        """Set mode register: 0=idle, 1=active, -1=stop."""
        self._io.setInputIntRegister(_REG_MODE_INT, mode)

    def get_coriolis(self, q: List[float], qd: List[float]) -> List[float]:
        """Compute Coriolis and centrifugal torques via Pinocchio."""
        q_arr = np.array(q)
        qd_arr = np.array(qd)
        pin.computeCoriolisMatrix(self._pin_model, self._pin_data, q_arr, qd_arr)
        return (self._pin_data.C @ qd_arr).tolist()

    def get_joint_positions(self) -> List[float]:
        """Read current joint positions via RTDE."""
        return list(self._recv.getActualQ())

    def get_joint_velocities(self) -> List[float]:
        """Read current joint velocities via RTDE."""
        return list(self._recv.getActualQd())

    def get_tcp_pose(self) -> List[float]:
        """Read current TCP pose [x,y,z,rx,ry,rz]."""
        return list(self._recv.getActualTCPPose())

    def is_connected(self) -> bool:
        """Check if receive interface is still connected."""
        if self._recv is None:
            return False
        return self._recv.isConnected()

    def disconnect(self):
        """Stop URScript and disconnect all interfaces."""
        # Signal URScript to stop via mode register
        if self._io is not None:
            try:
                self.set_mode(-1)
                time.sleep(0.1)
            except Exception:
                pass

        if self._recv is not None:
            try:
                self._recv.disconnect()
            except Exception:
                pass
            self._recv = None

        if self._io is not None:
            try:
                self._io.disconnect()
            except Exception:
                pass
            self._io = None

        print("[URScriptMgr] Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
