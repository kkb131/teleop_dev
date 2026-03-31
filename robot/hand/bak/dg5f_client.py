#!/usr/bin/env python3
"""Tesollo DG 5F M hand Modbus TCP driver.

Direct communication with the DG5F hand via Modbus TCP protocol.
No ROS2 dependency — uses pymodbus for register read/write.

Register layout based on dg5f_enum.hpp from dg5f_ros2:
  - Holding reg 0:     SYSTEM_STOP_START (1=start, 0=stop)
  - Holding reg 7-26:  MOTOR1~20_TARGET_POSITION
  - Holding reg 27-46: MOTOR1~20_MOTION_TIME
  - Input reg 6-25:    MOTOR1~20_CURRENT_POSITION
  - Input reg 26-45:   MOTOR1~20_CURRENT_CURRENT
  - Input reg 46-65:   MOTOR1~20_CURRENT_VELOCITY

Scaling:
  - Position: register_value * RAD_SCALE (π/1800)
  - Velocity: register_value * VEL_SCALE (π/30)
  - Current:  register_value * CUR_SCALE (0.001 A)

Usage:
    from robot.hand.dg5f_client import DG5FClient

    client = DG5FClient(ip="169.254.186.72")
    client.connect()
    client.start()
    client.set_positions([0.0] * 20)  # open hand
    positions = client.get_positions()
    client.stop()
    client.disconnect()
"""

import math
import time
from typing import Optional

import numpy as np

try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusIOException
    _HAS_PYMODBUS = True
except ImportError:
    ModbusIOException = OSError  # fallback for type hints
    _HAS_PYMODBUS = False


# ─────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────

NUM_MOTORS = 20
RAD_SCALE = math.pi / 1800.0   # register → radians
VEL_SCALE = math.pi / 30.0     # register → rad/s
CUR_SCALE = 0.001               # register → Amps

# Holding register addresses (write)
REG_SYSTEM_STOP_START = 0
REG_TARGET_POS_START = 7        # 7..26  (20 motors)
REG_MOTION_TIME_START = 27      # 27..46 (20 motors)
REG_PGAIN_START = 86            # 86..105
REG_DGAIN_START = 106           # 106..125
REG_IGAIN_START = 126           # 126..145
REG_TARGET_CURRENT_START = 389  # 389..408

# Input register addresses (read)
REG_IS_MOVING = 2
REG_CURRENT_POS_START = 6       # 6..25  (20 motors)
REG_CURRENT_CUR_START = 26      # 26..45 (20 motors)
REG_CURRENT_VEL_START = 46      # 46..65 (20 motors)
REG_TEMPERATURE_START = 66      # 66..85 (20 motors)

# Joint names
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


def _rad_to_reg(rad: float) -> int:
    """Convert radians to Modbus register value (signed int16)."""
    val = int(round(rad / RAD_SCALE))
    # Clamp to int16 range
    return max(-32768, min(32767, val))


def _reg_to_rad(reg: int) -> float:
    """Convert Modbus register value to radians."""
    # Handle signed int16
    if reg > 32767:
        reg -= 65536
    return reg * RAD_SCALE


def _reg_to_current(reg: int) -> float:
    """Convert register value to current in Amps."""
    if reg > 32767:
        reg -= 65536
    return reg * CUR_SCALE


def _reg_to_velocity(reg: int) -> float:
    """Convert register value to velocity in rad/s."""
    if reg > 32767:
        reg -= 65536
    return reg * VEL_SCALE


class DG5FClient:
    """Modbus TCP client for Tesollo DG 5F M hand.

    Parameters
    ----------
    ip : str
        IP address of the DG5F hand (default: 169.254.186.72).
    port : int
        Modbus TCP port (default: 502).
    hand_side : str
        "left" or "right" (affects joint naming only).
    timeout : float
        Connection timeout in seconds.
    """

    def __init__(self, ip: str = "169.254.186.72", port: int = 502,
                 hand_side: str = "right", timeout: float = 3.0,
                 slave_id: int = 1):
        if not _HAS_PYMODBUS:
            raise ImportError(
                "pymodbus is required for DG5F control.\n"
                "Install: pip install 'pymodbus>=3.10,<4'"
            )
        self._ip = ip
        self._port = port
        self._hand_side = hand_side
        self._timeout = timeout
        self._slave_id = slave_id
        self._client: Optional[ModbusTcpClient] = None
        self._connected = False
        self._started = False
        self._joint_names = RIGHT_JOINT_NAMES if hand_side == "right" else LEFT_JOINT_NAMES

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def started(self) -> bool:
        return self._started

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def connect(self):
        """Connect to DG5F hand via Modbus TCP."""
        self._client = ModbusTcpClient(
            host=self._ip,
            port=self._port,
            timeout=self._timeout,
        )
        connect_result = self._client.connect()
        connected_attr = getattr(self._client, 'connected', None)
        if not connect_result and not connected_attr:
            raise ConnectionError(
                f"Failed to connect to DG5F at {self._ip}:{self._port}. "
                f"connect()={connect_result}, connected={connected_attr}. "
                "Check network connection and hand power."
            )
        self._connected = True
        print(f"[DG5F] Connected to {self._ip}:{self._port} ({self._hand_side} hand)")

    def disconnect(self):
        """Disconnect from DG5F hand."""
        if self._started:
            self.stop()
        if self._client is not None:
            self._client.close()
            self._connected = False
            print("[DG5F] Disconnected")

    def start(self):
        """Enable motor control (SYSTEM_START)."""
        self._check_connected()
        try:
            self._client.write_register(REG_SYSTEM_STOP_START, 1, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to start motors: {e}") from e
        self._started = True
        print("[DG5F] System started (motors enabled)")

    def stop(self):
        """Disable motor control (SYSTEM_STOP)."""
        self._check_connected()
        try:
            self._client.write_register(REG_SYSTEM_STOP_START, 0, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to stop motors: {e}") from e
        self._started = False
        print("[DG5F] System stopped (motors disabled)")

    def set_positions(self, angles_rad: list | np.ndarray):
        """Set target positions for all 20 motors.

        Parameters
        ----------
        angles_rad : list or ndarray of 20 floats
            Target joint angles in radians.
        """
        self._check_connected()
        if len(angles_rad) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} angles, got {len(angles_rad)}")

        regs = [_rad_to_reg(float(a)) for a in angles_rad]
        # pymodbus write_registers expects unsigned values
        regs_unsigned = [r & 0xFFFF for r in regs]
        try:
            self._client.write_registers(REG_TARGET_POS_START, regs_unsigned, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to write positions: {e}") from e

    def set_motion_times(self, times_ms: list | np.ndarray):
        """Set motion time for each motor (trajectory duration).

        Parameters
        ----------
        times_ms : list or ndarray of 20 ints
            Motion time per joint in milliseconds.
        """
        self._check_connected()
        if len(times_ms) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} values, got {len(times_ms)}")

        regs = [max(0, min(65535, int(t))) for t in times_ms]
        try:
            self._client.write_registers(REG_MOTION_TIME_START, regs, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to write motion times: {e}") from e

    def get_positions(self) -> np.ndarray:
        """Read current joint positions.

        Returns
        -------
        ndarray[20] of joint angles in radians.
        """
        self._check_connected()
        try:
            result = self._client.read_input_registers(REG_CURRENT_POS_START, count=NUM_MOTORS, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to read positions: {e}") from e
        if result.isError():
            raise RuntimeError(f"Failed to read positions: {result}")
        return np.array([_reg_to_rad(r) for r in result.registers])

    def get_currents(self) -> np.ndarray:
        """Read motor currents.

        Returns
        -------
        ndarray[20] of motor currents in Amps.
        """
        self._check_connected()
        try:
            result = self._client.read_input_registers(REG_CURRENT_CUR_START, count=NUM_MOTORS, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to read currents: {e}") from e
        if result.isError():
            raise RuntimeError(f"Failed to read currents: {result}")
        return np.array([_reg_to_current(r) for r in result.registers])

    def get_velocities(self) -> np.ndarray:
        """Read motor velocities.

        Returns
        -------
        ndarray[20] of motor velocities in rad/s.
        """
        self._check_connected()
        try:
            result = self._client.read_input_registers(REG_CURRENT_VEL_START, count=NUM_MOTORS, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to read velocities: {e}") from e
        if result.isError():
            raise RuntimeError(f"Failed to read velocities: {result}")
        return np.array([_reg_to_velocity(r) for r in result.registers])

    def is_moving(self) -> bool:
        """Check if any motor is currently moving."""
        self._check_connected()
        try:
            result = self._client.read_input_registers(REG_IS_MOVING, count=1, device_id=self._slave_id)
        except ModbusIOException:
            return False
        if result.isError():
            return False
        return result.registers[0] != 0

    def get_temperatures(self) -> np.ndarray:
        """Read motor temperatures.

        Returns
        -------
        ndarray[20] of temperatures (raw register values).
        """
        self._check_connected()
        try:
            result = self._client.read_input_registers(REG_TEMPERATURE_START, count=NUM_MOTORS, device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to read temperatures: {e}") from e
        if result.isError():
            raise RuntimeError(f"Failed to read temperatures: {result}")
        return np.array(result.registers, dtype=np.float32)

    def set_pid_gains(self, p_gains: list, d_gains: list, i_gains: list):
        """Set PID gains for all motors.

        Parameters
        ----------
        p_gains, d_gains, i_gains : list of 20 ints
            PID gain values (register units).
        """
        self._check_connected()
        try:
            self._client.write_registers(REG_PGAIN_START, p_gains[:NUM_MOTORS], device_id=self._slave_id)
            self._client.write_registers(REG_DGAIN_START, d_gains[:NUM_MOTORS], device_id=self._slave_id)
            self._client.write_registers(REG_IGAIN_START, i_gains[:NUM_MOTORS], device_id=self._slave_id)
        except ModbusIOException as e:
            raise RuntimeError(f"Failed to write PID gains: {e}") from e

    def get_status(self) -> dict:
        """Get hand connection and state info."""
        return {
            "ip": self._ip,
            "port": self._port,
            "hand_side": self._hand_side,
            "connected": self._connected,
            "started": self._started,
        }

    def _check_connected(self):
        if not self._connected or self._client is None:
            raise RuntimeError("Not connected. Call connect() first.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()


# ─────────────────────────────────────────────────────────
# Standalone test
# ─────────────────────────────────────────────────────────

def main():
    import argparse

    parser = argparse.ArgumentParser(description="DG5F hand Modbus TCP test")
    parser.add_argument("--ip", default="169.254.186.72",
                        help="DG5F IP address")
    parser.add_argument("--port", type=int, default=502,
                        help="Modbus TCP port")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right"])
    args = parser.parse_args()

    print("=" * 55)
    print("  DG5F Hand — Modbus TCP Test")
    print(f"  Target: {args.ip}:{args.port}")
    print("=" * 55)

    with DG5FClient(ip=args.ip, port=args.port, hand_side=args.hand) as client:
        print(f"\n  Status: {client.get_status()}")

        # Read current positions
        print("\n  Current joint positions (rad):")
        positions = client.get_positions()
        for i, (name, pos) in enumerate(zip(client.joint_names, positions)):
            print(f"    {name}: {pos:+.4f}")

        # Read currents
        print("\n  Motor currents (A):")
        currents = client.get_currents()
        for name, cur in zip(client.joint_names, currents):
            print(f"    {name}: {cur:+.4f}")

        print(f"\n  Moving: {client.is_moving()}")


if __name__ == "__main__":
    main()
