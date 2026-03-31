"""Wrench frame transformation diagnostic tester.

Two modes:
  1. Sensor-only (default): displays 6 transformation candidates side-by-side
  2. Servo (--servo): actually commands the robot via admittance + IK + servoJ,
     switch between transformations with keys 1-6

Usage:
    cd /workspaces/tamp_ws/src/tamp_dev
    # Sensor reading only
    python3 -m robot.arm.admittance.test_wrench_frame --robot-ip 192.168.0.2
    # End-to-end servo test
    python3 -m robot.arm.admittance.test_wrench_frame --robot-ip 192.168.0.2 --servo
"""

import argparse
import sys
import time
import select
import termios
import tty

import numpy as np
import pinocchio as pin

from robot.config import URDF_PATH, RTDE_FREQUENCY
from robot.core.robot_backend import create_backend
from robot.core.ft_source import rotvec_to_matrix
from robot.core.pink_ik import PinkIK
from robot.core.compliant_control import (
    AdmittanceController,
    COMPLIANCE_PRESETS,
)


def get_key_nonblocking():
    """Non-blocking single key read from stdin."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None


def apply_rotation_delta(
    quat_xyzw: np.ndarray, angular_vel: np.ndarray, dt: float
) -> np.ndarray:
    """Apply angular velocity delta to a quaternion."""
    angle = np.linalg.norm(angular_vel) * dt
    if angle < 1e-10:
        return quat_xyzw.copy()
    axis = angular_vel / (np.linalg.norm(angular_vel) + 1e-15)
    aa = pin.AngleAxis(angle, axis)
    dR = aa.matrix()
    q_pin = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    R_new = dR @ q_pin.matrix()
    q_new = pin.Quaternion(R_new)
    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])


# --- Transformation candidates ---

TRANSFORM_NAMES = {
    "1": "raw (no rotation)",
    "2": "R_ur @ wrench (UR TCP pose rotvec)",
    "3": "R_ur.T @ wrench",
    "4": "negate X,Y only",
    "5": "R_ur @ wrench + negXY",
    "6": "R_fk @ wrench (Pinocchio FK)",
}


def quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    """Convert quaternion (x,y,z,w) to 3x3 rotation matrix via Pinocchio."""
    q = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    return q.matrix()


def apply_transform(key: str, wrench: np.ndarray, R: np.ndarray,
                    R_fk: np.ndarray = None) -> np.ndarray:
    """Apply the selected transformation to a 6D wrench.

    Args:
        key: Transform key "1"-"6".
        wrench: Raw 6D wrench in TCP frame.
        R: Rotation from UR TCP pose rotvec (UR Base frame).
        R_fk: Rotation from Pinocchio FK (base_link frame). Required for key "6".
    """
    f, t = wrench[:3], wrench[3:]
    if key == "1":
        fb, tb = f, t
    elif key == "2":
        fb, tb = R @ f, R @ t
    elif key == "3":
        fb, tb = R.T @ f, R.T @ t
    elif key == "4":
        fb = np.array([-f[0], -f[1], f[2]])
        tb = np.array([-t[0], -t[1], t[2]])
    elif key == "5":
        rf = R @ f
        rt = R @ t
        fb = np.array([-rf[0], -rf[1], rf[2]])
        tb = np.array([-rt[0], -rt[1], rt[2]])
    elif key == "6" and R_fk is not None:
        fb, tb = R_fk @ f, R_fk @ t
    else:
        fb, tb = f, t
    return np.concatenate([fb, tb])


# --- Sensor-only mode ---

def run_sensor_mode(backend):
    """Display all 6 transformation candidates side-by-side."""
    bias = np.zeros(6)

    # Pinocchio FK for candidate #6 (base_link frame rotation)
    ik = PinkIK(URDF_PATH)
    q_init = np.array(backend.get_joint_positions())
    ik.initialize(q_init)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = get_key_nonblocking()
            if key == "q":
                break
            elif key == "z":
                bias = np.array(backend.get_tcp_force())
                print("\033[2K\r[Diag] Sensor zeroed!")
                time.sleep(0.3)

            raw_wrench = np.array(backend.get_tcp_force()) - bias
            tcp_pose = backend.get_tcp_pose()
            rotvec = np.array(tcp_pose[3:6])
            R = rotvec_to_matrix(rotvec)  # UR Base frame rotation
            angle_deg = np.degrees(np.linalg.norm(rotvec))

            # Pinocchio FK rotation (base_link frame)
            q_now = np.array(backend.get_joint_positions())
            _, ee_quat = ik.get_ee_pose(q_now)
            R_fk = quat_to_matrix(ee_quat)

            f = raw_wrench[:3]
            t = raw_wrench[3:]

            candidates = {
                "1  raw     ": f,
                "2  R_ur@f  ": R @ f,
                "3  R_ur.T@f": R.T @ f,
                "4  neg(XY) ": np.array([-f[0], -f[1], f[2]]),
                "5  Rur+nXY ": (lambda v: np.array([-v[0], -v[1], v[2]]))(R @ f),
                "6  R_fk@f  ": R_fk @ f,
            }

            lines = ["\033[2J\033[H"]
            lines.append("=== Wrench Frame Diagnostic (SENSOR) ===")
            lines.append(
                f"TCP pose: [{tcp_pose[0]:+.3f}, {tcp_pose[1]:+.3f}, {tcp_pose[2]:+.3f}"
                f" | {rotvec[0]:+.3f}, {rotvec[1]:+.3f}, {rotvec[2]:+.3f}]"
            )
            lines.append(f"TCP rotvec angle: {angle_deg:.1f} deg")
            lines.append("")
            lines.append("Push the robot and check which row matches expected direction.")
            lines.append("Base frame: X=right, Y=forward(away from base), Z=up")
            lines.append("[z] zero sensor  [q] quit")
            lines.append("")
            lines.append(f"{'':13s} {'Force X':>9s} {'Force Y':>9s} {'Force Z':>9s}")
            lines.append(f"{'':13s} {'-------':>9s} {'-------':>9s} {'-------':>9s}")
            for name, fv in candidates.items():
                lines.append(f"{name}  {fv[0]:+9.2f} {fv[1]:+9.2f} {fv[2]:+9.2f}")

            lines.append("")
            lines.append(f"{'':13s} {'Torq X':>9s} {'Torq Y':>9s} {'Torq Z':>9s}")
            lines.append(f"{'':13s} {'------':>9s} {'------':>9s} {'------':>9s}")
            t_candidates = {
                "1  raw     ": t,
                "2  R_ur@t  ": R @ t,
                "3  R_ur.T@t": R.T @ t,
                "4  neg(XY) ": np.array([-t[0], -t[1], t[2]]),
                "5  Rur+nXY ": (lambda v: np.array([-v[0], -v[1], v[2]]))(R @ t),
                "6  R_fk@t  ": R_fk @ t,
            }
            for name, tv in t_candidates.items():
                lines.append(f"{name}  {tv[0]:+9.3f} {tv[1]:+9.3f} {tv[2]:+9.3f}")

            sys.stdout.write("\n".join(lines))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# --- Servo mode ---

def run_servo_mode(backend):
    """End-to-end test: F/T → transform → admittance → IK → servoJ."""
    dt = 1.0 / RTDE_FREQUENCY

    # Wait for valid joint state (like main.py)
    print("[Diag] Waiting for valid joint state...")
    for _ in range(50):
        q = backend.get_joint_positions()
        if any(v != 0.0 for v in q):
            break
        time.sleep(0.1)
    else:
        print("[Diag] ERROR: No valid joint state received!")
        return

    # Initialize IK (initialize sets PostureTask target, required for Pink QP)
    ik = PinkIK(URDF_PATH)
    q_current = np.array(backend.get_joint_positions())
    ik.initialize(q_current)
    home_pos, home_quat = ik.get_ee_pose(q_current)

    # Admittance controller (SOFT preset for gentle testing)
    controller = AdmittanceController(
        params=COMPLIANCE_PRESETS["SOFT"],
        max_disp_trans=0.05,
        max_disp_rot=0.15,
        force_deadzone=np.array([3.0, 3.0, 3.0, 0.3, 0.3, 0.3]),
    )

    bias = np.zeros(6)
    active_transform = "4"  # Start with negate X,Y (confirmed correct)

    print("[Diag] Servo mode. SOFT preset. Keys: 1-6 switch, z zero, q quit.")

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            t_start = time.perf_counter()

            # Key handling
            key = get_key_nonblocking()
            if key == "q":
                break
            elif key == "z":
                bias = np.array(backend.get_tcp_force())
                controller.reset()
                q_current = np.array(backend.get_joint_positions())
                ik.initialize(q_current)
                home_pos, home_quat = ik.get_ee_pose(q_current)
            elif key in TRANSFORM_NAMES:
                active_transform = key
                controller.reset()
                q_current = np.array(backend.get_joint_positions())
                ik.initialize(q_current)
                home_pos, home_quat = ik.get_ee_pose(q_current)

            # Read sensor + TCP pose
            raw_wrench = np.array(backend.get_tcp_force()) - bias
            tcp_pose = backend.get_tcp_pose()
            rotvec = np.array(tcp_pose[3:6])
            R = rotvec_to_matrix(rotvec)  # UR Base frame rotation

            # Pinocchio FK rotation (base_link frame) for candidate #6
            q_current = np.array(backend.get_joint_positions())
            _, ee_quat = ik.get_ee_pose(q_current)
            R_fk = quat_to_matrix(ee_quat)

            # Apply selected transformation
            wrench_base = apply_transform(active_transform, raw_wrench, R, R_fk)

            # Admittance dynamics
            disp = controller.update(wrench_base, dt)

            # Target = home + displacement
            target_pos = home_pos + disp[:3]
            target_quat = apply_rotation_delta(home_quat, disp[3:], 1.0)

            # IK (q_current already read above for FK rotation)
            ik.sync_configuration(q_current)
            q_target = ik.solve(target_pos, target_quat, dt)
            if q_target is None:
                q_target = q_current.copy()

            # ServoJ
            backend.send_joint_command(q_target.tolist())

            # Get current EE pose for display
            ee_pos, _ = ik.get_ee_pose(q_target)

            # Display
            lines = ["\033[2J\033[H"]
            lines.append("=== Wrench Frame Diagnostic (SERVO) ===")
            lines.append(
                f"Transform: [{active_transform}] {TRANSFORM_NAMES[active_transform]}"
            )
            lines.append(
                f"TCP pose:  [{tcp_pose[0]:+.3f}, {tcp_pose[1]:+.3f}, {tcp_pose[2]:+.3f}"
                f" | {rotvec[0]:+.3f}, {rotvec[1]:+.3f}, {rotvec[2]:+.3f}]"
            )
            lines.append("")
            lines.append(
                f"Wrench(base): [{wrench_base[0]:+6.1f}, {wrench_base[1]:+6.1f},"
                f" {wrench_base[2]:+6.1f} |"
                f" {wrench_base[3]:+5.2f}, {wrench_base[4]:+5.2f}, {wrench_base[5]:+5.2f}]"
            )
            lines.append(
                f"Displacement: [{disp[0]:+.4f}, {disp[1]:+.4f}, {disp[2]:+.4f} |"
                f" {disp[3]:+.4f}, {disp[4]:+.4f}, {disp[5]:+.4f}]"
            )
            lines.append(
                f"EE position:  [{ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, {ee_pos[2]:+.3f}]"
            )
            lines.append("")
            lines.append("[1-6] switch transform  [z] zero+reset  [q] quit")
            lines.append("Push the robot -> it should move in the SAME direction.")

            sys.stdout.write("\n".join(lines))
            sys.stdout.flush()

            # Loop timing
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        backend.on_trajectory_done()


def main():
    parser = argparse.ArgumentParser(description="Wrench frame diagnostic")
    parser.add_argument("--robot-ip", default="192.168.0.2")
    parser.add_argument("--servo", action="store_true",
                        help="Enable servo mode (F/T → admittance → IK → servoJ)")
    args = parser.parse_args()

    backend = create_backend("rtde", robot_ip=args.robot_ip)
    backend.connect()

    try:
        if args.servo:
            run_servo_mode(backend)
        else:
            run_sensor_mode(backend)
    finally:
        backend.disconnect()
        print("\n[Diag] Done.")


if __name__ == "__main__":
    main()
