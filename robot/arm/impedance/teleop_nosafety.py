#!/usr/bin/env python3
"""Impedance teleop WITHOUT safety monitor — for testing core pipeline on real hardware.

Pipeline: Keyboard Input -> ExpFilter -> Pink IK -> q_desired
  -> Python PD torque -> RTDE registers -> URScript direct_torque() (500Hz)

No TorqueSafetyMonitor, no workspace clamping, no e-stop, no timeout.
Torque saturation (clip to TORQUE_LIMITS) is always applied.

Usage:
  python3 -m robot.arm.impedance.teleop_nosafety --robot-ip 192.168.0.2
  python3 -m robot.arm.impedance.teleop_nosafety --robot-ip 192.168.0.2 --preset MEDIUM
  python3 -m robot.arm.impedance.teleop_nosafety --robot-ip 192.168.0.2 --input unified
"""

import argparse
import math
import signal
import time

import numpy as np
import pinocchio as pin

from robot.core.exp_filter import ExpFilter
from robot.core.input_handler import create_input
from robot.core.pink_ik import PinkIK
from robot.arm.impedance.impedance_config import ImpedanceConfig
from robot.arm.impedance.impedance_gains import (
    ImpedanceController,
    IMPEDANCE_PRESETS,
)
from robot.arm.impedance.urscript_manager import URScriptManager, TORQUE_LIMITS


HELP_KEYBOARD = """\
=== Impedance Teleop (No Safety) ===
  W/S : Fwd/Back   U/O : Roll +/-
  A/D : Left/Right  I/K : Pitch +/-
  Q/E : Up/Down    J/L : Yaw +/-
  C/V : Tool Z +/-
  +/= : Speed up     -  : Speed down
  1/2/3 : Stiff/Medium/Soft preset
  [/]   : Gain scale down/up
  ESC/x : Quit
===================================="""



def apply_rotation_delta(
    quat_xyzw: np.ndarray, angular_vel: np.ndarray, dt: float
) -> np.ndarray:
    angle = np.linalg.norm(angular_vel) * dt
    if angle < 1e-10:
        return quat_xyzw.copy()

    axis = angular_vel / (np.linalg.norm(angular_vel) + 1e-15)
    aa = pin.AngleAxis(angle, axis)
    dR = aa.matrix()

    q_pin = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    R_new = q_pin.matrix() @ dR
    q_new = pin.Quaternion(R_new)

    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])


running = True


def main():
    global running

    parser = argparse.ArgumentParser(description="Impedance Teleop (No Safety)")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--preset", default="SOFT", choices=list(IMPEDANCE_PRESETS.keys()))
    parser.add_argument("--hz", type=float, default=125.0)
    parser.add_argument("--input", choices=["keyboard", "unified"], default="keyboard")
    parser.add_argument("--no-coriolis", action="store_true")
    parser.add_argument("--config", type=str, default=None)
    args = parser.parse_args()

    config = ImpedanceConfig.load(args.config)
    dt = 1.0 / args.hz
    max_tau = np.array(TORQUE_LIMITS)
    max_q_error = np.array(config.impedance.max_joint_error)
    enable_coriolis = not args.no_coriolis

    # Impedance controller
    impedance = ImpedanceController(args.preset)

    # IK solver
    ik = PinkIK(
        config.urdf_path,
        ee_frame="tool0",
        position_cost=config.ik.position_cost,
        orientation_cost=config.ik.orientation_cost,
        posture_cost=config.ik.posture_cost,
        damping=config.ik.damping,
    )

    # Exp filter
    exp_filter = ExpFilter(
        config.filter.alpha_position, config.filter.alpha_orientation
    )

    input_handler = create_input(
        args.input,
        cartesian_step=config.input.cartesian_step,
        rotation_step=config.input.rotation_step,
        unified_port=config.input.unified_port,
    )

    # Connect to robot
    print(f"[NoSafety] Preset={args.preset}  Hz={args.hz}  Coriolis={enable_coriolis}  Input={args.input}")
    mgr = URScriptManager(args.robot_ip)
    mgr.connect()

    # Wait for valid joint state
    print("[NoSafety] Waiting for joint state...")
    for _ in range(50):
        q = mgr.get_joint_positions()
        if any(v != 0.0 for v in q):
            break
        time.sleep(0.1)

    q_current = np.array(mgr.get_joint_positions())
    ik.initialize(q_current)
    ee_pos, ee_quat = ik.get_ee_pose(q_current)
    exp_filter.reset(ee_pos, ee_quat)

    target_pos = ee_pos.copy()
    target_quat = ee_quat.copy()
    q_desired = q_current.copy()

    print(f"[NoSafety] Initial EE: x={ee_pos[0]:.4f} y={ee_pos[1]:.4f} z={ee_pos[2]:.4f}")
    print(f"[NoSafety] Kp={impedance.Kp}")
    print(f"[NoSafety] Kd={impedance.Kd}")
    print(HELP_KEYBOARD)

    # Wait for URScript startup
    print("[NoSafety] Waiting 1s for URScript to initialize...")
    time.sleep(1.0)

    # Activate torque relay
    mgr.set_mode(1)
    print("[NoSafety] mode=1 (active) — teleop running. Press ESC/x or Ctrl+C to stop.\n")

    def signal_handler(sig, frame):
        global running
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    count = 0
    print_interval = max(1, int(args.hz * 0.5))

    try:
        with input_handler:
            while running:
                t_loop = time.perf_counter()

                # 1. Read input
                cmd = input_handler.get_command(timeout=0.0)

                if cmd.quit:
                    break

                # Impedance presets (1/2/3 keys or B button cycle)
                preset = cmd.admittance_preset or cmd.impedance_preset
                if preset and preset in IMPEDANCE_PRESETS:
                    impedance.set_preset(preset)
                    print(f"  [Preset -> {preset}] Kp={impedance.Kp}  Kd={impedance.Kd}")
                if cmd.admittance_cycle:
                    impedance.cycle_preset()
                    print(f"  [Preset -> {impedance.preset_name}] Kp={impedance.Kp}  Kd={impedance.Kd}")

                if cmd.gain_scale_up:
                    impedance.scale_up()
                    print(f"  [Scale -> x{impedance.scale:.2f}]")
                if cmd.gain_scale_down:
                    impedance.scale_down()
                    print(f"  [Scale -> x{impedance.scale:.2f}]")

                # 2. Accumulate target pose
                target_pos = target_pos + cmd.velocity[:3]
                target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

                # 2b. Tool-frame Z-axis translation
                if cmd.tool_z_delta != 0.0:
                    R = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).matrix()
                    tool_z = R[:, 2]
                    target_pos += tool_z * cmd.tool_z_delta

                # 3. Exp filter
                filt_pos, filt_quat = exp_filter.update(target_pos, target_quat)

                # 4. Read actual state
                q_actual = np.array(mgr.get_joint_positions())
                qd_actual = np.array(mgr.get_joint_velocities())

                # 4b. Soft sync IK toward actual state to prevent drift
                ik.soft_sync(q_actual, alpha=config.ik.soft_sync_alpha)

                # 5. Pink IK -> q_desired
                q_ik = ik.solve(filt_pos, filt_quat, dt)
                if q_ik is not None:
                    q_desired = q_ik

                # 6. PD torque (clamp error to prevent jerky acceleration)
                q_error = np.clip(q_desired - q_actual, -max_q_error, max_q_error)
                tau = impedance.Kp * q_error - impedance.Kd * qd_actual

                # 7. Coriolis compensation
                if enable_coriolis:
                    coriolis = np.array(mgr.get_coriolis(
                        q_actual.tolist(), qd_actual.tolist()
                    ))
                    tau += coriolis

                # 8. Torque saturation
                tau = np.clip(tau, -max_tau, max_tau)

                # 9. Send
                mgr.send_torque(tau.tolist())

                # Update state
                q_current = q_actual
                ee_pos, ee_quat = ik.get_ee_pose(q_current)

                # 10. Print status
                count += 1
                if count % print_interval == 0:
                    rpy = ik.get_ee_rpy(q_current)
                    rpy_deg = [math.degrees(r) for r in rpy]
                    err_deg = np.degrees(q_error)
                    status = (
                        f"  EE=[{ee_pos[0]:+.3f} {ee_pos[1]:+.3f} {ee_pos[2]:+.3f}]  "
                        f"RPY=[{rpy_deg[0]:+5.1f} {rpy_deg[1]:+5.1f} {rpy_deg[2]:+5.1f}]  "
                        f"err=[{' '.join(f'{v:+5.2f}' for v in err_deg)}]  "
                        f"tau=[{' '.join(f'{v:+6.1f}' for v in tau)}]  "
                        f"{impedance.preset_name} x{impedance.scale:.2f}"
                    )
                    print(f"\r{status}", end="", flush=True)

                # 11. Loop timing
                elapsed = time.perf_counter() - t_loop
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n[NoSafety] ERROR: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[NoSafety] Stopping...")
        mgr.set_mode(-1)
        time.sleep(0.2)
        mgr.disconnect()
        print("[NoSafety] Done.")


if __name__ == "__main__":
    main()
