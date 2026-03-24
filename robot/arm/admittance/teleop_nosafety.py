#!/usr/bin/env python3
"""Teleop admittance control without safety monitor.

Pipeline: Input -> ExpFilter -> [Admittance] -> Pink IK -> Robot (servoJ)

No workspace clamping, velocity limiting, timeout, or e-stop.
Use this for testing admittance teleop without safety constraints.

Usage:
  # Sim mode (mock hardware or Isaac Sim)
  python3 -m standalone.teleop_admittance.teleop_nosafety --mode sim --input keyboard

  # Real robot
  python3 -m standalone.teleop_admittance.teleop_nosafety --mode rtde --input keyboard --robot-ip 192.168.0.2
"""

import argparse
import math
import signal
import sys
import time
from typing import Optional

import numpy as np
import pinocchio as pin

from teleop_dev.robot.config import URDF_PATH
from teleop_dev.robot.core.robot_backend import create_backend, RobotBackend
from teleop_dev.robot.arm.admittance.teleop_config import TeleopConfig
from teleop_dev.robot.core.exp_filter import ExpFilter
from teleop_dev.robot.core.pink_ik import PinkIK
from teleop_dev.robot.core.input_handler import create_input, InputHandler
from teleop_dev.robot.arm.admittance.admittance_layer import AdmittanceLayer


HELP_KEYBOARD = """\
=== UR10e Teleop (No Safety) ===
  W/S : Fwd/Back  U/O : Roll +/-
  A/D : Left/Right I/K : Pitch +/-
  Q/E : Up/Down   J/L : Yaw +/-
  C/V : Tool Z +/- (EE forward/back)
  +/= : Speed up    -  : Speed down
  ESC/x : Quit
  --- Admittance ---
  t   : Toggle ON/OFF   z  : Zero F/T
  1/2/3/4 : Stiff/Med/Soft/Free
===================================="""

HELP_JOYSTICK = """\
=== UR10e Teleop (No Safety) ===
  L-Stick : XY move   R-Stick : Roll/Pitch
  LT/RT   : Down/Up   LB/RB   : Yaw -/+
  D-pad L/R : Tool Z +/- (EE fwd/back)
  D-pad U/D : Speed +/-
  B : Cycle Preset   Y : Zero F/T
  START : Reset   BACK : Quit   Logo : E-Stop
  Admittance: always ON
===================================="""

STATUS_LINES = 6


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
    R_new = q_pin.matrix() @ dR
    q_new = pin.Quaternion(R_new)

    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])


class TeleopNoSafety:
    """Teleop controller without safety monitor."""

    def __init__(self, config: TeleopConfig):
        self.config = config
        self.running = True

        # Modules
        self.backend: Optional[RobotBackend] = None
        self.input_handler: Optional[InputHandler] = None
        self.exp_filter = ExpFilter(
            config.filter.alpha_position, config.filter.alpha_orientation
        )
        self.ik = PinkIK(
            config.urdf_path,
            ee_frame="tool0",
            position_cost=config.ik.position_cost,
            orientation_cost=config.ik.orientation_cost,
            posture_cost=config.ik.posture_cost,
            damping=config.ik.damping,
        )
        self.admittance: Optional[AdmittanceLayer] = None

        # State
        self.q_current: Optional[np.ndarray] = None
        self.ee_pos: Optional[np.ndarray] = None
        self.ee_quat: Optional[np.ndarray] = None
        self._speed_scale = 1.0

        # Display
        self._last_display_time = 0.0

        # Sim-mode ROS controller switcher
        self._switcher = None
        self._ros_node = None

    def _setup_sim_controller(self):
        """Switch to forward_position_controller for sim mode."""
        try:
            import rclpy
            from rclpy.node import Node
            from teleop_dev.robot.core.controller_utils import ControllerSwitcher
        except ImportError:
            return

        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("teleop_nosafety_switch")
        self._switcher = ControllerSwitcher(self._ros_node)

        if not self._switcher.wait_for_services(timeout_sec=3.0):
            self._ros_node.destroy_node()
            self._ros_node = None
            self._switcher = None
            return

        self._switcher.activate_forward_position()

    def _cleanup_sim_controller(self):
        """Restore original controller on exit."""
        if self._switcher:
            self._switcher.restore_original()
        if self._ros_node:
            self._ros_node.destroy_node()

    def _write_status(self, ee_vel: float):
        """Write fixed status block to terminal."""
        now = time.monotonic()
        if now - self._last_display_time < 0.1:
            return
        self._last_display_time = now

        if self.ee_pos is None or self.q_current is None:
            return

        rpy = self.ik.get_ee_rpy(self.q_current)
        rpy_deg = [math.degrees(r) for r in rpy]
        joints_deg = [math.degrees(j) for j in self.q_current]

        w = 72

        # Admittance status line
        adm = self.admittance
        if adm is not None and adm.has_sensor:
            if adm.enabled:
                f_raw = adm.get_wrench()
                dx = adm.displacement
                adm_str = (f"ON [{adm.preset_name}] | "
                           f"F: [{f_raw[0]:.1f},{f_raw[1]:.1f},{f_raw[2]:.1f}]N | "
                           f"dx: {np.linalg.norm(dx[:3])*1000:.1f}mm")
            else:
                adm_str = f"OFF [{adm.preset_name}] (t: toggle)"
        else:
            adm_str = "N/A (sim mode)"

        lines = [
            f"  EE Pos : x={self.ee_pos[0]:7.4f}  y={self.ee_pos[1]:7.4f}  z={self.ee_pos[2]:7.4f} m".ljust(w),
            f"  EE RPY : R={rpy_deg[0]:6.1f}  P={rpy_deg[1]:6.1f}  Y={rpy_deg[2]:6.1f} deg".ljust(w),
            f"  Joints : [{' '.join(f'{j:6.1f}' for j in joints_deg)}] deg".ljust(w),
            f"  Vel    : {ee_vel:.4f} m/s  |  Speed: {self._speed_scale:.1f}x".ljust(w),
            f"  Admit  : {adm_str}".ljust(w),
            f"  Mode   : {self.config.robot.mode} {self.config.frequency}Hz  |  NO SAFETY".ljust(w),
        ]

        output = f"\033[{STATUS_LINES}A"
        for line in lines:
            output += f"\r{line}\n"
        sys.stdout.write(output)
        sys.stdout.flush()

    def run(self):
        """Main control loop."""
        cfg = self.config

        # Create backend
        backend_kwargs = {"robot_ip": cfg.robot.ip}
        self.backend = create_backend(cfg.robot.mode, **backend_kwargs)

        # Create input (network mode uses separate, lower velocity scales)
        if cfg.input.type == "network":
            lin_scale = cfg.input.network_linear_scale
            ang_scale = cfg.input.network_angular_scale
        else:
            lin_scale = cfg.input.xbox_linear_scale
            ang_scale = cfg.input.xbox_angular_scale

        self.input_handler = create_input(
            cfg.input.type,
            cartesian_step=cfg.input.cartesian_step,
            rotation_step=cfg.input.rotation_step,
            linear_scale=lin_scale,
            angular_scale=ang_scale,
            network_port=cfg.input.network_port,
        )

        # Sim mode: switch controller
        if cfg.robot.mode == "sim":
            self._setup_sim_controller()

        dt = cfg.dt
        print(f"\n[Teleop-NoSafety] Mode: {cfg.robot.mode} | Input: {cfg.input.type} | "
              f"Freq: {cfg.frequency}Hz | dt: {dt*1000:.1f}ms")
        print(f"[Teleop-NoSafety] URDF: {cfg.urdf_path}")
        print("[Teleop-NoSafety] WARNING: No safety monitor active!")

        with self.backend:
            # Wait for initial joint state
            print("[Teleop-NoSafety] Waiting for joint state...")
            for _ in range(50):
                q = self.backend.get_joint_positions()
                if any(v != 0.0 for v in q):
                    break
                time.sleep(0.1)

            self.q_current = np.array(self.backend.get_joint_positions())

            # Initialize modules
            self.ik.initialize(self.q_current)
            self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
            self.exp_filter.reset(self.ee_pos, self.ee_quat)
            self.admittance = AdmittanceLayer(
                cfg.admittance, self.backend, cfg.robot.mode
            )

            print(f"[Teleop-NoSafety] Initial EE: x={self.ee_pos[0]:.4f} y={self.ee_pos[1]:.4f} z={self.ee_pos[2]:.4f}")
            help_text = HELP_KEYBOARD if cfg.input.type == "keyboard" else HELP_JOYSTICK
            print(help_text)

            # Reserve blank lines for status display
            for _ in range(STATUS_LINES):
                print()

            with self.input_handler:
                try:
                    self._control_loop(dt)
                except KeyboardInterrupt:
                    pass

        # Cleanup
        print()
        if cfg.robot.mode == "sim" and self._switcher:
            print("Restoring original controller...")
            self._cleanup_sim_controller()

        print("[Teleop-NoSafety] Done.")

    def _control_loop(self, dt: float):
        """Inner control loop running at configured frequency."""
        prev_ee_pos = self.ee_pos.copy()

        target_pos = self.ee_pos.copy()
        target_quat = self.ee_quat.copy()

        while self.running:
            t_start = time.perf_counter()

            # 1. Read input
            cmd = self.input_handler.get_command(timeout=0.001)
            self._speed_scale = cmd.speed_scale

            if cmd.quit:
                self.running = False
                break

            if cmd.reset:
                # Re-sync IK and filter to current robot state
                self.q_current = np.array(self.backend.get_joint_positions())
                self.ik.sync_configuration(self.q_current)
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
                self.exp_filter.reset(self.ee_pos, self.ee_quat)
                self.admittance.reset()
                target_pos = self.ee_pos.copy()
                target_quat = self.ee_quat.copy()

            # Admittance commands
            if cmd.admittance_cycle:
                self.admittance.cycle_preset()
            if cmd.admittance_preset:
                self.admittance.set_preset(cmd.admittance_preset)
            if cmd.ft_zero:
                self.admittance.zero_sensor()

            # 2. Accumulate target pose
            target_pos = target_pos + cmd.velocity[:3]
            target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

            # 2b. Tool-frame Z-axis translation
            if cmd.tool_z_delta != 0.0:
                R = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).matrix()
                tool_z = R[:, 2]
                target_pos += tool_z * cmd.tool_z_delta

            # 3. Exponential filter
            filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

            # 4. Admittance displacement
            adm_disp = self.admittance.compute_displacement(self.q_current, dt)
            compliant_pos = filt_pos + adm_disp[:3]
            compliant_quat = apply_rotation_delta(filt_quat, adm_disp[3:], 1.0)

            # 5. Pink IK (soft sync to actual state to prevent drift)
            q_actual = np.array(self.backend.get_joint_positions())
            self.ik.soft_sync(q_actual, alpha=self.config.ik.soft_sync_alpha)
            q_target = self.ik.solve(compliant_pos, compliant_quat, dt)
            if q_target is None:
                q_target = self.q_current.copy()

            # 6. Compute EE velocity for display
            ee_vel = np.linalg.norm(self.ee_pos - prev_ee_pos) / dt
            prev_ee_pos = self.ee_pos.copy()

            # 7. Send command directly (no safety check)
            self.backend.send_joint_command(q_target.tolist())
            self.q_current = np.array(self.backend.get_joint_positions())
            self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)

            # 8. Display
            self._write_status(ee_vel)

            # 9. Loop timing
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description="UR10e Teleop (No Safety)")
    parser.add_argument("--mode", choices=["sim", "rtde"], default=None,
                        help="Backend mode (overrides config)")
    parser.add_argument("--input", choices=["keyboard", "xbox", "network"], default=None,
                        help="Input device (overrides config)")
    parser.add_argument("--robot-ip", type=str, default=None,
                        help="Robot IP for rtde mode (overrides config)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to YAML config file")
    args = parser.parse_args()

    # Load config
    config = TeleopConfig.load(args.config)

    # CLI overrides
    if args.mode:
        config.robot.mode = args.mode
    if args.input:
        config.input.type = args.input
    if args.robot_ip:
        config.robot.ip = args.robot_ip

    # Run
    controller = TeleopNoSafety(config)

    def signal_handler(sig, frame):
        controller.running = False

    signal.signal(signal.SIGINT, signal_handler)

    controller.run()


if __name__ == "__main__":
    main()
