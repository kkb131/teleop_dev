#!/usr/bin/env python3
"""Teleop servo control -- main loop orchestrator.

Pipeline: Input -> ExpFilter -> Workspace Clamp -> [Admittance] -> Pink IK -> SafetyMonitor -> Robot

Usage:
  # Sim mode (mock hardware or Isaac Sim)
  python3 -m standalone.teleop_admittance.main --mode sim --input keyboard

  # Real robot
  python3 -m standalone.teleop_admittance.main --mode rtde --input keyboard --robot-ip 192.168.0.2

  # With custom config and logging
  python3 -m standalone.teleop_admittance.main --mode sim --config path/to/config.yaml --log
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime
from typing import Optional

import numpy as np
import pinocchio as pin

from teleop_dev.robot.config import URDF_PATH
from teleop_dev.robot.core.robot_backend import create_backend, RobotBackend
from teleop_dev.robot.arm.admittance.teleop_config import TeleopConfig
from teleop_dev.robot.core.exp_filter import ExpFilter
from teleop_dev.robot.core.pink_ik import PinkIK
from teleop_dev.robot.core.input_handler import create_input, InputHandler
from teleop_dev.robot.arm.admittance.safety_monitor import SafetyMonitor
from teleop_dev.robot.arm.admittance.admittance_layer import AdmittanceLayer


HELP_KEYBOARD = """\
=== UR10e Teleop Servo (Pink IK) ===
  W/S : Fwd/Back  U/O : Roll +/-
  A/D : Left/Right I/K : Pitch +/-
  Q/E : Up/Down   J/L : Yaw +/-
  C/V : Tool Z +/- (EE forward/back)
  +/= : Speed up    -  : Speed down
  Space : E-Stop   R  : Reset E-Stop
  ESC/x : Quit
  --- Admittance ---
  t   : Toggle ON/OFF   z  : Zero F/T
  1/2/3/4 : Stiff/Med/Soft/Free
========================================"""

HELP_JOYSTICK = """\
=== UR10e Teleop Servo (Pink IK) ===
  L-Stick : XY move   R-Stick : Roll/Pitch
  LT/RT   : Down/Up   LB/RB   : Yaw -/+
  D-pad L/R : Tool Z +/- (EE fwd/back)
  D-pad U/D : Speed +/-
  B : Cycle Preset   Y : Zero F/T
  START : Reset   BACK : Quit   Logo : E-Stop
  Admittance: always ON
========================================"""

# Number of fixed status lines
STATUS_LINES = 8


def apply_rotation_delta(
    quat_xyzw: np.ndarray, angular_vel: np.ndarray, dt: float,
    local_frame: bool = True,
) -> np.ndarray:
    """Apply angular velocity delta to a quaternion.

    Args:
        local_frame: If True, rotate in body/tool frame (R @ dR).
                     If False, rotate in world/base frame (dR @ R).
    """
    angle = np.linalg.norm(angular_vel) * dt
    if angle < 1e-10:
        return quat_xyzw.copy()

    axis = angular_vel / (np.linalg.norm(angular_vel) + 1e-15)
    aa = pin.AngleAxis(angle, axis)
    dR = aa.matrix()

    q_pin = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    if local_frame:
        R_new = q_pin.matrix() @ dR   # tool-frame (user input)
    else:
        R_new = dR @ q_pin.matrix()   # base-frame (admittance)
    q_new = pin.Quaternion(R_new)

    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])


class TeleopController:
    """Main teleop controller integrating all modules."""

    def __init__(self, config: TeleopConfig, log_path: Optional[str] = None,
                 extra_input_kwargs: dict | None = None):
        self.config = config
        self.running = True
        self._extra_input_kwargs = extra_input_kwargs or {}

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
        self.safety: Optional[SafetyMonitor] = None
        self.admittance: Optional[AdmittanceLayer] = None

        # State
        self.q_current: Optional[np.ndarray] = None
        self.ee_pos: Optional[np.ndarray] = None
        self.ee_quat: Optional[np.ndarray] = None
        self._speed_scale = 1.0

        # Logging
        self._log_writer = None
        self._log_file = None
        if log_path:
            self._log_file = open(log_path, "w", newline="")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow([
                "timestamp", "ee_x", "ee_y", "ee_z",
                "ee_roll", "ee_pitch", "ee_yaw",
                "j1", "j2", "j3", "j4", "j5", "j6",
                "ee_vel", "safety_status",
            ])

        # Display
        self._last_display_time = 0.0

        # Sim-mode ROS controller switcher
        self._switcher = None
        self._ros_node = None

    def _setup_sim_controller(self):
        """Switch to forward_position_controller for sim mode.

        Only needed with ROS2 mock hardware driver.
        Isaac Sim handles position commands directly -- no controller_manager.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from teleop_dev.robot.core.controller_utils import ControllerSwitcher
        except ImportError:
            return

        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("teleop_controller_switch")
        self._switcher = ControllerSwitcher(self._ros_node)

        if not self._switcher.wait_for_services(timeout_sec=3.0):
            # No controller_manager (Isaac Sim) -- skip silently
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

    def _write_status(self, ee_vel: float, safety_status: str, safety_msg: str):
        """Write fixed status block to terminal. No ANSI — use \\r overwrite."""
        now = time.monotonic()
        if now - self._last_display_time < 0.1:
            return
        self._last_display_time = now

        if self.ee_pos is None or self.q_current is None:
            return

        rpy = self.ik.get_ee_rpy(self.q_current)
        rpy_deg = [math.degrees(r) for r in rpy]
        joints_deg = [math.degrees(j) for j in self.q_current]

        # Build status block — each line padded to fixed width to overwrite previous
        w = 72  # line width

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
            f"  Safety : {safety_status}  {safety_msg}".ljust(w),
            f"  E-Stop : {'!! ACTIVE !! (R: reset)' if self.safety.estop_active else 'off (Space: trigger)'}".ljust(w),
            f"  Admit  : {adm_str}".ljust(w),
            f"  Input  : {self.safety.time_since_input_ms:.0f}ms ago  |  {self.config.robot.mode} {self.config.frequency}Hz".ljust(w),
        ]

        # Move cursor up N lines, write, stay at bottom
        output = f"\033[{STATUS_LINES}A"
        for line in lines:
            output += f"\r{line}\n"
        sys.stdout.write(output)
        sys.stdout.flush()

    def _log_step(self, ee_vel: float, safety_status: str):
        """Write one row to CSV log."""
        if self._log_writer is None:
            return
        rpy = self.ik.get_ee_rpy(self.q_current)
        self._log_writer.writerow([
            f"{time.time():.6f}",
            f"{self.ee_pos[0]:.6f}", f"{self.ee_pos[1]:.6f}", f"{self.ee_pos[2]:.6f}",
            f"{rpy[0]:.6f}", f"{rpy[1]:.6f}", f"{rpy[2]:.6f}",
            *[f"{j:.6f}" for j in self.q_current],
            f"{ee_vel:.6f}", safety_status,
        ])

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

        vive_kwargs = {
            "vive_port": cfg.input.vive_port,
            "vive_linear_scale": cfg.input.vive_linear_scale,
            "vive_angular_scale": cfg.input.vive_angular_scale,
            "vive_deadzone": cfg.input.vive_deadzone,
            "calibration_file": cfg.input.vive_calibration_file,
        }
        vive_kwargs.update(self._extra_input_kwargs)  # CLI overrides

        # Unified input: pose_provider set later after IK init (see run())
        unified_kwargs = {
            "unified_port": cfg.input.unified_port,
        }

        self.input_handler = create_input(
            cfg.input.type,
            cartesian_step=cfg.input.cartesian_step,
            rotation_step=cfg.input.rotation_step,
            linear_scale=lin_scale,
            angular_scale=ang_scale,
            network_port=cfg.input.network_port,
            **vive_kwargs,
            **unified_kwargs,
        )

        # Sim mode: switch controller
        if cfg.robot.mode == "sim":
            self._setup_sim_controller()

        dt = cfg.dt
        print(f"\n[Teleop] Mode: {cfg.robot.mode} | Input: {cfg.input.type} | "
              f"Freq: {cfg.frequency}Hz | dt: {dt*1000:.1f}ms")
        print(f"[Teleop] URDF: {cfg.urdf_path}")
        print(f"[Teleop] Safety: timeout={cfg.safety.packet_timeout_ms}ms, "
              f"max_jvel={cfg.safety.max_joint_vel} rad/s, "
              f"max_ee_vel={cfg.safety.max_ee_velocity} m/s")

        with self.backend:
            # Wait for initial joint state
            print("[Teleop] Waiting for joint state...")
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
            self.safety = SafetyMonitor(cfg.safety, self.backend)
            self.admittance = AdmittanceLayer(
                cfg.admittance, self.backend, cfg.robot.mode
            )

            # Set pose_provider for unified input (needs IK + backend ready)
            from teleop_dev.robot.core.input_handler import UnifiedNetworkInput
            if isinstance(self.input_handler, UnifiedNetworkInput):
                self.input_handler._pose_provider = lambda: self.ik.get_ee_pose(
                    np.array(self.backend.get_joint_positions())
                )

            print(f"[Teleop] Initial EE: x={self.ee_pos[0]:.4f} y={self.ee_pos[1]:.4f} z={self.ee_pos[2]:.4f}")
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
        # Move past status display
        print()
        if cfg.robot.mode == "sim" and self._switcher:
            print("Restoring original controller...")
            self._cleanup_sim_controller()
        if self._log_file:
            self._log_file.close()
            print("[Teleop] Log saved.")

        print("[Teleop] Done.")

    def _control_loop(self, dt: float):
        """Inner control loop running at configured frequency."""
        prev_ee_pos = self.ee_pos.copy()

        # Persistent target -- accumulates deltas so holding a key
        # continuously moves the target forward.
        target_pos = self.ee_pos.copy()
        target_quat = self.ee_quat.copy()

        # Reset safety timestamp so the 200ms timeout countdown starts
        # from loop entry, not from SafetyMonitor creation.
        self.safety.update_input_timestamp()

        while self.running:
            t_start = time.perf_counter()

            # 1. Read input
            cmd = self.input_handler.get_command(timeout=0.001)
            self._speed_scale = cmd.speed_scale

            if cmd.quit:
                self.running = False
                break

            if cmd.estop:
                self.safety.trigger_estop()

            if cmd.reset:
                self.safety.reset_estop()
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

            # Determine if there is input
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                has_input = True
            else:
                has_input = np.any(cmd.velocity != 0) or cmd.tool_z_delta != 0.0
            if has_input or self.admittance.enabled:
                self.safety.update_input_timestamp()

            # 2. Update target pose
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                # Absolute pose from unified protocol — use directly
                target_pos = cmd.target_pos.copy()
                target_quat = cmd.target_quat.copy()  # xyzw (already converted)
            else:
                # Velocity mode (local keyboard/xbox) — accumulate
                target_pos = target_pos + cmd.velocity[:3]
                target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

            # 2b. Tool-frame Z-axis translation
            if cmd.tool_z_delta != 0.0:
                R = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).matrix()
                tool_z = R[:, 2]
                target_pos += tool_z * cmd.tool_z_delta

            # 3. Exponential filter
            filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

            # 4. Workspace clamping (Level 3)
            clamped_pos = self.safety.clamp_workspace(filt_pos)
            # Keep target in sync with clamped bounds
            target_pos = clamped_pos.copy()

            # 4.5 Admittance displacement
            adm_disp = self.admittance.compute_displacement(self.q_current, dt)

            compliant_pos = clamped_pos + adm_disp[:3]
            compliant_quat = apply_rotation_delta(filt_quat, adm_disp[3:], 1.0, local_frame=False)
            # Re-clamp after admittance offset
            compliant_pos = self.safety.clamp_workspace(compliant_pos)

            # 5. Pink IK (soft sync to actual state to prevent config drift)
            q_actual = np.array(self.backend.get_joint_positions())
            self.ik.soft_sync(q_actual, alpha=self.config.ik.soft_sync_alpha)
            q_target = self.ik.solve(compliant_pos, compliant_quat, dt)
            if q_target is None:
                q_target = self.q_current.copy()

            # 6. Safety check (Levels 1, 2, 4)
            result = self.safety.check_and_apply(q_target, self.q_current, dt)

            # 7. Compute EE velocity for display
            ee_vel = np.linalg.norm(self.ee_pos - prev_ee_pos) / dt
            prev_ee_pos = self.ee_pos.copy()

            # 8. Send command
            if result.is_safe:
                self.backend.send_joint_command(result.q_safe.tolist())
                self.q_current = result.q_safe.copy()
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
            else:
                self.q_current = np.array(self.backend.get_joint_positions())
                # Timeout: hold position via servoJ to keep stream alive
                # E-Stop: stopScript() already called, don't send servoJ
                if result.level != "ESTOP":
                    self.backend.send_joint_command(self.q_current.tolist())
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)

            # Sync target_quat to pre-admittance orientation to prevent drift
            # (same pattern as target_pos = clamped_pos.copy() above)
            target_quat = filt_quat.copy()

            # 9. Display & log
            self._write_status(ee_vel, result.level, result.message)
            self._log_step(ee_vel, result.level)

            # 10. Loop timing
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description="UR10e Teleop Servo Control")
    parser.add_argument("--mode", choices=["sim", "rtde"], default=None,
                        help="Backend mode (overrides config)")
    parser.add_argument("--input", choices=["keyboard", "xbox", "network", "vive", "unified"], default=None,
                        help="Input device (overrides config)")
    parser.add_argument("--vive-port", type=int, default=9871,
                        help="UDP port for Vive tracker input (default: 9871)")
    parser.add_argument("--calibration-file", type=str, default=None,
                        help="Vive calibration JSON file (SteamVR→robot transform)")
    parser.add_argument("--robot-ip", type=str, default=None,
                        help="Robot IP for rtde mode (overrides config)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to YAML config file")
    parser.add_argument("--log", action="store_true",
                        help="Enable CSV logging")
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

    # Log path
    log_path = None
    if args.log:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"teleop_log_{ts}.csv"
        print(f"[Teleop] Logging to: {log_path}")

    # Vive-specific kwargs
    extra_input_kwargs = {}
    if hasattr(args, "vive_port") and args.vive_port:
        extra_input_kwargs["vive_port"] = args.vive_port
    if hasattr(args, "calibration_file") and args.calibration_file:
        extra_input_kwargs["calibration_file"] = args.calibration_file

    # Run
    controller = TeleopController(config, log_path=log_path,
                                  extra_input_kwargs=extra_input_kwargs)

    def signal_handler(sig, frame):
        controller.running = False

    signal.signal(signal.SIGINT, signal_handler)

    controller.run()


if __name__ == "__main__":
    main()
