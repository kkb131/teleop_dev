#!/usr/bin/env python3
"""Impedance teleop control -- Python PD torque via custom URScript at 500Hz.

Pipeline: Input -> ExpFilter -> Workspace Clamp -> Pink IK -> q_desired
  RTDE mode:  q_desired -> Python PD torque -> RTDE registers -> URScript direct_torque() (500Hz)
  Sim mode:   q_desired -> send_joint_command() (position fallback)

Usage:
  # Sim mode (mock hardware — position fallback, no torque)
  python3 -m robot.arm.impedance.main --mode sim --input keyboard

  # Real robot (impedance torque control)
  python3 -m robot.arm.impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2
"""

import argparse
import csv
import math
import signal
import sys
import time
from datetime import datetime
from typing import Optional

import numpy as np
import pinocchio as pin

from robot.config import URDF_PATH
from robot.core.robot_backend import create_backend, RobotBackend
from robot.core.exp_filter import ExpFilter
from robot.core.pink_ik import PinkIK
from robot.core.input_handler import create_input, InputHandler
from robot.arm.impedance.impedance_config import ImpedanceConfig
from robot.arm.impedance.impedance_gains import (
    ImpedanceController,
    IMPEDANCE_PRESETS,
)
from robot.arm.impedance.torque_safety import TorqueSafetyMonitor
from robot.arm.impedance.urscript_manager import TORQUE_LIMITS


HELP_KEYBOARD = """\
=== UR10e Impedance Teleop (URScript PD) ===
  W/S : Fwd/Back  U/O : Roll +/-
  A/D : Left/Right I/K : Pitch +/-
  Q/E : Up/Down   J/L : Yaw +/-
  C/V : Tool Z +/-
  +/= : Speed up    -  : Speed down
  Space : E-Stop   R  : Reset E-Stop
  ESC/x : Quit
  --- Impedance ---
  1/2/3 : Stiff/Medium/Soft preset
  [/]   : Gain scale down/up
============================================"""

STATUS_LINES = 8


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


class ImpedanceTeleopController:
    """Main impedance teleop controller.

    Python side (125Hz): Input -> Filter -> IK -> q_desired -> RTDE registers
    URScript side (500Hz): PD torque = Kp*(q_d - q) - Kd*qd + C(q,qd)
    """

    def __init__(self, config: ImpedanceConfig, log_path: Optional[str] = None):
        self.config = config
        self.running = True

        # Modules
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
        self.impedance = ImpedanceController(config.impedance.default_preset)
        self.safety: Optional[TorqueSafetyMonitor] = None

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
                "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
            ])

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
            from robot.core.controller_utils import ControllerSwitcher
        except ImportError:
            return

        if not rclpy.ok():
            rclpy.init()

        self._ros_node = Node("impedance_teleop_controller_switch")
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

    def _write_status(
        self,
        ee_vel: float,
        safety_level: str,
        safety_msg: str,
        applied_torques: Optional[np.ndarray] = None,
    ):
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
        w = 76

        # Impedance status
        imp = self.impedance
        imp_str = f"{imp.preset_name} x{imp.scale:.2f}"
        if applied_torques is not None:
            tau_str = " ".join(f"{t:6.1f}" for t in applied_torques)
            imp_str += f" | tau: [{tau_str}]"

        lines = [
            f"  EE Pos : x={self.ee_pos[0]:7.4f}  y={self.ee_pos[1]:7.4f}  z={self.ee_pos[2]:7.4f} m".ljust(w),
            f"  EE RPY : R={rpy_deg[0]:6.1f}  P={rpy_deg[1]:6.1f}  Y={rpy_deg[2]:6.1f} deg".ljust(w),
            f"  Joints : [{' '.join(f'{j:6.1f}' for j in joints_deg)}] deg".ljust(w),
            f"  Vel    : {ee_vel:.4f} m/s  |  Speed: {self._speed_scale:.1f}x".ljust(w),
            f"  Safety : {safety_level}  {safety_msg}".ljust(w),
            f"  E-Stop : {'!! ACTIVE !! (R: reset)' if self.safety.estop_active else 'off (Space: trigger)'}".ljust(w),
            f"  Imped  : {imp_str}".ljust(w),
            f"  Input  : {self.safety.time_since_input_ms:.0f}ms ago  |  {self.config.robot.mode} {self.config.frequency}Hz".ljust(w),
        ]

        output = f"\033[{STATUS_LINES}A"
        for line in lines:
            output += f"\r{line}\n"
        sys.stdout.write(output)
        sys.stdout.flush()

    def _log_step(
        self, ee_vel: float, safety_status: str,
        applied_torques: Optional[np.ndarray] = None,
    ):
        """Write one row to CSV log."""
        if self._log_writer is None:
            return
        rpy = self.ik.get_ee_rpy(self.q_current)
        tau = applied_torques if applied_torques is not None else [0.0] * 6
        self._log_writer.writerow([
            f"{time.time():.6f}",
            f"{self.ee_pos[0]:.6f}", f"{self.ee_pos[1]:.6f}", f"{self.ee_pos[2]:.6f}",
            f"{rpy[0]:.6f}", f"{rpy[1]:.6f}", f"{rpy[2]:.6f}",
            *[f"{j:.6f}" for j in self.q_current],
            f"{ee_vel:.6f}", safety_status,
            *[f"{t:.4f}" for t in tau],
        ])

    def run(self):
        """Main entry point — dispatches to RTDE or sim mode."""
        cfg = self.config

        self.input_handler = create_input(
            cfg.input.type,
            cartesian_step=cfg.input.cartesian_step,
            rotation_step=cfg.input.rotation_step,
            unified_port=cfg.input.unified_port,
        )

        dt = cfg.dt
        print(f"\n[ImpedanceTeleop] Mode: {cfg.robot.mode} | Input: {cfg.input.type} | "
              f"Freq: {cfg.frequency}Hz | dt: {dt*1000:.1f}ms")
        print(f"[ImpedanceTeleop] Preset: {self.impedance.preset_name} | "
              f"Coriolis: {cfg.impedance.enable_coriolis_comp}")

        if cfg.robot.mode == "rtde":
            self._run_impedance(cfg, dt)
        else:
            self._run_sim_fallback(cfg, dt)

        if self._log_file:
            self._log_file.close()
            print("[ImpedanceTeleop] Log saved.")
        print("[ImpedanceTeleop] Done.")

    def _run_impedance(self, cfg: ImpedanceConfig, dt: float):
        """Real robot: Python PD torque via custom URScript + RTDE registers."""
        from robot.arm.impedance.urscript_manager import URScriptManager

        mgr = URScriptManager(cfg.robot.ip, frequency=cfg.frequency)
        mgr.connect()

        # Wait for initial joint state
        print("[ImpedanceTeleop] Waiting for joint state...")
        for _ in range(50):
            q = mgr.get_joint_positions()
            if any(v != 0.0 for v in q):
                break
            time.sleep(0.1)

        self.q_current = np.array(mgr.get_joint_positions())
        self.ik.initialize(self.q_current)
        self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
        self.exp_filter.reset(self.ee_pos, self.ee_quat)
        self.safety = TorqueSafetyMonitor(cfg.safety)

        # Set pose_provider for unified input
        from robot.core.input_handler import UnifiedNetworkInput
        if isinstance(self.input_handler, UnifiedNetworkInput):
            self.input_handler._pose_provider = lambda: self.ik.get_ee_pose(
                np.array(mgr.get_joint_positions())
            )

        print(f"[ImpedanceTeleop] Initial EE: x={self.ee_pos[0]:.4f} "
              f"y={self.ee_pos[1]:.4f} z={self.ee_pos[2]:.4f}")
        print(HELP_KEYBOARD)
        for _ in range(STATUS_LINES):
            print()

        # Activate torque relay on URScript side
        mgr.set_mode(1)

        try:
            with self.input_handler:
                self._control_loop_impedance(cfg, dt, mgr)
        except KeyboardInterrupt:
            pass
        finally:
            print("\n[ImpedanceTeleop] Stopping...")
            mgr.set_mode(-1)
            time.sleep(0.2)
            mgr.disconnect()

    def _run_sim_fallback(self, cfg: ImpedanceConfig, dt: float):
        """Sim mode: position control fallback (same pipeline, no torque)."""
        self._setup_sim_controller()

        backend_kwargs = {"robot_ip": cfg.robot.ip}
        backend = create_backend("sim", **backend_kwargs)

        with backend:
            print("[ImpedanceTeleop] Waiting for joint state...")
            for _ in range(50):
                q = backend.get_joint_positions()
                if any(v != 0.0 for v in q):
                    break
                time.sleep(0.1)

            self.q_current = np.array(backend.get_joint_positions())
            self.ik.initialize(self.q_current)
            self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
            self.exp_filter.reset(self.ee_pos, self.ee_quat)
            self.safety = TorqueSafetyMonitor(cfg.safety)

            # Set pose_provider for unified input
            from robot.core.input_handler import UnifiedNetworkInput
            if isinstance(self.input_handler, UnifiedNetworkInput):
                self.input_handler._pose_provider = lambda: self.ik.get_ee_pose(
                    np.array(backend.get_joint_positions())
                )

            print(f"[ImpedanceTeleop] Initial EE: x={self.ee_pos[0]:.4f} "
                  f"y={self.ee_pos[1]:.4f} z={self.ee_pos[2]:.4f}")
            print("[ImpedanceTeleop] Sim mode: position fallback (no torque)")
            print(HELP_KEYBOARD)
            for _ in range(STATUS_LINES):
                print()

            try:
                with self.input_handler:
                    self._control_loop_sim(cfg, dt, backend)
            except KeyboardInterrupt:
                pass

        if self._switcher:
            print("Restoring original controller...")
            self._cleanup_sim_controller()

    def _handle_command(self, cmd):
        """Handle common command flags (estop, reset, impedance presets)."""
        if cmd.quit:
            self.running = False
            return True

        if cmd.estop:
            self.safety.trigger_estop()

        if cmd.reset:
            self.safety.reset_estop()
            return "reset"

        # Impedance presets (1/2/3 keys or B button cycle)
        preset = cmd.admittance_preset or cmd.impedance_preset
        if preset and preset in IMPEDANCE_PRESETS:
            self.impedance.set_preset(preset)
        if cmd.admittance_cycle:
            self.impedance.cycle_preset()

        # Gain scaling
        if cmd.gain_scale_up:
            self.impedance.scale_up()
        if cmd.gain_scale_down:
            self.impedance.scale_down()

        self._speed_scale = cmd.speed_scale
        return False

    def _control_loop_impedance(self, cfg, dt, mgr):
        """Inner loop for RTDE mode — Python-side PD torque via RTDE registers."""
        prev_ee_pos = self.ee_pos.copy()
        target_pos = self.ee_pos.copy()
        target_quat = self.ee_quat.copy()
        q_desired = self.q_current.copy()
        max_tau = np.array(TORQUE_LIMITS)
        max_q_error = np.array(cfg.impedance.max_joint_error)
        enable_coriolis = cfg.impedance.enable_coriolis_comp
        ik_lookahead_dt = cfg.ik.impedance_lookahead
        active = True  # torque control active flag

        # Reset safety timestamp so timeout countdown starts from loop entry
        self.safety.update_input_timestamp()

        while self.running:
            t_loop_start = time.perf_counter()

            # 1. Read input
            cmd = self.input_handler.get_command(timeout=0.0)
            result = self._handle_command(cmd)
            if result is True:
                break
            if result == "reset":
                self.q_current = np.array(mgr.get_joint_positions())
                self.ik.sync_configuration(self.q_current)
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
                self.exp_filter.reset(self.ee_pos, self.ee_quat)
                target_pos = self.ee_pos.copy()
                target_quat = self.ee_quat.copy()
                q_desired = self.q_current.copy()
                active = True
                mgr.set_mode(1)

            # Determine input presence
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                has_input = True
            else:
                has_input = np.any(cmd.velocity != 0) or cmd.tool_z_delta != 0.0
            if has_input:
                self.safety.update_input_timestamp()

            # 2. Update target pose
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                target_pos = cmd.target_pos.copy()
                target_quat = cmd.target_quat.copy()
            else:
                target_pos = target_pos + cmd.velocity[:3]
                target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

            # 2b. Tool-frame Z-axis translation (velocity mode only)
            if cmd.tool_z_delta != 0.0:
                R = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).matrix()
                tool_z = R[:, 2]
                target_pos += tool_z * cmd.tool_z_delta

            # 3. Exponential filter
            filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

            # 4. Workspace clamping
            clamped_pos = self.safety.clamp_workspace(filt_pos)
            target_pos = clamped_pos.copy()

            # 5. Read actual state from robot
            q_actual = np.array(mgr.get_joint_positions())
            qd_actual = np.array(mgr.get_joint_velocities())

            # 6. Pink IK → q_desired (sync + lookahead for impedance)
            # Sync to actual state each loop so IK computes from correct starting
            # point, then solve with large virtual dt so q_desired steps far enough
            # toward the Cartesian target to produce meaningful PD spring torque.
            self.ik.sync_configuration(q_actual)
            q_ik = self.ik.solve(clamped_pos, filt_quat, ik_lookahead_dt)
            if q_ik is not None:
                q_desired = q_ik

            # 7. Safety check
            safety_result = self.safety.check(q_desired, q_actual, qd_actual)

            # 8. Compute EE velocity for display
            ee_vel = np.linalg.norm(self.ee_pos - prev_ee_pos) / dt
            prev_ee_pos = self.ee_pos.copy()

            # 9. Compute PD torque and send
            # In impedance mode, only E-STOP zeroes torque.
            # PD runs continuously — deviation IS the control signal,
            # and timeout should hold position (not sag under gravity).
            applied_torques = None
            if safety_result.level == "ESTOP":
                # E-stop: zero torque, full reset
                mgr.send_torque([0.0] * 6)
                if active:
                    mgr.set_mode(0)
                active = False
                self.q_current = q_actual
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
                q_desired = q_actual.copy()
            else:
                # Normal PD torque (safe, timeout, deviation, vel_limit)
                # Safety is guaranteed by max_joint_error clipping + TORQUE_LIMITS
                if not active:
                    active = True
                    mgr.set_mode(1)

                q_error = np.clip(q_desired - q_actual,
                                  -max_q_error, max_q_error)
                tau = self.impedance.Kp * q_error \
                    - self.impedance.Kd * qd_actual

                if enable_coriolis:
                    coriolis = np.array(mgr.get_coriolis(
                        q_actual.tolist(), qd_actual.tolist()
                    ))
                    tau += coriolis

                tau = np.clip(tau, -max_tau, max_tau)
                applied_torques = tau.copy()

                mgr.send_torque(tau.tolist())
                self.q_current = q_actual
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)

            # 10. Display & log
            self._write_status(ee_vel, safety_result.level, safety_result.message,
                               applied_torques)
            self._log_step(ee_vel, safety_result.level, applied_torques)

            # 11. Servo loop timing
            elapsed = time.perf_counter() - t_loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _control_loop_sim(self, cfg, dt, backend):
        """Inner loop for sim mode (position control fallback)."""
        prev_ee_pos = self.ee_pos.copy()
        target_pos = self.ee_pos.copy()
        target_quat = self.ee_quat.copy()

        # Reset safety timestamp so timeout countdown starts from loop entry
        self.safety.update_input_timestamp()

        while self.running:
            t_start = time.perf_counter()

            # 1. Read input
            cmd = self.input_handler.get_command(timeout=0.001)
            result = self._handle_command(cmd)
            if result is True:
                break
            if result == "reset":
                self.q_current = np.array(backend.get_joint_positions())
                self.ik.sync_configuration(self.q_current)
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
                self.exp_filter.reset(self.ee_pos, self.ee_quat)
                target_pos = self.ee_pos.copy()
                target_quat = self.ee_quat.copy()

            # Determine input presence
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                has_input = True
            else:
                has_input = np.any(cmd.velocity != 0) or cmd.tool_z_delta != 0.0
            if has_input:
                self.safety.update_input_timestamp()

            # 2. Update target pose
            if cmd.mode == "absolute" and cmd.target_pos is not None:
                target_pos = cmd.target_pos.copy()
                target_quat = cmd.target_quat.copy()
            else:
                target_pos = target_pos + cmd.velocity[:3]
                target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

            # 2b. Tool-frame Z-axis translation (velocity mode only)
            if cmd.tool_z_delta != 0.0:
                R = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).matrix()
                tool_z = R[:, 2]
                target_pos += tool_z * cmd.tool_z_delta

            # 3. Filter
            filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

            # 4. Workspace clamp
            clamped_pos = self.safety.clamp_workspace(filt_pos)
            target_pos = clamped_pos.copy()

            # 5. Read actual state and soft sync IK
            q_actual = np.array(backend.get_joint_positions())
            qd_actual = np.array(backend.get_joint_velocities())
            self.ik.soft_sync(q_actual, alpha=self.config.ik.soft_sync_alpha)

            # 6. IK
            q_desired = self.ik.solve(clamped_pos, filt_quat, dt)
            if q_desired is None:
                q_desired = self.q_current.copy()
            safety_result = self.safety.check(q_desired, q_actual, qd_actual)

            # 7. EE velocity
            ee_vel = np.linalg.norm(self.ee_pos - prev_ee_pos) / dt
            prev_ee_pos = self.ee_pos.copy()

            # 8. Send position command (sim fallback)
            if safety_result.is_safe:
                backend.send_joint_command(q_desired.tolist())
                self.q_current = q_desired.copy()
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
            else:
                if safety_result.level == "ESTOP":
                    backend.emergency_stop()
                self.q_current = np.array(backend.get_joint_positions())
                self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)

            target_quat = self.ee_quat.copy()

            # 9. Display & log
            self._write_status(ee_vel, safety_result.level, safety_result.message)
            self._log_step(ee_vel, safety_result.level)

            # 10. Loop timing
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description="UR10e Impedance Teleop Control")
    parser.add_argument("--mode", choices=["sim", "rtde"], default=None,
                        help="Backend mode (overrides config)")
    parser.add_argument("--input", choices=["keyboard", "unified"], default=None,
                        help="Input device (overrides config)")
    parser.add_argument("--robot-ip", type=str, default=None,
                        help="Robot IP for rtde mode (overrides config)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to YAML config file")
    parser.add_argument("--log", action="store_true",
                        help="Enable CSV logging")
    args = parser.parse_args()

    config = ImpedanceConfig.load(args.config)

    if args.mode:
        config.robot.mode = args.mode
    if args.input:
        config.input.type = args.input
    if args.robot_ip:
        config.robot.ip = args.robot_ip

    log_path = None
    if args.log:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"impedance_teleop_log_{ts}.csv"
        print(f"[ImpedanceTeleop] Logging to: {log_path}")

    controller = ImpedanceTeleopController(config, log_path=log_path)

    def signal_handler(sig, frame):
        controller.running = False

    signal.signal(signal.SIGINT, signal_handler)

    controller.run()


if __name__ == "__main__":
    main()
