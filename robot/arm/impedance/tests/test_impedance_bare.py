#!/usr/bin/env python3
"""Bare-minimum impedance control test — no safety, no IK, no input handler.

Connects to the robot, uploads URScript, and runs a PD position-hold loop.
The robot should hold its current position via impedance torque control.

Usage:
  python3 -m robot.arm.impedance.test_impedance_bare --robot-ip 192.168.0.2
  python3 -m robot.arm.impedance.test_impedance_bare --robot-ip 192.168.0.2 --preset MEDIUM
  python3 -m robot.arm.impedance.test_impedance_bare --robot-ip 192.168.0.2 --hz 50

Expected: robot holds position. Push it by hand → resists → returns.
"""

import argparse
import time

import numpy as np

from robot.arm.impedance.urscript_manager import URScriptManager, TORQUE_LIMITS
from robot.arm.impedance.impedance_gains import IMPEDANCE_PRESETS


def main():
    parser = argparse.ArgumentParser(description="Bare impedance hold test")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--preset", default="SOFT", choices=list(IMPEDANCE_PRESETS.keys()))
    parser.add_argument("--hz", type=float, default=125.0, help="Control loop frequency")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run for N seconds (0=until Ctrl+C)")
    parser.add_argument("--no-coriolis", action="store_true",
                        help="Disable Coriolis compensation")
    args = parser.parse_args()

    gains = IMPEDANCE_PRESETS[args.preset]
    Kp = gains.Kp
    Kd = gains.Kd
    max_tau = np.array(TORQUE_LIMITS)
    dt = 1.0 / args.hz
    enable_coriolis = not args.no_coriolis

    print(f"[BareTest] Preset={args.preset}  Hz={args.hz}  Coriolis={enable_coriolis}")
    print(f"[BareTest] Kp={Kp}")
    print(f"[BareTest] Kd={Kd}")

    mgr = URScriptManager(args.robot_ip)
    mgr.connect()

    # Wait for valid joint state
    print("[BareTest] Waiting for joint state...")
    for _ in range(50):
        q = mgr.get_joint_positions()
        if any(v != 0.0 for v in q):
            break
        time.sleep(0.1)

    q_desired = np.array(mgr.get_joint_positions())
    print(f"[BareTest] q_desired = [{', '.join(f'{v:.4f}' for v in q_desired)}]")

    # Wait for URScript to start (it begins in idle mode=0, calling sync())
    print("[BareTest] Waiting 1s for URScript to initialize...")
    time.sleep(1.0)

    # Activate torque relay
    mgr.set_mode(1)
    print("[BareTest] mode=1 (active) — robot should hold position now")
    print("[BareTest] Press Ctrl+C to stop\n")

    t_start = time.time()
    count = 0

    try:
        while True:
            t_loop = time.perf_counter()

            # Read state
            q = np.array(mgr.get_joint_positions())
            qd = np.array(mgr.get_joint_velocities())

            # PD torque
            q_error = q_desired - q
            tau = Kp * q_error - Kd * qd

            # Coriolis compensation
            if enable_coriolis:
                coriolis = np.array(mgr.get_coriolis(q.tolist(), qd.tolist()))
                tau += coriolis

            # Saturate
            tau = np.clip(tau, -max_tau, max_tau)

            # Send
            mgr.send_torque(tau.tolist())

            # Print every ~0.5s
            count += 1
            if count % max(1, int(args.hz * 0.5)) == 0:
                err_deg = np.degrees(q_error)
                print(f"  err(deg)=[{', '.join(f'{v:+6.2f}' for v in err_deg)}]  "
                      f"tau(Nm)=[{', '.join(f'{v:+7.2f}' for v in tau)}]  "
                      f"qd=[{', '.join(f'{v:+5.3f}' for v in qd)}]")

            # Duration check
            if args.duration > 0 and (time.time() - t_start) > args.duration:
                print(f"\n[BareTest] Duration {args.duration}s reached.")
                break

            # Timing
            elapsed = time.perf_counter() - t_loop
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[BareTest] Ctrl+C received.")
    except Exception as e:
        print(f"\n[BareTest] ERROR: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("[BareTest] Stopping...")
        mgr.set_mode(-1)
        time.sleep(0.2)
        mgr.disconnect()
        print("[BareTest] Done.")


if __name__ == "__main__":
    main()
