#!/usr/bin/env python3
"""Step 5: ViveNetworkInput velocity computation test.

Sends mock UDP packets simulating known trajectories, then reads
the velocity output from ViveNetworkInput. NO SteamVR required.

Usage: python3 -m operator.arm.tests.test_step5_velocity [--port 9872]
"""

import argparse
import json
import math
import socket
import sys
import time
import threading

import numpy as np


def _send_mock_trajectory(port: int, trajectory: list[dict], hz: float = 50):
    """Send a sequence of mock packets to localhost."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", port)
    dt = 1.0 / hz

    for pkt in trajectory:
        data = json.dumps(pkt).encode()
        sock.sendto(data, target)
        time.sleep(dt)

    sock.close()


def _make_packet(pos, quat=None, tracking=True, t=None):
    if quat is None:
        quat = [1.0, 0.0, 0.0, 0.0]
    return {
        "type": "vive",
        "pos": list(pos),
        "quat": list(quat),
        "tracking": tracking,
        "buttons": {"estop": False, "reset": False, "quit": False,
                    "speed_up": False, "speed_down": False},
        "timestamp": t if t is not None else time.time(),
    }


def main():
    parser = argparse.ArgumentParser(description="Step 5: Velocity computation test")
    parser.add_argument("--port", type=int, default=9872,
                        help="UDP port for test (default: 9872, avoid conflict)")
    args = parser.parse_args()

    # Import ViveNetworkInput here (requires standalone package)
    try:
        from robot.core.input_handler import ViveNetworkInput
    except ImportError:
        print("[FAIL] Cannot import ViveNetworkInput")
        print("       Run from: cd /workspaces/tamp_ws/src/tamp_dev")
        sys.exit(1)

    print("=" * 50)
    print("  Step 5: Velocity Computation Test")
    print("=" * 50)
    passed = 0
    failed = 0
    port = args.port

    # --- Test 1: Stationary tracker → zero velocity ---
    print("\n[TEST] Stationary tracker → velocity ≈ 0...", end=" ")
    vive_input = ViveNetworkInput(port=port, deadzone=0.001)
    vive_input.setup()

    hz = 50.0
    dt = 1.0 / hz
    t_base = time.time()

    # Send stationary packets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(20):
        pkt = _make_packet([0.5, 1.0, 0.3], t=t_base + i * dt)
        sock.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port))
        time.sleep(0.005)

    time.sleep(0.05)
    cmd = vive_input.get_command()
    vel_mag = np.linalg.norm(cmd.velocity)
    vive_input.cleanup()

    if vel_mag < 0.01:
        print(f"[PASS] (|vel|={vel_mag:.4f})")
        passed += 1
    else:
        print(f"[FAIL] (|vel|={vel_mag:.4f}, expected ≈0)")
        failed += 1

    # --- Test 2: Linear motion → correct velocity ---
    print("[TEST] Linear motion X (0.1 m/s) → vx ≈ 0.1...", end=" ")
    port2 = port + 1
    vive_input2 = ViveNetworkInput(port=port2, deadzone=0.001)
    vive_input2.setup()

    t_base = time.time()
    speed = 0.1  # m/s in X (SteamVR X → Robot X with default mapping)
    # Default calibration: SteamVR X → Robot X, Y → Z, Z → -Y
    for i in range(30):
        x = speed * i * dt
        pkt = _make_packet([x, 1.0, 0.3], t=t_base + i * dt)
        sock.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port2))
        time.sleep(0.005)

    time.sleep(0.05)
    cmd2 = vive_input2.get_command()
    vive_input2.cleanup()

    # With default axis mapping: vive X → robot X
    vx = cmd2.velocity[0]
    if abs(vx - speed) < 0.05:
        print(f"[PASS] (vx={vx:.4f})")
        passed += 1
    else:
        print(f"[FAIL] (vx={vx:.4f}, expected ≈{speed})")
        print(f"       Full velocity: {cmd2.velocity}")
        failed += 1

    # --- Test 3: Tracking lost → velocity = 0 ---
    print("[TEST] Tracking lost → velocity = 0...", end=" ")
    port3 = port + 2
    vive_input3 = ViveNetworkInput(port=port3, deadzone=0.001)
    vive_input3.setup()

    t_base = time.time()
    # Send some valid, then lost
    for i in range(10):
        pkt = _make_packet([0.5, 1.0, 0.3], t=t_base + i * dt)
        sock.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port3))
        time.sleep(0.005)

    # Now send tracking=False
    pkt_lost = _make_packet([0.5, 1.0, 0.3], tracking=False, t=t_base + 10 * dt)
    sock.sendto(json.dumps(pkt_lost).encode(), ("127.0.0.1", port3))
    time.sleep(0.05)

    cmd3 = vive_input3.get_command()
    vive_input3.cleanup()

    vel_mag3 = np.linalg.norm(cmd3.velocity)
    if vel_mag3 < 1e-6:
        print(f"[PASS] (|vel|={vel_mag3:.6f})")
        passed += 1
    else:
        print(f"[FAIL] (|vel|={vel_mag3:.6f}, expected 0)")
        failed += 1

    # --- Test 4: E-stop flag ---
    print("[TEST] E-stop button → estop=True...", end=" ")
    port4 = port + 3
    vive_input4 = ViveNetworkInput(port=port4)
    vive_input4.setup()

    pkt_estop = _make_packet([0.5, 1.0, 0.3], t=time.time())
    pkt_estop["buttons"]["estop"] = True
    sock.sendto(json.dumps(pkt_estop).encode(), ("127.0.0.1", port4))
    time.sleep(0.05)

    cmd4 = vive_input4.get_command()
    vive_input4.cleanup()

    if cmd4.estop:
        print("[PASS]")
        passed += 1
    else:
        print("[FAIL] estop flag not set")
        failed += 1

    sock.close()

    # Summary
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 5 complete — proceed to Step 6")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 50}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
