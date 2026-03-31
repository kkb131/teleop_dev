#!/usr/bin/env python3
"""RTDE communication diagnostic for UR10e impedance torque control.

Tests each communication capability individually.
Run on a machine connected to the robot:

  python3 -m robot.arm.impedance.test_rtde_comm --robot-ip 192.168.0.2

Results are printed as PASS/FAIL for each test.
"""

import argparse
import socket
import time
from pathlib import Path

import numpy as np

_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"
_UR_SECONDARY_PORT = 30002


def test_recv_interface(ip: str) -> bool:
    """Test 1: RTDEReceiveInterface - read robot state."""
    print("\n=== TEST 1: RTDEReceiveInterface (robot state) ===")
    try:
        import rtde_receive
        recv = rtde_receive.RTDEReceiveInterface(ip)
        q = list(recv.getActualQ())
        qd = list(recv.getActualQd())
        tcp = list(recv.getActualTCPPose())
        print(f"  q  = [{', '.join(f'{v:.4f}' for v in q)}]")
        print(f"  qd = [{', '.join(f'{v:.4f}' for v in qd)}]")
        print(f"  tcp= [{', '.join(f'{v:.4f}' for v in tcp)}]")
        recv.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_io_lower(ip: str) -> bool:
    """Test 2: RTDEIOInterface lower range - double + int register writes."""
    print("\n=== TEST 2: RTDEIOInterface (lower range, double+int) ===")
    try:
        import rtde_io
        io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)

        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        print("  double regs 18-22: write OK")

        io.setInputIntRegister(18, 0)
        io.setInputIntRegister(19, 0)
        print("  int regs 18-19: write OK")

        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_io_upper(ip: str) -> bool:
    """Test 3: RTDEIOInterface upper range."""
    print("\n=== TEST 3: RTDEIOInterface (upper range) ===")
    try:
        import rtde_io
        io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=True)
        io.setInputDoubleRegister(42, 0.0)
        print("  double reg 42: write OK")
        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_socket_upload(ip: str) -> bool:
    """Test 4: URScript upload via TCP socket (port 30002)."""
    print(f"\n=== TEST 4: URScript upload via socket (port {_UR_SECONDARY_PORT}) ===")
    try:
        # Send a minimal test script
        test_script = 'def test_prog():\n  textmsg("socket_upload_test")\n  sync()\nend\ntest_prog()\n'
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((ip, _UR_SECONDARY_PORT))
        sock.sendall(test_script.encode("utf-8") + b"\n")
        sock.close()
        print("  minimal script sent OK")

        time.sleep(1.0)

        # Send the actual torque relay script
        if _SCRIPT_PATH.exists():
            script_text = _SCRIPT_PATH.read_text()
            sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock2.settimeout(5.0)
            sock2.connect((ip, _UR_SECONDARY_PORT))
            sock2.sendall(script_text.encode("utf-8") + b"\n")
            sock2.close()
            print(f"  {_SCRIPT_PATH.name} sent OK ({len(script_text)} bytes)")
        else:
            print(f"  SKIP: {_SCRIPT_PATH} not found")

        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_pinocchio_coriolis(ip: str) -> bool:
    """Test 5: Pinocchio Coriolis computation with live robot state."""
    print("\n=== TEST 5: Pinocchio Coriolis (local dynamics) ===")
    try:
        import pinocchio as pin
        import rtde_receive
        from robot.config import URDF_PATH

        model = pin.buildModelFromUrdf(URDF_PATH)
        data = model.createData()
        print(f"  Pinocchio model: {model.nq} joints")

        recv = rtde_receive.RTDEReceiveInterface(ip)
        q = np.array(recv.getActualQ())
        qd = np.array(recv.getActualQd())
        recv.disconnect()

        pin.computeCoriolisMatrix(model, data, q, qd)
        coriolis = data.C @ qd
        print(f"  coriolis = [{', '.join(f'{v:.4f}' for v in coriolis)}]")

        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_full_pipeline(ip: str) -> bool:
    """Test 6: Full pipeline - recv + io + socket upload + register write."""
    print(f"\n=== TEST 6: Full pipeline (recv + io + socket + regs) ===")
    recv = None
    io = None
    try:
        import rtde_receive
        import rtde_io as rtde_io_mod

        # Step 1: recv
        recv = rtde_receive.RTDEReceiveInterface(ip)
        q = list(recv.getActualQ())
        print(f"  recv: OK (q[0]={q[0]:.4f})")

        # Step 2: io (lower range)
        io = rtde_io_mod.RTDEIOInterface(ip, use_upper_range_registers=False)
        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        io.setInputIntRegister(18, 0)
        io.setInputIntRegister(19, 0)  # mode=idle
        print("  io: OK (regs zeroed)")

        # Step 3: upload script via socket
        if _SCRIPT_PATH.exists():
            script_text = _SCRIPT_PATH.read_text()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ip, _UR_SECONDARY_PORT))
            sock.sendall(script_text.encode("utf-8") + b"\n")
            sock.close()
            print(f"  script upload via socket: OK")
        else:
            print(f"  script upload: SKIP (file not found)")

        time.sleep(1.0)

        # Step 4: activate mode
        io.setInputIntRegister(19, 1)  # mode=active
        print("  mode=active: OK")

        # Step 5: write test torques (all zero)
        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        io.setInputIntRegister(18, 0)  # tau5=0
        print("  zero torque write: OK")

        time.sleep(2.0)

        # Step 6: stop
        io.setInputIntRegister(19, -1)  # mode=stop
        print("  mode=stop: OK")

        time.sleep(0.2)
        recv.disconnect()
        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL at step: {e}")
        if recv:
            try:
                recv.disconnect()
            except Exception:
                pass
        if io:
            try:
                io.disconnect()
            except Exception:
                pass
        return False


def main():
    parser = argparse.ArgumentParser(description="RTDE communication diagnostic")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--test", type=int, default=0,
                        help="Run specific test (1-6), 0=all")
    args = parser.parse_args()

    tests = [
        (1, test_recv_interface),
        (2, test_io_lower),
        (3, test_io_upper),
        (4, test_socket_upload),
        (5, test_pinocchio_coriolis),
        (6, test_full_pipeline),
    ]

    results = {}
    for num, fn in tests:
        if args.test > 0 and num != args.test:
            continue
        try:
            results[num] = fn(args.robot_ip)
        except Exception as e:
            print(f"  UNEXPECTED ERROR: {e}")
            results[num] = False

    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    for num, fn in tests:
        if num in results:
            status = "PASS" if results[num] else "FAIL"
            print(f"  Test {num}: {status}  {fn.__doc__.strip().split(chr(10))[0]}")

    failed = [n for n, r in results.items() if not r]
    if failed:
        print(f"\nFailed tests: {failed}")
    else:
        print("\nAll tests passed!")


if __name__ == "__main__":
    main()
