#!/usr/bin/env python3
"""Isolated diagnostic tests for URScript torque relay.

Runs 5 independent tests (A-E), each testing one specific component.
Results are printed to Python terminal AND visible on UR teach pendant log.

Test A: Script upload verification (no RTDE, no torque)
Test B: RTDE register read echo (no torque)
Test C: direct_torque with RTDE connected (no thread, no register read)
Test D: Threaded direct_torque (no RTDE)
Test E: Thread torque + register read (minimal reproduction of our architecture)

Usage:
  python3 -m standalone.teleop_impedance.test_torque_diag --robot-ip 192.168.0.2
  python3 -m standalone.teleop_impedance.test_torque_diag --robot-ip 192.168.0.2 --test B
  python3 -m standalone.teleop_impedance.test_torque_diag --robot-ip 192.168.0.2 --test E
"""

import argparse
import socket
import time

_UR_SECONDARY_PORT = 30002


def _upload_script(ip: str, script_text: str, label: str):
    """Upload URScript via TCP socket to UR Secondary Interface."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((ip, _UR_SECONDARY_PORT))
        sock.sendall(script_text.encode("utf-8") + b"\n")
        print(f"  [Upload] {label} sent OK ({len(script_text)} bytes)")
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Test A: Script Upload Verification
# ---------------------------------------------------------------------------
def test_a(ip: str):
    """Test A: Verify script upload works (unique timestamp)."""
    ts = int(time.time())
    script = f"""\
def diag_a():
  textmsg("DIAG_A: upload_ok ts={ts}")
  sync()
  sync()
  textmsg("DIAG_A: done")
end
diag_a()
"""
    print(f"\n{'='*60}")
    print(f"TEST A: Script Upload Verification  (ts={ts})")
    print(f"{'='*60}")
    print(f"  Expected on UR panel: DIAG_A: upload_ok ts={ts}")

    _upload_script(ip, script, "diag_a")
    print("  Waiting 3s for script execution...")
    time.sleep(3.0)
    print("  TEST A: Check UR panel for DIAG_A messages")


# ---------------------------------------------------------------------------
# Test B: RTDE Register Read Echo
# ---------------------------------------------------------------------------
def test_b(ip: str):
    """Test B: Verify RTDE register values are readable from URScript."""
    script = """\
def diag_b():
  textmsg("DIAG_B: start")
  local i = 0
  while i < 10:
    local mode = read_input_integer_register(19)
    local tau5 = read_input_integer_register(18)
    local tau0 = read_input_float_register(18)
    textmsg(str_cat(str_cat(str_cat(str_cat(str_cat("DIAG_B: mode=", to_str(mode)), " tau5="), to_str(tau5)), " tau0="), to_str(tau0)))
    i = i + 1
    sync()
  end
  textmsg("DIAG_B: done")
end
diag_b()
"""
    print(f"\n{'='*60}")
    print("TEST B: RTDE Register Read Echo")
    print(f"{'='*60}")

    import rtde_io
    io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)
    print("  RTDEIOInterface connected (lower range)")

    # Write known test values
    io.setInputDoubleRegister(18, 1.11)
    io.setInputIntRegister(18, 42)
    io.setInputIntRegister(19, 0)
    print("  Registers set: double18=1.11, int18=42, int19=0")
    print("  Expected on UR panel: DIAG_B: mode=0 tau5=42 tau0=1.11...")

    time.sleep(0.5)  # propagation
    _upload_script(ip, script, "diag_b")
    print("  Waiting 5s for script execution...")
    time.sleep(5.0)

    io.disconnect()
    print("  RTDEIOInterface disconnected")
    print("  TEST B: Check UR panel for DIAG_B messages")


# ---------------------------------------------------------------------------
# Test C: direct_torque with RTDE Connected
# ---------------------------------------------------------------------------
def test_c(ip: str):
    """Test C: Verify direct_torque works with RTDE interfaces connected."""
    script = """\
def diag_c():
  textmsg("DIAG_C: direct_torque with RTDE connected")
  local zero_tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  local i = 0
  while i < 2500:
    direct_torque(zero_tau)
    i = i + 1
  end
  textmsg("DIAG_C: done (5s)")
  stopj(4.0)
end
diag_c()
"""
    print(f"\n{'='*60}")
    print("TEST C: direct_torque + RTDE Connected")
    print(f"{'='*60}")

    import rtde_receive
    import rtde_io
    recv = rtde_receive.RTDEReceiveInterface(ip)
    io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)
    print("  RTDEReceiveInterface + RTDEIOInterface connected")

    q = list(recv.getActualQ())
    print(f"  q[0]={q[0]:.4f} (state read OK)")
    print("  Expected on UR panel: DIAG_C: done (5s)")

    _upload_script(ip, script, "diag_c")
    print("  Waiting 7s for script execution (5s torque + margin)...")
    time.sleep(7.0)

    recv.disconnect()
    io.disconnect()
    print("  Interfaces disconnected")
    print("  TEST C: Check UR panel for DIAG_C messages")


# ---------------------------------------------------------------------------
# Test D: Threaded direct_torque (no RTDE)
# ---------------------------------------------------------------------------
def test_d(ip: str):
    """Test D: Verify direct_torque works from a URScript thread."""
    script = """\
def diag_d():
  textmsg("DIAG_D: threaded direct_torque (no RTDE regs)")
  global diag_running = True

  thread torqueLoop():
    local zero_tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    while diag_running:
      direct_torque(zero_tau)
    end
  end

  local thrd = run torqueLoop()
  textmsg("DIAG_D: thread started")

  local count = 0
  while count < 2500:
    count = count + 1
    if count % 500 == 0:
      textmsg(str_cat("DIAG_D: alive count=", to_str(count)))
    end
    sync()
  end

  diag_running = False
  join thrd
  textmsg("DIAG_D: done (5s)")
  stopj(4.0)
end
diag_d()
"""
    print(f"\n{'='*60}")
    print("TEST D: Threaded direct_torque (no RTDE)")
    print(f"{'='*60}")
    print("  Expected on UR panel: DIAG_D: alive count=500, 1000, ..., 2500, done (5s)")

    _upload_script(ip, script, "diag_d")
    print("  Waiting 7s for script execution...")
    time.sleep(7.0)
    print("  TEST D: Check UR panel for DIAG_D messages")


# ---------------------------------------------------------------------------
# Test E: Thread Torque + Register Read (Minimal Reproduction)
# ---------------------------------------------------------------------------
def test_e(ip: str):
    """Test E: Minimal reproduction of our architecture (thread torque + register read)."""
    script = """\
def diag_e():
  textmsg("DIAG_E: thread torque + register read")
  global diag_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  global diag_active = False
  global diag_running = True

  thread torqueLoop():
    local zero_tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    while diag_running:
      if diag_active:
        direct_torque(diag_cmd)
      else:
        direct_torque(zero_tau)
      end
    end
  end

  local thrd = run torqueLoop()
  textmsg("DIAG_E: thread started")

  local count = 0
  while True:
    local mode = read_input_integer_register(19)

    if mode == -1:
      textmsg("DIAG_E: got mode=-1, exiting")
      break
    end

    if mode > 0:
      diag_active = True
    else:
      diag_active = False
    end

    count = count + 1
    if count % 500 == 0:
      textmsg(str_cat(str_cat(str_cat("DIAG_E: alive mode=", to_str(mode)), " count="), to_str(count)))
    end

    sync()
  end

  diag_running = False
  join thrd
  textmsg("DIAG_E: stopping")
  stopj(4.0)
  textmsg("DIAG_E: done")
end
diag_e()
"""
    print(f"\n{'='*60}")
    print("TEST E: Thread Torque + Register Read (Minimal Reproduction)")
    print(f"{'='*60}")

    import rtde_io
    io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)
    print("  RTDEIOInterface connected (lower range)")

    # Initialize registers
    for i in range(18, 23):
        io.setInputDoubleRegister(i, 0.0)
    io.setInputIntRegister(18, 0)
    io.setInputIntRegister(19, 0)  # mode=idle
    print("  Registers initialized (all zero, mode=0)")
    print("  Expected on UR panel: DIAG_E: alive mode=0 count=500, 1000, ...")

    time.sleep(0.5)  # propagation
    _upload_script(ip, script, "diag_e")

    print("  Waiting 2s for script startup...")
    time.sleep(2.0)

    print("  Keeping mode=0 for 5s (script should keep running)...")
    for sec in range(5):
        io.setInputIntRegister(19, 0)  # keep mode=idle
        time.sleep(1.0)
        print(f"    {sec+1}/5s...")

    print("  Sending mode=-1 (stop signal)...")
    io.setInputIntRegister(19, -1)
    time.sleep(2.0)

    io.disconnect()
    print("  RTDEIOInterface disconnected")
    print("  TEST E: Check UR panel for DIAG_E messages")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="URScript torque relay diagnostic")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--test", default="ALL",
                        choices=["A", "B", "C", "D", "E", "ALL"],
                        help="Run specific test or ALL (default)")
    args = parser.parse_args()

    tests = {
        "A": test_a,
        "B": test_b,
        "C": test_c,
        "D": test_d,
        "E": test_e,
    }

    if args.test == "ALL":
        run_tests = ["A", "B", "C", "D", "E"]
    else:
        run_tests = [args.test]

    print(f"\nDiagnostic Tests: {', '.join(run_tests)}")
    print(f"Robot IP: {args.robot_ip}")
    print("=" * 60)
    print("CHECK UR TEACH PENDANT LOG TAB FOR DIAG_* MESSAGES")
    print("=" * 60)

    for name in run_tests:
        try:
            tests[name](args.robot_ip)
        except Exception as e:
            print(f"\n  TEST {name} FAILED WITH EXCEPTION: {e}")
            import traceback
            traceback.print_exc()

        if name != run_tests[-1]:
            print("\n  --- Waiting 3s before next test ---")
            time.sleep(3.0)

    print(f"\n{'='*60}")
    print("ALL TESTS COMPLETE")
    print("Check UR teach pendant log for results:")
    for name in run_tests:
        print(f"  - DIAG_{name}: ...")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
