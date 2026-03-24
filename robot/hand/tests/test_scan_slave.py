#!/usr/bin/env python3
"""Scan Modbus slave IDs to find the DG5F hand's unit ID.

Tries read_input_registers on slave IDs 0-247 and reports which ones respond.

Usage:
    python3 -m tesollo.tests.test_scan_slave --ip 169.254.186.72
    python3 -m tesollo.tests.test_scan_slave --ip 169.254.186.72 --start 0 --end 10
"""

import argparse
import socket
import sys

try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusIOException
except ImportError:
    print("pymodbus required: pip install 'pymodbus>=3.10,<4'")
    sys.exit(1)


def _check_tcp_port(ip, port, timeout=2.0):
    """Raw TCP connection test to verify Modbus server is listening."""
    try:
        sock = socket.create_connection((ip, port), timeout=timeout)
        sock.close()
        return True
    except (OSError, socket.timeout):
        return False


def main():
    parser = argparse.ArgumentParser(description="Scan Modbus slave IDs for DG5F")
    parser.add_argument("--ip", default="169.254.186.72", help="DG5F IP address")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP port")
    parser.add_argument("--start", type=int, default=0, help="Start slave ID (default: 0)")
    parser.add_argument("--end", type=int, default=247, help="End slave ID (default: 247)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout per ID (seconds)")
    args = parser.parse_args()

    # ── Step 1: Raw TCP check ──────────────────────────
    print(f"[DIAG] Raw TCP test to {args.ip}:{args.port}...", end=" ")
    if _check_tcp_port(args.ip, args.port):
        print("OK (port open)")
    else:
        print("FAIL (port closed or unreachable)")
        print("  → Check: hand power, Ethernet cable, IP config")
        sys.exit(1)

    # ── Step 2: Modbus TCP connect ─────────────────────
    import pymodbus
    print(f"[DIAG] pymodbus version: {pymodbus.__version__}")
    client = ModbusTcpClient(host=args.ip, port=args.port, timeout=args.timeout)
    connect_result = client.connect()
    connected_attr = getattr(client, 'connected', 'N/A')
    print(f"[DIAG] connect() returned: {connect_result}, client.connected: {connected_attr}")
    if not connect_result and not connected_attr:
        print(f"[FAIL] Modbus TCP connect failed to {args.ip}:{args.port}")
        print("  → Raw TCP works but pymodbus connect failed.")
        print("  → Try: pip install --upgrade pymodbus")
        sys.exit(1)
    print(f"[DIAG] Modbus TCP connected to {args.ip}:{args.port}")

    # ── Step 3: Quick test with default ID=1 ───────────
    print(f"[DIAG] Quick test with device_id=1...", end=" ")
    try:
        result = client.read_input_registers(0, count=1, device_id=1)
        if hasattr(result, 'registers'):
            print(f"OK (reg[0]={result.registers[0]})")
            print(f"\n  → device_id=1 works! No full scan needed.")
            client.close()
            return
        else:
            print(f"error response: {result}")
    except ModbusIOException as e:
        print(f"no response ({e})")
    except Exception as e:
        print(f"unexpected error: {type(e).__name__}: {e}")

    # ── Step 4: Full scan ──────────────────────────────
    print(f"\nScanning slave IDs {args.start}-{args.end}...")
    print(f"Timeout per ID: {args.timeout}s")
    print("-" * 50)

    found = []
    for sid in range(args.start, args.end + 1):
        try:
            result = client.read_input_registers(0, count=1, device_id=sid)
            if hasattr(result, 'registers'):
                print(f"  [HIT] device_id = {sid}  (reg[0] = {result.registers[0]})")
                found.append(sid)
            elif sid % 50 == 0:
                print(f"  ... scanning ID {sid} (error response)...")
        except ModbusIOException:
            if sid % 50 == 0:
                print(f"  ... scanning ID {sid} (no response)...")
        except Exception as e:
            if sid % 50 == 0:
                print(f"  ... scanning ID {sid} ({type(e).__name__})...")

    client.close()

    print("-" * 50)
    if found:
        print(f"Found {len(found)} responding device ID(s): {found}")
        print(f"\nUse:  --slave-id {found[0]}")
    else:
        print("No responding device IDs found.")
        print("\nDiagnostics:")
        print("  ✓ TCP port 502 is open (connection succeeded)")
        print("  ✓ Modbus TCP client connected")
        print("  ✗ No Modbus protocol response from any device ID")
        print("\nPossible causes:")
        print("  1. Hand firmware not fully booted (wait 10-15s after power on)")
        print("  2. Hand in error/fault state (check LED indicators)")
        print("  3. Modbus TCP server not enabled on the hand")
        print("  4. Firewall blocking Modbus responses")


if __name__ == "__main__":
    main()
