#!/usr/bin/env python3
"""Manus Quantum Metagloves reader — subprocess wrapper.

Launches the SDKClient_Linux C++ binary with --stream-json flag
and reads newline-delimited JSON from its stdout. This avoids
all ctypes/callback issues by delegating SDK interaction to the
proven C++ example code.

Usage:
    python3 -m sender.hand.manus_reader [--hand right] [--sdk-bin sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out]
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


# ─────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES_PER_FINGER = {
    "Thumb": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Index": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Middle": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Ring": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Pinky": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
}

NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

DEFAULT_SDK_BIN = "sender/hand/sdk/SDKClient_Linux/SDKClient_Linux.out"

# Strip ANSI escape sequences (e.g. \x1b[0;1H cursor moves from C++ binary)
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


# ─────────────────────────────────────────────────────────
# Data output (interface unchanged from ctypes version)
# ─────────────────────────────────────────────────────────

@dataclass
class HandData:
    """Hand tracking data from a single Manus glove."""
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    finger_spread: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    wrist_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wrist_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    hand_side: str = "right"
    timestamp: float = 0.0


class ManusReader:
    """Reads hand data from Manus gloves via SDKClient_Linux subprocess.

    The C++ binary handles all SDK interaction (initialization, callbacks,
    coordinate system, connection). This Python class simply reads JSON
    lines from stdout.

    Parameters
    ----------
    sdk_bin_path : str
        Path to SDKClient_Linux.out binary.
    hand_side : str
        "left", "right", or "both"
    """

    def __init__(self, sdk_bin_path: str = DEFAULT_SDK_BIN,
                 hand_side: str = "right", verbose: bool = False):
        self._bin_path = sdk_bin_path
        self._hand_side = hand_side
        self._verbose = verbose
        self._proc: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._connected = False

        # Data cache (thread-safe)
        self._lock = threading.Lock()
        self._left_data: Optional[HandData] = None
        self._right_data: Optional[HandData] = None
        self._data_received = threading.Event()
        self._line_count = 0
        self._error_count = 0
        self._left_count = 0
        self._right_count = 0
        self._debug_info: Optional[dict] = None

    def connect(self):
        """Launch C++ binary and start reading JSON from stdout."""
        bin_path = Path(self._bin_path)
        if not bin_path.exists():
            raise FileNotFoundError(
                f"SDK binary not found: {bin_path.resolve()}\n"
                f"Build it first: cd manus/sdk && bash build.sh"
            )

        # cwd MUST be the directory containing the binary (rpath = ./ManusSDK/lib)
        bin_resolved = bin_path.resolve()
        cwd = bin_resolved.parent
        sdk_lib_dir = cwd / "ManusSDK" / "lib"

        # Set LD_LIBRARY_PATH to include ManusSDK/lib
        env = dict(os.environ)
        ld_path = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{sdk_lib_dir}:{ld_path}" if ld_path else str(sdk_lib_dir)

        print(f"[ManusReader] Launching: {bin_resolved} --stream-json")
        print(f"[ManusReader] Working dir: {cwd}")
        print(f"[ManusReader] LD_LIBRARY_PATH includes: {sdk_lib_dir}")

        self._proc = subprocess.Popen(
            [str(bin_resolved), "--stream-json"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(cwd),
            env=env,
        )

        self._connected = True

        # Start reader thread (stdout JSON)
        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._reader_thread.start()

        # Start stderr reader thread (for debugging)
        self._stderr_thread = threading.Thread(
            target=self._read_stderr, daemon=True
        )
        self._stderr_thread.start()

        # Wait for first data
        print("[ManusReader] Waiting for data...", end=" ", flush=True)
        if self._data_received.wait(timeout=10.0):
            print(f"OK (received {self._line_count} lines)")
        else:
            print("TIMEOUT")
            # Check if process died
            if self._proc.poll() is not None:
                stderr = self._proc.stderr.read().decode(errors="replace")
                raise RuntimeError(
                    f"SDK binary exited with code {self._proc.returncode}\n"
                    f"stderr: {stderr[:500]}"
                )

    def disconnect(self):
        """Stop the C++ binary."""
        self._connected = False
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
            print("[ManusReader] Process stopped")

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        """Read latest hand data from cache.

        Returns HandData with joint_angles in RADIANS (converted from SDK degrees).
        """
        if not self._connected:
            return None

        target_side = side or self._hand_side

        with self._lock:
            if target_side == "left":
                return self._left_data
            else:
                return self._right_data

    def get_both_hands(self) -> dict[str, Optional[HandData]]:
        return {"left": self.get_hand_data("left"),
                "right": self.get_hand_data("right")}

    def get_status(self) -> dict:
        proc_alive = self._proc is not None and self._proc.poll() is None
        return {
            "sdk_loaded": True,
            "connected": self._connected and proc_alive,
            "hand_side": self._hand_side,
            "bin_path": self._bin_path,
            "lines_received": self._line_count,
            "left_count": self._left_count,
            "right_count": self._right_count,
            "errors": self._error_count,
            "proc_alive": proc_alive,
        }

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        return self._data_received.wait(timeout=timeout)

    def get_debug_info(self) -> Optional[dict]:
        with self._lock:
            return self._debug_info

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── Private ──────────────────────────────────────

    def _read_stderr(self):
        """Read and print C++ binary stderr for debugging."""
        try:
            while self._connected:
                raw_line = self._proc.stderr.readline()
                if not raw_line:
                    break
                line = raw_line.decode(errors="replace").rstrip()
                if line and self._verbose:
                    print(f"[SDK] {line}", flush=True)
        except Exception:
            pass

    def _read_loop(self):
        """Read JSON lines from C++ binary stdout."""
        try:
            while self._connected:
                raw_line = self._proc.stdout.readline()
                if not raw_line:
                    break
                line = _ANSI_RE.sub('', raw_line.decode(errors="replace")).strip()

                if not line or not line.startswith("{"):
                    continue

                try:
                    pkt = json.loads(line)
                except json.JSONDecodeError:
                    self._error_count += 1
                    continue

                if pkt.get("type") == "debug":
                    with self._lock:
                        self._debug_info = pkt
                    continue

                if pkt.get("type") != "manus":
                    continue

                hand_side = pkt.get("hand", "right")
                angles_deg = pkt.get("joint_angles", [])
                spread_deg = pkt.get("finger_spread", [])
                timestamp = pkt.get("timestamp", time.time())

                if len(angles_deg) != NUM_JOINTS:
                    self._error_count += 1
                    continue

                # Convert degrees → radians
                joint_angles = np.deg2rad(
                    np.array(angles_deg, dtype=np.float32)
                )
                finger_spread = np.deg2rad(
                    np.array(spread_deg[:NUM_FINGERS], dtype=np.float32)
                ) if len(spread_deg) >= NUM_FINGERS else np.zeros(NUM_FINGERS)

                hd = HandData(
                    joint_angles=joint_angles,
                    finger_spread=finger_spread,
                    wrist_pos=np.array(pkt.get("wrist_pos", [0, 0, 0]),
                                       dtype=np.float32),
                    wrist_quat=np.array(pkt.get("wrist_quat", [1, 0, 0, 0]),
                                        dtype=np.float32),
                    hand_side=hand_side,
                    timestamp=timestamp,
                )

                with self._lock:
                    if hand_side == "left":
                        self._left_data = hd
                        self._left_count += 1
                    else:
                        self._right_data = hd
                        self._right_count += 1

                self._line_count += 1
                self._data_received.set()

        except Exception as e:
            if self._connected:
                print(f"[ManusReader] Reader thread error: {e}",
                      file=sys.stderr)


# ─────────────────────────────────────────────────────────
# Standalone test
# ─────────────────────────────────────────────────────────

def _print_hand_data(data: HandData) -> str:
    lines = [f"  Hand: {data.hand_side.upper()}  "
             f"(t={data.timestamp:.1f})"]
    for f_idx, fname in enumerate(FINGER_NAMES):
        joints = data.joint_angles[f_idx * JOINTS_PER_FINGER:
                                   (f_idx + 1) * JOINTS_PER_FINGER]
        jnames = JOINT_NAMES_PER_FINGER[fname]
        vals = "  ".join(f"{jnames[j]}={joints[j]:+7.3f}"
                         for j in range(JOINTS_PER_FINGER))
        lines.append(f"  {fname:7s}: {vals}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Manus Quantum Metagloves reader (subprocess mode)")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"])
    parser.add_argument("--sdk-bin", default=DEFAULT_SDK_BIN,
                        help="Path to SDKClient_Linux.out")
    parser.add_argument("--hz", type=int, default=30)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--debug", action="store_true",
                        help="Debug mode: show only SDK debug info, no live display")
    args = parser.parse_args()

    print("=" * 65)
    print("  Manus Quantum Metagloves — Reader (subprocess mode)")
    print("=" * 65)

    with ManusReader(sdk_bin_path=args.sdk_bin, hand_side=args.hand,
                     verbose=args.debug) as reader:
        status = reader.get_status()
        print(f"  Connected: {status['connected']}")
        print(f"  Lines received: {status['lines_received']}")

        # Wait briefly for debug info from C++ and display it
        time.sleep(1.0)
        dbg = reader.get_debug_info()
        if dbg:
            print(f"  SDK Debug: L_gloveID={dbg.get('L_gloveID', '?')} "
                  f"R_gloveID={dbg.get('R_gloveID', '?')} "
                  f"L_ergoID={dbg.get('L_ergoID', '?')} "
                  f"R_ergoID={dbg.get('R_ergoID', '?')} "
                  f"landscape={dbg.get('landscape', '?')}")
        else:
            print("  SDK Debug: (no debug info received)")

        print("-" * 65)
        print("  Press Ctrl+C to stop.\n")

        dt = 1.0 / args.hz
        count = 0
        start_time = time.time()
        null_count = 0
        num_lines = NUM_FINGERS + 1

        if args.debug:
            # Debug mode: no ANSI cursor, just print debug info periodically
            try:
                while True:
                    time.sleep(1.0)
                    dbg = reader.get_debug_info()
                    s = reader.get_status()
                    if dbg:
                        print(f"  [DEBUG] frame={dbg.get('frame', '?')} "
                              f"L_gloveID={dbg.get('L_gloveID', 0)} "
                              f"R_gloveID={dbg.get('R_gloveID', 0)} "
                              f"L_ergoID={dbg.get('L_ergoID', 0)} "
                              f"R_ergoID={dbg.get('R_ergoID', 0)} "
                              f"landscape={dbg.get('landscape', '?')} "
                              f"L={s['left_count']} R={s['right_count']} "
                              f"err={s['errors']}")
                    else:
                        print(f"  [DEBUG] (no debug JSON) "
                              f"L={s['left_count']} R={s['right_count']} "
                              f"err={s['errors']}")

                    if args.duration > 0 and time.time() - start_time >= args.duration:
                        break
            except KeyboardInterrupt:
                pass
        else:
            # Normal mode: live display with ANSI cursor
            try:
                while True:
                    t_loop = time.perf_counter()

                    if args.hand == "both":
                        for side in ("left", "right"):
                            data = reader.get_hand_data(side)
                            if data is not None:
                                if count > 0:
                                    print(f"\033[{num_lines * 2 + 1}A", end="")
                                print(_print_hand_data(data))
                            else:
                                null_count += 1
                    else:
                        data = reader.get_hand_data()
                        if data is not None:
                            if count > 0:
                                print(f"\033[{num_lines}A", end="")
                            print(_print_hand_data(data))
                        else:
                            null_count += 1
                            if null_count == 1 or null_count % 60 == 0:
                                s = reader.get_status()
                                print(f"\r  [NO DATA] null={null_count} "
                                      f"lines={s['lines_received']} "
                                      f"err={s['errors']}    ",
                                      end="", flush=True)

                    count += 1

                    if args.duration > 0 and time.time() - start_time >= args.duration:
                        break

                    remaining = dt - (time.perf_counter() - t_loop)
                    if remaining > 0:
                        time.sleep(remaining)

            except KeyboardInterrupt:
                pass

        elapsed = time.time() - start_time
        hz = count / elapsed if elapsed > 0 else 0
        s = reader.get_status()
        print(f"\n{'=' * 65}")
        print(f"  Frames: {count}, Duration: {elapsed:.1f}s, Rate: {hz:.1f}Hz")
        print(f"  Lines from SDK: {s['lines_received']}, Errors: {s['errors']}")
        print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
