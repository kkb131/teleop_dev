#!/usr/bin/env python3
"""Vive Tracker real-time monitor — curses-based terminal dashboard.

Receives UDP packets from vive_sender.py and displays live tracking data
with color-coded status, position/orientation, buttons, network stats,
and a mini position history graph.

Usage:
    python3 -m operator.arm.monitor --port 9871
    python3 -m operator.arm.monitor --port 9871 --bind 0.0.0.0
"""

import argparse
import collections
import curses
import json
import math
import socket
import statistics
import threading
import time


# Unicode block characters for mini graph (8 levels)
_BLOCKS = " ▁▂▃▄▅▆▇█"


def _quat_to_euler(w, x, y, z):
    """Convert quaternion (wxyz) to euler angles (roll, pitch, yaw) in degrees."""
    # Roll (X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _make_sparkline(values, width=40):
    """Create a unicode sparkline from a list of values."""
    if not values:
        return " " * width

    recent = list(values)[-width:]
    if len(recent) < 2:
        return " " * width

    vmin = min(recent)
    vmax = max(recent)
    vrange = vmax - vmin

    if vrange < 1e-9:
        return _BLOCKS[4] * len(recent) + " " * (width - len(recent))

    chars = []
    for v in recent:
        idx = int((v - vmin) / vrange * 8)
        idx = max(0, min(8, idx))
        chars.append(_BLOCKS[idx])

    line = "".join(chars)
    if len(line) < width:
        line = " " * (width - len(line)) + line
    return line


class ViveMonitor:
    """UDP receiver + curses display for Vive Tracker data."""

    def __init__(self, port: int, bind_addr: str = "0.0.0.0"):
        self._port = port
        self._bind = bind_addr

        # Shared state (written by receiver thread, read by display)
        self._lock = threading.Lock()
        self._last_pkt = None
        self._last_recv_time = 0.0
        self._pkt_count = 0
        self._lost_count = 0
        self._recv_times = collections.deque(maxlen=500)
        self._latencies = collections.deque(maxlen=200)
        self._pos_history_x = collections.deque(maxlen=150)  # ~3s at 50Hz
        self._pos_history_y = collections.deque(maxlen=150)
        self._pos_history_z = collections.deque(maxlen=150)
        self._prev_pos = None
        self._prev_time = None
        self._lin_vel = 0.0
        self._ang_vel = 0.0
        self._prev_quat = None
        self._running = True

    def _receiver_thread(self):
        """Background thread: receive UDP packets."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind, self._port))
        sock.settimeout(0.5)

        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                recv_time = time.time()
                pkt = json.loads(data)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, OSError):
                continue

            with self._lock:
                self._last_pkt = pkt
                self._last_recv_time = recv_time
                self._pkt_count += 1
                self._recv_times.append(recv_time)

                tracking = pkt.get("tracking", False)
                if not tracking:
                    self._lost_count += 1

                # Latency
                ts = pkt.get("timestamp")
                if ts is not None:
                    lat = recv_time - ts
                    if lat > 0:
                        self._latencies.append(lat)

                # Position history + velocity
                pos = pkt.get("pos")
                if pos and tracking:
                    self._pos_history_x.append(pos[0])
                    self._pos_history_y.append(pos[1])
                    self._pos_history_z.append(pos[2])

                    if self._prev_pos is not None and self._prev_time is not None:
                        dt = recv_time - self._prev_time
                        if dt > 0.001:
                            dx = [pos[i] - self._prev_pos[i] for i in range(3)]
                            self._lin_vel = math.sqrt(sum(d * d for d in dx)) / dt

                    self._prev_pos = pos
                    self._prev_time = recv_time

                # Angular velocity estimate
                quat = pkt.get("quat")
                if quat and tracking and self._prev_quat is not None:
                    dt = recv_time - (self._prev_time or recv_time)
                    if dt > 0.001:
                        # Approximate: angle between consecutive quaternions
                        dot = sum(a * b for a, b in zip(quat, self._prev_quat))
                        dot = max(-1.0, min(1.0, dot))
                        angle = 2.0 * math.acos(abs(dot))
                        self._ang_vel = angle / dt
                if quat and tracking:
                    self._prev_quat = quat

        sock.close()

    def run(self, stdscr):
        """Main curses display loop."""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)  # 20 Hz refresh

        # Colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # tracking OK
        curses.init_pair(2, curses.COLOR_RED, -1)      # tracking lost
        curses.init_pair(3, curses.COLOR_YELLOW, -1)   # warning
        curses.init_pair(4, curses.COLOR_CYAN, -1)     # header
        curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_RED)  # button active
        curses.init_pair(6, curses.COLOR_WHITE, -1)    # normal

        # Start receiver
        t = threading.Thread(target=self._receiver_thread, daemon=True)
        t.start()

        W = 54  # box width

        while self._running:
            key = stdscr.getch()
            if key == ord("q") or key == ord("Q") or key == 27:
                self._running = False
                break

            stdscr.erase()
            h, w = stdscr.getmaxyx()
            if h < 30 or w < W + 2:
                stdscr.addstr(0, 0, f"Terminal too small ({w}x{h}). Need {W + 2}x30+")
                stdscr.refresh()
                continue

            with self._lock:
                pkt = dict(self._last_pkt) if self._last_pkt else None
                recv_time = self._last_recv_time
                pkt_count = self._pkt_count
                lost_count = self._lost_count
                recv_times = list(self._recv_times)
                latencies = list(self._latencies)
                hist_x = list(self._pos_history_x)
                hist_y = list(self._pos_history_y)
                hist_z = list(self._pos_history_z)
                lin_vel = self._lin_vel
                ang_vel = self._ang_vel

            row = 0

            def hline(ch="═"):
                return "╠" + ch * (W - 2) + "╣"

            def line(text):
                padded = text[:W - 4].ljust(W - 4)
                return "║ " + padded + " ║"

            # Header
            stdscr.addstr(row, 0, "╔" + "═" * (W - 2) + "╗")
            row += 1
            title = "Vive Tracker Monitor"
            stdscr.addstr(row, 0, "║" + title.center(W - 2) + "║",
                          curses.color_pair(4) | curses.A_BOLD)
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Status
            now = time.time()
            if pkt is None:
                status_str = "● WAITING"
                status_color = curses.color_pair(3)
                hz_str = "---"
            else:
                tracking = pkt.get("tracking", False)
                age = now - recv_time
                if age > 1.0:
                    status_str = "● TIMEOUT"
                    status_color = curses.color_pair(2) | curses.A_BOLD
                elif tracking:
                    status_str = "● TRACKING"
                    status_color = curses.color_pair(1) | curses.A_BOLD
                else:
                    status_str = "● LOST"
                    status_color = curses.color_pair(2) | curses.A_BOLD

                # Hz calculation
                if len(recv_times) > 2:
                    dt_total = recv_times[-1] - recv_times[0]
                    if dt_total > 0:
                        hz_str = f"{(len(recv_times) - 1) / dt_total:.1f}"
                    else:
                        hz_str = "---"
                else:
                    hz_str = "---"

            status_line = f"Status: {status_str}"
            hz_part = f"Hz: {hz_str}"
            combined = status_line.ljust(28) + hz_part
            stdscr.addstr(row, 0, "║ ", curses.color_pair(6))
            stdscr.addstr(row, 2, "Status: ")
            stdscr.addstr(row, 10, status_str, status_color)
            stdscr.addstr(row, 28, hz_part.rjust(W - 30))
            stdscr.addstr(row, W - 1, " ║")
            row += 1

            # Packet count
            track_pct = 100.0 * (1.0 - lost_count / pkt_count) if pkt_count > 0 else 0.0
            pkt_line = f"Packets: {pkt_count}"
            lost_line = f"Lost: {lost_count} ({100.0 - track_pct:.1f}%)"
            stdscr.addstr(row, 0, line(pkt_line.ljust(28) + lost_line))
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Position
            if pkt and pkt.get("tracking"):
                pos = pkt.get("pos", [0, 0, 0])
                stdscr.addstr(row, 0, line("Position (m)"))
                row += 1
                pos_str = f"  X: {pos[0]:+8.4f}   Y: {pos[1]:+8.4f}   Z: {pos[2]:+8.4f}"
                stdscr.addstr(row, 0, line(pos_str))
            else:
                stdscr.addstr(row, 0, line("Position (m)"))
                row += 1
                stdscr.addstr(row, 0, line("  --- no data ---"))
            row += 1
            stdscr.addstr(row, 0, line(""))
            row += 1

            # Orientation - quaternion
            if pkt and pkt.get("tracking"):
                q = pkt.get("quat", [1, 0, 0, 0])
                stdscr.addstr(row, 0, line("Orientation (quat wxyz)"))
                row += 1
                q_str = f"  W:{q[0]:+7.4f}  X:{q[1]:+7.4f}  Y:{q[2]:+7.4f}  Z:{q[3]:+7.4f}"
                stdscr.addstr(row, 0, line(q_str))
                row += 1
                stdscr.addstr(row, 0, line(""))
                row += 1

                # Euler
                roll, pitch, yaw = _quat_to_euler(q[0], q[1], q[2], q[3])
                stdscr.addstr(row, 0, line("Orientation (euler deg)"))
                row += 1
                e_str = f"  Roll:{roll:+7.1f}  Pitch:{pitch:+7.1f}  Yaw:{yaw:+7.1f}"
                stdscr.addstr(row, 0, line(e_str))
            else:
                stdscr.addstr(row, 0, line("Orientation (quat wxyz)"))
                row += 1
                stdscr.addstr(row, 0, line("  --- no data ---"))
                row += 1
                stdscr.addstr(row, 0, line(""))
                row += 1
                stdscr.addstr(row, 0, line("Orientation (euler deg)"))
                row += 1
                stdscr.addstr(row, 0, line("  --- no data ---"))
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Velocity
            stdscr.addstr(row, 0, line("Velocity (estimated)"))
            row += 1
            vel_str = f"  Lin: {lin_vel:.3f} m/s    Ang: {ang_vel:.2f} rad/s"
            stdscr.addstr(row, 0, line(vel_str))
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Buttons
            stdscr.addstr(row, 0, line("Buttons"))
            row += 1
            if pkt:
                btns = pkt.get("buttons", {})
                btn_parts = []
                btn_names = [
                    ("estop", "E-Stop"),
                    ("reset", "Reset"),
                    ("quit", "Quit"),
                    ("speed_up", "Spd+"),
                    ("speed_down", "Spd-"),
                ]

                stdscr.addstr(row, 0, "║ ", curses.color_pair(6))
                col = 3
                for key, label in btn_names:
                    active = btns.get(key, False)
                    text = f" {label} "
                    if active:
                        stdscr.addstr(row, col, f"[{text}]",
                                      curses.color_pair(5) | curses.A_BOLD)
                    else:
                        stdscr.addstr(row, col, f"[{text}]", curses.color_pair(6))
                    col += len(text) + 3
                stdscr.addstr(row, W - 1, " ║")
            else:
                stdscr.addstr(row, 0, line("  --- waiting ---"))
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Network stats
            stdscr.addstr(row, 0, line("Network"))
            row += 1
            if latencies:
                lat_mean = sum(latencies) / len(latencies) * 1000
                lat_max = max(latencies) * 1000
                lat_str = f"  Latency: {lat_mean:.1f}ms (max {lat_max:.1f}ms)"
            else:
                lat_str = "  Latency: --- (clocks not synced?)"
            stdscr.addstr(row, 0, line(lat_str))
            row += 1

            if len(recv_times) > 2:
                intervals = [recv_times[i + 1] - recv_times[i]
                             for i in range(len(recv_times) - 1)]
                jitter = statistics.stdev(intervals) * 1000 if len(intervals) > 1 else 0
                jit_str = f"  Jitter:  σ={jitter:.1f}ms"
            else:
                jit_str = "  Jitter:  ---"
            stdscr.addstr(row, 0, line(jit_str))
            row += 1

            age = now - recv_time if recv_time > 0 else -1
            if age >= 0:
                age_str = f"  Last packet: {age:.2f}s ago"
                age_color = curses.color_pair(1) if age < 0.5 else curses.color_pair(2)
            else:
                age_str = "  Last packet: none"
                age_color = curses.color_pair(3)
            stdscr.addstr(row, 0, line(age_str), age_color)
            row += 1
            stdscr.addstr(row, 0, hline())
            row += 1

            # Position sparklines
            spark_w = W - 8
            stdscr.addstr(row, 0, line("Position History (last 3s)"))
            row += 1
            for label, hist in [("X", hist_x), ("Y", hist_y), ("Z", hist_z)]:
                spark = _make_sparkline(hist, spark_w)
                stdscr.addstr(row, 0, line(f" {label} {spark}"))
                row += 1

            # Footer
            stdscr.addstr(row, 0, "╚" + "═" * (W - 2) + "╝")
            row += 1
            stdscr.addstr(row, 0, f"  Port: {self._port}  |  Press 'q' to quit",
                          curses.color_pair(6) | curses.A_DIM)

            stdscr.refresh()

        t.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description="Vive Tracker real-time monitor")
    parser.add_argument("--port", type=int, default=9871, help="UDP port to listen on")
    parser.add_argument("--bind", default="0.0.0.0", help="Bind address")
    args = parser.parse_args()

    monitor = ViveMonitor(port=args.port, bind_addr=args.bind)
    curses.wrapper(monitor.run)


if __name__ == "__main__":
    main()
