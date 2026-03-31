#!/usr/bin/env python3
"""Real-time hand skeleton visualizer for Manus glove data.

Renders a 2D hand skeleton with per-finger joint angles,
bar charts, and wrist pose info. Supports three input modes:

  --mock   : Animated mock data (no hardware, for testing)
  --sdk    : Live data from Manus glove via ManusReader
  --udp    : Receive UDP packets from manus_sender

Usage:
    # Mock data (no hardware needed):
    python3 -m operator.hand.hand_visualizer

    # Real glove:
    python3 -m operator.hand.hand_visualizer --sdk --sdk-path operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out

    # UDP receiver (run manus_sender on another terminal):
    python3 -m operator.hand.hand_visualizer --udp --port 9872
"""

import argparse
import json
import math
import socket
import sys
import threading
import time
from typing import Optional

import numpy as np

try:
    import pygame
    import pygame.gfxdraw
except ImportError:
    print("[ERROR] pygame not installed. Run: pip install pygame")
    sys.exit(1)

from operator.hand.manus_reader import (
    HandData, FINGER_NAMES, JOINT_NAMES_PER_FINGER,
    NUM_JOINTS, NUM_FINGERS, JOINTS_PER_FINGER,
)


# ─────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────

WINDOW_W, WINDOW_H = 1280, 720
BG_COLOR = (25, 25, 35)
PANEL_COLOR = (35, 35, 50)
TEXT_COLOR = (200, 200, 210)
GRID_COLOR = (50, 50, 65)
JOINT_RADIUS = 6
BONE_WIDTH = 4

# Finger colors (Thumb, Index, Middle, Ring, Pinky)
FINGER_COLORS = [
    (255, 100, 100),   # Thumb  — red
    (100, 200, 255),   # Index  — blue
    (100, 255, 150),   # Middle — green
    (255, 200, 80),    # Ring   — yellow
    (200, 130, 255),   # Pinky  — purple
]

FINGER_COLORS_DIM = [
    tuple(max(c // 3, 30) for c in col) for col in FINGER_COLORS
]

# Hand geometry — relative bone lengths (px)
# Each finger: [metacarpal (in palm), proximal, intermediate, distal]
BONE_LENGTHS = {
    "Thumb":  [45, 38, 30, 25],
    "Index":  [70, 42, 26, 20],
    "Middle": [70, 46, 30, 22],
    "Ring":   [65, 42, 28, 20],
    "Pinky":  [60, 34, 22, 18],
}

# Base angles from palm center (radians, 0 = up)
FINGER_BASE_ANGLES = {
    "Thumb":  math.radians(-70),
    "Index":  math.radians(-25),
    "Middle": math.radians(-5),
    "Ring":   math.radians(12),
    "Pinky":  math.radians(30),
}

# Palm attachment offsets from palm center (x, y)
PALM_OFFSETS = {
    "Thumb":  (-45, 15),
    "Index":  (-28, -38),
    "Middle": (-5, -42),
    "Ring":   (18, -38),
    "Pinky":  (38, -30),
}


# ─────────────────────────────────────────────────────────
# Mock data generator
# ─────────────────────────────────────────────────────────

class MockDataSource:
    """Generates animated mock hand data for testing."""

    def __init__(self, hand_side: str = "right"):
        self._side = hand_side
        self._t0 = time.time()

    def get(self) -> HandData:
        t = time.time() - self._t0
        angles = np.zeros(NUM_JOINTS, dtype=np.float32)

        for f in range(NUM_FINGERS):
            base = f * JOINTS_PER_FINGER
            freq = 0.5 + f * 0.15
            phase = f * 0.4

            # Spread: small oscillation
            angles[base + 0] = math.sin(t * freq + phase) * 0.15

            # Flexion joints: wave-like curl/uncurl
            curl = (math.sin(t * freq + phase) + 1.0) * 0.5  # [0, 1]
            angles[base + 1] = curl * 1.2   # MCP flexion
            angles[base + 2] = curl * 1.5   # PIP flexion
            angles[base + 3] = curl * 1.0   # DIP flexion

        spread = np.array([angles[f * JOINTS_PER_FINGER] for f in range(NUM_FINGERS)])

        return HandData(
            joint_angles=angles,
            finger_spread=spread,
            wrist_pos=np.array([0.0, 0.0, 0.3]),
            wrist_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            hand_side=self._side,
            timestamp=time.time(),
        )


# ─────────────────────────────────────────────────────────
# UDP data source
# ─────────────────────────────────────────────────────────

class UDPDataSource:
    """Receives hand data from manus_sender via UDP."""

    def __init__(self, port: int = 9872, bind_ip: str = "0.0.0.0"):
        self._port = port
        self._bind_ip = bind_ip
        self._latest: Optional[HandData] = None
        self._lock = threading.Lock()
        self._running = False
        self._pkt_count = 0

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_ip, self._port))
        self._sock.settimeout(0.1)
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print(f"[UDP] Listening on {self._bind_ip}:{self._port}")

    def stop(self):
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        if hasattr(self, "_sock"):
            self._sock.close()

    def get(self) -> Optional[HandData]:
        with self._lock:
            return self._latest

    @property
    def packet_count(self) -> int:
        return self._pkt_count

    def _recv_loop(self):
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(4096)
                pkt = json.loads(raw.decode())
                data = HandData(
                    joint_angles=np.array(pkt["joint_angles"], dtype=np.float32),
                    finger_spread=np.array(pkt["finger_spread"], dtype=np.float32),
                    wrist_pos=np.array(pkt.get("wrist_pos", [0, 0, 0])),
                    wrist_quat=np.array(pkt.get("wrist_quat", [1, 0, 0, 0])),
                    hand_side=pkt.get("hand", "right"),
                    timestamp=pkt.get("timestamp", time.time()),
                )
                with self._lock:
                    self._latest = data
                self._pkt_count += 1
            except socket.timeout:
                continue
            except Exception:
                continue


# ─────────────────────────────────────────────────────────
# SDK data source
# ─────────────────────────────────────────────────────────

class SDKDataSource:
    """Reads from Manus glove via ManusReader."""

    def __init__(self, sdk_path: str, hand_side: str = "right"):
        from operator.hand.manus_reader import ManusReader
        self._reader = ManusReader(sdk_bin_path=sdk_path, hand_side=hand_side)
        self._reader.connect()
        print(f"[SDK] Connected to Manus glove ({hand_side})")

    def get(self) -> Optional[HandData]:
        return self._reader.get_hand_data()

    def stop(self):
        self._reader.disconnect()


# ─────────────────────────────────────────────────────────
# Renderer
# ─────────────────────────────────────────────────────────

class HandVisualizer:
    """Pygame-based hand skeleton renderer."""

    def __init__(self):
        pygame.init()
        self._screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Manus Hand Visualizer")
        self._clock = pygame.time.Clock()
        self._font = pygame.font.SysFont("monospace", 14)
        self._font_large = pygame.font.SysFont("monospace", 18, bold=True)
        self._font_title = pygame.font.SysFont("monospace", 22, bold=True)
        self._frame_count = 0
        self._fps_timer = time.time()
        self._display_fps = 0.0
        self._data_fps = 0.0
        self._last_data_time = 0.0
        self._data_count = 0
        self._data_fps_timer = time.time()
        self._baseline = None
        self._baseline_samples = []
        self._baseline_count = 30  # Average first N frames as baseline

    def draw(self, data: Optional[HandData], mode: str, extra_info: str = ""):
        self._screen.fill(BG_COLOR)

        # Layout: skeleton on left, bar charts on right
        skeleton_cx = WINDOW_W // 3
        skeleton_cy = WINDOW_H // 2 + 40
        bar_x_start = WINDOW_W // 2 + 20

        # Title bar
        self._draw_title_bar(data, mode, extra_info)

        if data is not None:
            # Track data rate
            self._data_count += 1
            now = time.time()
            dt = now - self._data_fps_timer
            if dt >= 1.0:
                self._data_fps = self._data_count / dt
                self._data_count = 0
                self._data_fps_timer = now

            # Draw hand skeleton
            is_left = (data.hand_side == "left")
            self._draw_hand(skeleton_cx, skeleton_cy, data, is_left)

            # Draw bar charts
            self._draw_bar_charts(bar_x_start, 70, data)

            # Draw wrist info
            self._draw_wrist_info(bar_x_start, WINDOW_H - 100, data)
        else:
            # No data
            txt = self._font_large.render("Waiting for data...", True, (150, 150, 150))
            rect = txt.get_rect(center=(WINDOW_W // 2, WINDOW_H // 2))
            self._screen.blit(txt, rect)

        # FPS counter
        self._frame_count += 1
        now = time.time()
        if now - self._fps_timer >= 1.0:
            self._display_fps = self._frame_count / (now - self._fps_timer)
            self._frame_count = 0
            self._fps_timer = now

        fps_txt = self._font.render(
            f"Render: {self._display_fps:.0f} FPS | Data: {self._data_fps:.0f} Hz",
            True, (100, 100, 120)
        )
        self._screen.blit(fps_txt, (WINDOW_W - 320, WINDOW_H - 25))

        pygame.display.flip()
        self._clock.tick(60)

    def _draw_title_bar(self, data: Optional[HandData], mode: str, extra_info: str):
        pygame.draw.rect(self._screen, PANEL_COLOR, (0, 0, WINDOW_W, 50))
        pygame.draw.line(self._screen, GRID_COLOR, (0, 50), (WINDOW_W, 50))

        title = self._font_title.render("Manus Hand Visualizer", True, TEXT_COLOR)
        self._screen.blit(title, (15, 12))

        mode_colors = {"mock": (255, 200, 80), "sdk": (100, 255, 150), "udp": (100, 200, 255)}
        mode_col = mode_colors.get(mode, TEXT_COLOR)
        mode_txt = self._font.render(f"[{mode.upper()}]", True, mode_col)
        self._screen.blit(mode_txt, (300, 16))

        if data is not None:
            hand_txt = self._font.render(
                f"Hand: {data.hand_side.upper()}", True, TEXT_COLOR
            )
            self._screen.blit(hand_txt, (380, 16))

        if extra_info:
            info_txt = self._font.render(extra_info, True, (120, 120, 140))
            self._screen.blit(info_txt, (520, 16))

    def _draw_hand(self, cx: int, cy: int, data: HandData, is_left: bool):
        """Draw 2D hand skeleton centered at (cx, cy)."""
        # Build baseline from first N frames (open hand at startup)
        if self._baseline is None:
            self._baseline_samples.append(data.joint_angles.copy())
            if len(self._baseline_samples) >= self._baseline_count:
                self._baseline = np.mean(self._baseline_samples, axis=0)
                self._baseline_samples = []
            else:
                # Use zeros until baseline is ready
                pass

        scale = 1.8
        mirror = -1 if is_left else 1

        # Draw palm outline
        palm_pts = []
        for fname in FINGER_NAMES:
            ox, oy = PALM_OFFSETS[fname]
            px = cx + int(ox * mirror * scale)
            py = cy + int(oy * scale)
            palm_pts.append((px, py))

        # Add wrist points
        wrist_l = (cx + int(-35 * mirror * scale), cy + int(45 * scale))
        wrist_r = (cx + int(35 * mirror * scale), cy + int(45 * scale))

        # Palm polygon
        palm_polygon = [palm_pts[0]] + palm_pts + [wrist_r, wrist_l]
        pygame.draw.polygon(self._screen, (45, 45, 60), palm_polygon)
        pygame.draw.polygon(self._screen, (80, 80, 100), palm_polygon, 2)

        # Draw each finger
        for f_idx, fname in enumerate(FINGER_NAMES):
            color = FINGER_COLORS[f_idx]
            color_dim = FINGER_COLORS_DIM[f_idx]
            bones = BONE_LENGTHS[fname]
            base_angle = FINGER_BASE_ANGLES[fname]

            # Apply mirror for left hand
            if is_left:
                base_angle = math.pi - base_angle

            # Start position
            ox, oy = PALM_OFFSETS[fname]
            start_x = cx + ox * mirror * scale
            start_y = cy + oy * scale

            # Joint angles for this finger (subtract baseline if available)
            j_base = f_idx * JOINTS_PER_FINGER
            if self._baseline is not None:
                adj = data.joint_angles - self._baseline
            else:
                adj = data.joint_angles

            spread = adj[j_base + 0]
            flexions = [
                adj[j_base + 1],
                adj[j_base + 2],
                adj[j_base + 3],
            ]

            # Build kinematic chain
            # Spread modifies base direction, flexion bends each segment
            angle = base_angle + spread * mirror * (-1)

            points = [(int(start_x), int(start_y))]
            x, y = start_x, start_y

            for bone_idx in range(1, 4):  # 3 visible bones (proximal, intermediate, distal)
                bone_len = bones[bone_idx] * scale

                # Apply flexion (positive = curl inward)
                angle += flexions[bone_idx - 1] * 1.0

                dx = math.sin(angle) * bone_len * (-1)
                dy = math.cos(angle) * bone_len * (-1)
                x += dx
                y += dy
                points.append((int(x), int(y)))

            # Draw bones
            for i in range(len(points) - 1):
                pygame.draw.line(self._screen, color_dim, points[i], points[i + 1], BONE_WIDTH + 2)
                pygame.draw.line(self._screen, color, points[i], points[i + 1], BONE_WIDTH)

            # Draw joints
            for i, pt in enumerate(points):
                r = JOINT_RADIUS if i > 0 else JOINT_RADIUS - 1
                pygame.draw.circle(self._screen, (255, 255, 255), pt, r + 1)
                pygame.draw.circle(self._screen, color, pt, r)

            # Finger label at tip
            tip = points[-1]
            label = self._font.render(fname[0], True, color)
            self._screen.blit(label, (tip[0] - 4, tip[1] - 22))

        # Hand label
        label = self._font_large.render(
            f"{'LEFT' if is_left else 'RIGHT'} HAND", True, TEXT_COLOR
        )
        rect = label.get_rect(center=(cx, cy + int(75 * scale)))
        self._screen.blit(label, rect)

    def _draw_bar_charts(self, x_start: int, y_start: int, data: HandData):
        """Draw per-joint bar charts on the right panel."""
        bar_w = 100
        bar_h = 12
        gap_y = 2
        finger_gap = 10

        # Section title
        title = self._font_large.render("Joint Angles (rad)", True, TEXT_COLOR)
        self._screen.blit(title, (x_start, y_start - 5))

        y = y_start + 25

        for f_idx, fname in enumerate(FINGER_NAMES):
            color = FINGER_COLORS[f_idx]
            color_dim = FINGER_COLORS_DIM[f_idx]
            jnames = JOINT_NAMES_PER_FINGER[fname]

            # Finger name
            flabel = self._font.render(fname, True, color)
            self._screen.blit(flabel, (x_start, y))
            y += 18

            for j in range(JOINTS_PER_FINGER):
                idx = f_idx * JOINTS_PER_FINGER + j
                val = data.joint_angles[idx]

                # Joint name
                jlabel = self._font.render(f"  {jnames[j]:11s}", True, (140, 140, 155))
                self._screen.blit(jlabel, (x_start, y))

                # Bar background
                bar_x = x_start + 140
                pygame.draw.rect(self._screen, (40, 40, 55),
                                 (bar_x, y + 1, bar_w, bar_h))

                # Bar fill — normalize roughly to [-1, 2] range for display
                max_val = 2.0
                min_val = -0.5
                norm = (val - min_val) / (max_val - min_val)
                norm = max(0.0, min(1.0, norm))
                fill_w = int(norm * bar_w)

                if fill_w > 0:
                    pygame.draw.rect(self._screen, color_dim,
                                     (bar_x, y + 1, fill_w, bar_h))
                    pygame.draw.rect(self._screen, color,
                                     (bar_x, y + 1, fill_w, bar_h), 1)

                # Zero line
                zero_x = bar_x + int((-min_val) / (max_val - min_val) * bar_w)
                if bar_x < zero_x < bar_x + bar_w:
                    pygame.draw.line(self._screen, (100, 100, 120),
                                     (zero_x, y + 1), (zero_x, y + bar_h), 1)

                # Value text
                vtxt = self._font.render(f"{val:+.3f}", True, (160, 160, 175))
                self._screen.blit(vtxt, (bar_x + bar_w + 8, y))

                y += bar_h + gap_y

            y += finger_gap

    def _draw_wrist_info(self, x: int, y: int, data: HandData):
        """Draw wrist position and quaternion."""
        pygame.draw.line(self._screen, GRID_COLOR, (x, y), (x + 400, y))
        y += 8

        title = self._font.render("Wrist", True, TEXT_COLOR)
        self._screen.blit(title, (x, y))
        y += 20

        pos = data.wrist_pos
        quat = data.wrist_quat
        pos_txt = self._font.render(
            f"  Pos:  x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}", True, (140, 140, 155)
        )
        quat_txt = self._font.render(
            f"  Quat: w={quat[0]:+.4f}  x={quat[1]:+.4f}  y={quat[2]:+.4f}  z={quat[3]:+.4f}",
            True, (140, 140, 155)
        )
        self._screen.blit(pos_txt, (x, y))
        self._screen.blit(quat_txt, (x, y + 18))

    def quit(self):
        pygame.quit()


# ─────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Manus glove real-time hand skeleton visualizer"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--mock", action="store_true", default=True,
                       help="Use animated mock data (default)")
    group.add_argument("--sdk", action="store_true",
                       help="Read from Manus glove via SDK")
    group.add_argument("--udp", action="store_true",
                       help="Receive UDP packets from manus_sender")

    parser.add_argument("--hand", default="right", choices=["left", "right"],
                        help="Hand side (default: right)")
    parser.add_argument("--sdk-path", default="operator/hand/sdk/SDKClient_Linux/SDKClient_Linux.out",
                        help="Path to SDKClient_Linux.out (with --sdk)")
    parser.add_argument("--port", type=int, default=9872,
                        help="UDP port (with --udp, default: 9872)")
    parser.add_argument("--bind-ip", default="0.0.0.0",
                        help="UDP bind IP (with --udp, default: 0.0.0.0)")
    args = parser.parse_args()

    # Determine mode
    if args.sdk:
        mode = "sdk"
    elif args.udp:
        mode = "udp"
    else:
        mode = "mock"

    print("=" * 55)
    print("  Manus Hand Visualizer")
    print(f"  Mode: {mode.upper()} | Hand: {args.hand}")
    print("  Controls: ESC/Q = Quit")
    print("=" * 55)

    # Create data source
    source = None
    udp_source = None

    if mode == "mock":
        source = MockDataSource(hand_side=args.hand)
    elif mode == "sdk":
        try:
            source = SDKDataSource(sdk_path=args.sdk_path, hand_side=args.hand)
        except Exception as e:
            print(f"[ERROR] SDK connection failed: {e}")
            print("        Falling back to mock data")
            mode = "mock"
            source = MockDataSource(hand_side=args.hand)
    elif mode == "udp":
        udp_source = UDPDataSource(port=args.port, bind_ip=args.bind_ip)
        udp_source.start()

    # Create visualizer
    viz = HandVisualizer()

    try:
        running = True
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

            # Get data
            if mode == "udp":
                data = udp_source.get()
                extra = f"Port: {args.port} | Pkts: {udp_source.packet_count}"
            else:
                data = source.get()
                extra = ""

            # Draw
            viz.draw(data, mode, extra)

    except KeyboardInterrupt:
        pass
    finally:
        viz.quit()
        if mode == "sdk" and hasattr(source, "stop"):
            source.stop()
        if udp_source is not None:
            udp_source.stop()

    print("\nVisualizer closed.")


if __name__ == "__main__":
    main()
