#!/usr/bin/env python3
"""UDP joystick sender — run on a PC with a gamepad connected.

Reads joystick via pygame and sends raw axis/button data over UDP.
The receiver (NetworkInput in input_handler.py) handles deadzone/scaling.

Requirements: pygame  (pip install pygame)

Usage:
    python3 joystick_sender.py --target-ip <JETSON_IP> --port 9870
"""

import argparse
import json
import socket
import time

import pygame


def main():
    parser = argparse.ArgumentParser(description="UDP joystick sender")
    parser.add_argument("--target-ip", required=True, help="Receiver IP (Jetson)")
    parser.add_argument("--port", type=int, default=9870, help="UDP port (default: 9870)")
    parser.add_argument("--hz", type=int, default=50, help="Send rate in Hz (default: 50)")
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick found. Connect a gamepad and retry.")
        pygame.quit()
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[Sender] Joystick: {js.get_name()}")
    print(f"[Sender]   Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}, Hats: {js.get_numhats()}")
    print(f"[Sender] Sending to {args.target_ip}:{args.port} at {args.hz} Hz")
    print("[Sender] Press Ctrl+C to stop.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target_ip, args.port)
    dt = 1.0 / args.hz
    send_count = 0

    try:
        while True:
            t_start = time.perf_counter()

            pygame.event.pump()

            axes = [js.get_axis(i) for i in range(js.get_numaxes())]
            buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
            hat = list(js.get_hat(0)) if js.get_numhats() > 0 else [0, 0]

            pkt = json.dumps({"axes": axes, "buttons": buttons, "hat": hat})
            sock.sendto(pkt.encode(), target)

            send_count += 1
            if send_count % (args.hz * 5) == 0:
                print(f"[Sender] Sent {send_count} packets")

            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\n[Sender] Stopped. Total packets sent: {send_count}")
    finally:
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
