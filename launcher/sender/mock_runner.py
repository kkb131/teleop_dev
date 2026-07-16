#!/usr/bin/env python3
"""XR 러너 mock — 데모/E2E 검증용 (헤드셋/로봇 불필요).

실제 러너의 로그 계약(status.py 파서가 검증한 포맷)을 그대로 흉내내고,
pty stdin 으로 들어오는 키에 반응한다:
    'r' : --fail-first N 회까지는 거부 WARN, 이후 calibrate 라인 (SYNC 전이)
    'p' : PAUSE 토글
    'x'/'q' : 종료 (exit 0)

플래그:
    --single        : 무라벨 단일 모드 ([Sender] / [XRArmSender])
    --fail-first N  : 처음 N 번의 'r' 을 헤드셋-없음 WARN 으로 거부 (기본 0)
    --no-handshake  : Pose query failed 출력 (하드 실패 경로 테스트)
    --hz            : 상태 라인 주기 (기본 1.0s — 실물은 2s)
"""

from __future__ import annotations

import argparse
import select
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="mock XR runner (E2E 용)")
    parser.add_argument("--single", action="store_true")
    parser.add_argument("--fail-first", type=int, default=0)
    parser.add_argument("--no-handshake", action="store_true")
    parser.add_argument("--hz", type=float, default=1.0)
    args = parser.parse_args()

    sides = ["single"] if args.single else ["right", "left"]

    def arm_tag(side):
        return "[XRArmSender]" if side == "single" else f"[XRArmSender:{side}]"

    def sender_tag(side):
        return "[Sender]" if side == "single" else f"[Sender:{side}]"

    # tty 면 cbreak (실제 러너와 동일 조건 — pty 하에서 즉시 키 수신)
    old_settings = None
    if sys.stdin.isatty():
        import termios
        import tty
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    print("[run_xr_dual_teleop] BridgePoseStore: http://localhost:8013/", flush=True)
    print("[BridgePoseStore] ws client connected: 127.0.0.1", flush=True)
    for _ in sides:
        if args.no_handshake:
            print("[Sender] WARNING: Pose query failed. Using default home pose.",
                  flush=True)
        else:
            print("[Sender] Initial pose received: pos=[0.500, -0.100, 0.400]",
                  flush=True)
    print("[Sender] Running. Press Q or Ctrl+C to stop.", flush=True)

    state = "READY"          # READY → SYNC ↔ PAUSE
    r_count = 0
    n = 0
    period = 1.0 / max(args.hz, 0.1)
    last_status = 0.0

    try:
        while True:
            # 키 입력
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch in ("x", "q"):
                    print("[xr_dual] quit — 전체 종료", flush=True)
                    return 0
                if ch == "r":
                    r_count += 1
                    if r_count <= args.fail_first:
                        for s in sides:
                            print(f"{arm_tag(s)} WARN: 헤드셋 ws msg 없음 — "
                                  f"헤드셋 사이트 접속 + Enter VR/AR + 손 들이밀고 'r' 재시도",
                                  flush=True)
                    else:
                        for s in sides:
                            print(f"{arm_tag(s)} calibrate (r)", flush=True)
                            if args.no_handshake:
                                print("[Sender] WARNING: Pose query failed. "
                                      "Using default home pose.", flush=True)
                        state = "SYNC"
                elif ch == "p":
                    state = "PAUSE" if state == "SYNC" else \
                        ("SYNC" if state == "PAUSE" else state)
                    print(f"[XRArmSender] {'⏸  PAUSED' if state == 'PAUSE' else '▶  RESUMED'}",
                          flush=True)

            # 주기 상태 라인
            now = time.perf_counter()
            if now - last_status >= period:
                last_status = now
                n += 1
                for s in sides:
                    print(f"{sender_tag(s)} #{n:>7d}  "
                          f"pos=[+0.500, -0.100, +0.400]  spd={state} x0.5",
                          flush=True)
                    hand_tag = ("[run_xr_teleop:hand]" if s == "single"
                                else f"[xr_dual:hand:{s}]")
                    print(f"{hand_tag} #{n:>6d} TRACK skip=0 "
                          f"q[5/9/13/17]=[+0.10/+0.20/+0.30/+0.40]", flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        if old_settings is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    sys.exit(main())
