# Phase D · Unit D1 — 통합 launcher (`scripts/run_xr_teleop.py`)

## 목적

조종 PC 에서 한 process 로:
- BridgePoseStore (Singleton, ws server)
- XR arm sender (UDP 9871)
- XR hand sender (UDP 9872)

세 가지를 동시 실행. port 8013 (ws bridge) 중복 부팅 회피 + arm/hand 가 같은 헤드셋 데이터 공유.

## 설계

```
main() — 메인 thread
    │
    ├─ BridgePoseStore(use_hand_tracking=True)   ← Singleton (port 8013)
    │     └─ background aiohttp HTTP+WS server (daemon thread)
    │
    ├─ hand sender thread (daemon, background)
    │     └─ _hand_thread_fn — 60Hz retarget + UDP 9872 송신
    │     └─ pynput keyboard 미사용 (메인 termios 와 충돌 회피)
    │
    └─ arm sender thread (foreground, 메인 thread 직접 실행)
          └─ XRArmSender.run() — termios 키 입력 + 50Hz UDP 9871
          └─ 'x' 또는 'q' 종료 시 stop_event.set()
```

## 키 매핑 (모두 메인 thread = arm sender 가 받음)

| 키 | 동작 |
|---|---|
| `r` | arm sync 시작 / recalibrate (사용자 + robot origin 동시 capture) |
| `p` | arm pause/resume (손 새 위치로 옮길 때 안전) |
| `c` | immediate recalibrate |
| `Space` | E-Stop (arm 측 ButtonState.estop) |
| `+` / `-` | speed scale |
| `x` / `Esc` / `q` | sender 종료 |
| Ctrl+C | 강제 종료 (signal propagate) |

hand sender 는 background 라 별도 키 처리 안 함. 손 동작 자체로 제어 (open/fist/pinch).

## 신규 파일

`src/teleop_dev/scripts/run_xr_teleop.py`:
- `main()` — argparse + Singleton store + 두 thread 시작
- `_arm_thread_fn(args, store, stop_event)` — XRArmSender 메인 thread 실행
- `_hand_thread_fn(args, store, stop_event)` — retarget + UDP 송신 loop

xr_hand_sender 의 `main()` 을 그대로 호출하지 않은 이유: pynput 두 번 시작 회피 + termios 충돌 회피 + 종료 동기화. 내부 loop 만 재사용 (`_build_packet`, `_build_null_packet`, `XRDexRetargeter`).

## CLI

```bash
# 둘 다 (default)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10

# 팔만
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-hand

# 손만
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --no-arm

# 안전 (저속 / 1:1 미만)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3

# 모든 옵션 표시
python3 -m scripts.run_xr_teleop --help
```

## 검증 (smoke)

### 1. import + argparse

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m scripts.run_xr_teleop --help
# 정상 출력되어야 함
```

### 2. Singleton 검증 (BridgePoseStore 한 번만 부팅)

```bash
# robot loopback (admittance sim + manus_sender style receiver dry)
# Terminal 1
python3 -m robot.arm.admittance.main --mode sim --input unified

# Terminal 2
python3 -m robot.hand.receiver --dry-run --hand right

# Terminal 3
adb reverse tcp:8013 tcp:8013
python3 -m scripts.run_xr_teleop --target-ip 127.0.0.1

# 기대: "BridgePoseStore: http://localhost:8013/" 메시지 1회만
# (두 sender 가 같은 인스턴스 사용)
```

### 3. 두 sender 동시 동작 확인

```bash
# 위 셋업 그대로, 헤드셋 → http://localhost:8013/ → Enter VR → 손 들이밀기
# 'r' 키 → arm calibrate
# arm sender 상태 표시: [Sender] #...  pos=[+0.10, -0.40, +0.40]  spd=SYNC x1.0
# hand sender (background) 2초마다: [run_xr_teleop:hand] #720 TRACK ws_msg=1440 q[5/9/13/17]=...
```

## 종료 흐름

1. 사용자가 'x' / 'q' / 'Esc' 입력
2. arm thread (foreground) `run()` 종료 → stop_event.set()
3. hand thread (background, daemon) loop 가 `stop_event` 체크해 종료
4. `hand_thread.join(timeout=2.0)`
5. main() return

Ctrl+C 도 같은 흐름 — arm 의 KeyboardInterrupt 가 stop_event 트리거.

## 영향 받는 파일

신규:
- `src/teleop_dev/scripts/run_xr_teleop.py`

수정 없음.
