# Phase E · Unit E1 — Sender 측 안전 layer

## 목적

조종 PC 의 사용자 손/머리 추적이 끊겼을 때 (헤드셋 연결 끊김, 손 시야 밖, Chrome 멈춤) robot 으로 stale target 송신 방지 + workspace envelope clamp.

## 출처

- xr_teleop: [`docs/remote_teleop_office_to_field_analysis.md §4.6`](../../../../xr_teleop/docs/remote_teleop_office_to_field_analysis.md) — 100ms heartbeat / workspace boundary 권고
- teleop_dev: [`robot/arm/admittance/safety_monitor.py`](../../../robot/arm/admittance/safety_monitor.py) — robot PC 측 4단계 safety (E-Stop, workspace, velocity, timeout). sender 측 layer 는 그 외 추가 보호.

## 핵심 설계 결정

| 검사 | sender 측 | robot 측 |
|---|---|---|
| **freshness watchdog** | StoreWatchdog (200ms default) — stale 시 송신 skip | (admittance 의 udp timeout) |
| **workspace clamp** | WorkspaceLimits — sender 측 clamp 후 송신 | safety_monitor 가 별도 envelope 재검사 |
| **E-Stop** | Space 키 → ButtonState.estop=True | safety_monitor.process_estop() |
| **velocity** | (없음 — robot 측이 잘 함) | safety_monitor max_vel + admittance config |

## 신규 파일

`src/teleop_dev/sender/xr_common/watchdog.py`:
- `WorkspaceLimits` dataclass + `clamp(pos) → (clamped, was_clamped)`
- `StoreWatchdog(store, timeout_s)` + `fresh()` / `stale_count`

## 수정 파일

### `sender/arm/xr_sender.py`

- 생성자 추가 인자: `watchdog_timeout_s=0.2`, `workspace=None`, `enforce_workspace=True`
- `_read_input()` 안:
  - `self._watchdog.fresh()` False 면 virtual_pose 유지 + 1회 / 0.6s / 6s 카운터 별 WARN
  - target_pos 가 workspace 밖이면 clamp + WARN
- CLI 추가:
  - `--watchdog-timeout-s 0.2`
  - `--no-workspace-clamp`
  - `--workspace default`

### `scripts/run_xr_teleop.py`

- hand thread 에도 `StoreWatchdog` — stale 시 송신 skip (receiver.py EMA 가 last valid hold)
- 통합 CLI:
  - `--watchdog-timeout-s 0.2`
  - `--no-workspace-clamp`

## WorkspaceLimits default (UR10e 가정)

```
x: [-0.7, +0.7]
y: [-0.7,  0.0]    ← robot 뒤로 안 가도록
z: [+0.05, +0.8]   ← 책상 위
```

⚠️ 사용자 셋업의 책상 / 안전 fence 에 맞춰 수정. 향후 yaml config 로 분리 권장.

## 검증 (단위)

```python
import numpy as np, time
from sender.xr_common.watchdog import WorkspaceLimits, StoreWatchdog

# clamp inside / outside
w = WorkspaceLimits()
p, c = w.clamp(np.array([0.3, -0.4, 0.4]));  assert not c
p, c = w.clamp(np.array([1.5, -0.4, 0.4]));  assert c and p[0] == 0.7
p, c = w.clamp(np.array([0.3, -0.4, -0.1])); assert c and p[2] == 0.05

# watchdog
class Mock:
    t = 0.0
    def get_stats(self): return {"last_msg_time": self.t, "msg_count": 1, "head_msg_count": 1, "hand_msg_count": 1, "port": 8013}
m = Mock()
wd = StoreWatchdog(m, 0.2)
assert not wd.fresh()                         # first msg 없음
m.t = time.perf_counter();           assert wd.fresh()
m.t = time.perf_counter() - 1.0;     assert not wd.fresh()
```

위 단위 테스트 모두 본 PC 에서 통과 확인.

## 동작 (smoke)

1. `python3 -m scripts.run_xr_teleop --target-ip 127.0.0.1` 시작 — watchdog default 0.2s
2. 헤드셋 안 들이밀고 sender 송신:
   - 첫 1초: `[run_xr_teleop:hand] skip=N` 증가 (BridgePoseStore 첫 msg 없음)
3. 헤드셋 → Enter VR + 손 들이밀기:
   - msg 도착 → `fresh()=True` → 송신 시작
4. 헤드셋 종료 (Quit VR):
   - 200ms 후 sender 가 stale 판정 → `WARN: BridgePoseStore stale (count=1)` 메시지
   - 0.6s 후 `count=30`, 6s 후 `count=300` 로그
   - virtual_pose 유지 — robot 측은 마지막 target 으로 stationary
5. 사용자가 손 다시 들이밀면:
   - watchdog `fresh()=True` → 송신 재개

workspace clamp:
- 사용자 손이 워크스페이스 밖으로 나가면 target_pos 가 envelope 안으로 clamp 됨
- 콘솔에 `WARN: target pos [...] → clamped [...]` 1 / 30 / 300 번째 출력

## CLI

```bash
# default safety (watchdog 200ms, workspace clamp on)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3

# watchdog 더 빠르게 (LAN 가까운 환경)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3 \
    --watchdog-timeout-s 0.1

# workspace clamp 비활성 (위험 — sender 측만, robot 측 safety 는 그대로)
python3 -m scripts.run_xr_teleop --target-ip 192.168.0.10 --scale 0.3 \
    --no-workspace-clamp
```

## Out of Scope (별도 작업)

- F/T 임계 초과 시 자동 정지 (admittance config 의 max_force 그대로 활용)
- 로봇 PC watchdog (이미 admittance 의 socket timeout / safety_monitor)
- 해상도 가변 영상 / latency HUD (Phase F+ 또는 Phase 4)
- E-stop 핫키 GUI (현재 Space 키만)

## 영향 받는 파일

신규:
- `src/teleop_dev/sender/xr_common/watchdog.py`

수정:
- `src/teleop_dev/sender/arm/xr_sender.py` (생성자 + _read_input + CLI)
- `src/teleop_dev/scripts/run_xr_teleop.py` (hand thread + CLI)

수정 없음:
- robot/arm/admittance/safety_monitor.py — 그대로
- robot/arm/admittance/main.py — 그대로
- robot/hand/receiver.py — 그대로 (EMA filter 가 hold-on-stale 역할)
