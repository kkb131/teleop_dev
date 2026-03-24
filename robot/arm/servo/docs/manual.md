# servo/ 학습 매뉴얼 — 기본 텔레옵 (난이도 ★)

> **위치**: `standalone/servo/`
> **의존 core 모듈**: `robot_backend`, `kinematics` (DLS IK)
> **학습 선수 지식**: [core/ 매뉴얼](../../core/docs/manual.md) §2 (로봇 통신), §4.1 (DLS IK)

---

## 1. 왜 servo/가 존재하는가

servo/는 standalone 패키지에서 **가장 단순한 실시간 제어** 모듈이다.

- **목적**: 키보드/조이스틱으로 UR10e를 실시간 텔레오퍼레이션
- **핵심 특징**: 복잡한 안전 시스템 없이, 최소한의 코드로 로봇 제어 가능
- **학습 가치**: RobotBackend + IK 사용법을 가장 단순한 형태로 보여줌

teleop_admittance나 teleop_impedance와 달리:
- F/T 센서, 안전 모니터, 필터 등이 **없거나 단순**
- Pink IK 대신 **Pinocchio DLS** (관절 한계 미고려)
- 독립 모듈(main.py) 없이 **각 스크립트가 독립 실행 가능**

---

## 2. 4개 스크립트 비교

| 스크립트 | 입력 | 제어 공간 | IK | 핵심 특징 |
|---------|------|----------|-----|----------|
| `keyboard_cartesian.py` | 키보드 | 카르테시안 (XYZ + RPY) | Pinocchio DLS | 가장 기본적인 텔레옵 |
| `keyboard_forward.py` | 키보드 | 관절 공간 (J1~J6 직접) | 없음 (직접 제어) | IK 없이 관절 직접 조작 |
| `joystick_cartesian.py` | Xbox 조이스틱 | 카르테시안 | Pinocchio DLS | 아날로그 연속 입력 |
| `keyboard_servo_admittance.py` | 키보드 + F/T | 카르테시안 | Pinocchio DLS | 간이 어드미턴스 (프로토타입) |

### 학습 순서 추천
1. `keyboard_forward.py` — IK 없이 관절 직접 제어 (가장 단순)
2. `keyboard_cartesian.py` — DLS IK 도입 (카르테시안 제어)
3. `joystick_cartesian.py` — 입력 장치 변경 (Xbox)
4. `keyboard_servo_admittance.py` — F/T 어드미턴스 추가 (teleop_admittance 프리뷰)

---

## 3. 데이터 흐름 분석

### keyboard_cartesian.py (핵심 파이프라인)

```
┌──────────┐     ┌─────────┐     ┌───────────┐     ┌──────────┐
│ Keyboard │ ──▷ │  twist  │ ──▷ │ DLS IK    │ ──▷ │ Backend  │
│ (w/a/s/d)│     │ [6]     │     │ Δq = f(J) │     │ servoJ() │
└──────────┘     └─────────┘     └───────────┘     └──────────┘
                                      ▲
                                      │
                              ┌───────────────┐
                              │ get_joint_    │
                              │ positions()   │
                              └───────────────┘
```

1. **키 입력** → KEY_MAP에서 twist 방향 벡터 조회 (예: 'w' → [1,0,0,0,0,0])
2. **속도 스케일링** → `twist × speed_scale`
3. **DLS IK** → `compute_joint_delta(q, twist, dt)` → Δq 계산
4. **관절 한계 클램핑** → `clamp_positions(q + Δq)`
5. **로봇 명령 전송** → `backend.send_joint_command(target)`

### keyboard_forward.py (더 단순)

```
키보드 (1-6: 관절 선택, w/s: +/-) → target_q[i] += step_size → backend.send_joint_command()
```

IK가 전혀 없다. 관절 위치를 직접 증감시킨다.

---

## 4. 핵심 코드 분석

### 4.1 keyboard_cartesian.py — 메인 루프

```python
# keyboard_cartesian.py:216-238 (run 메서드 핵심)
while self.running:
    self.update_positions()           # 1. 현재 관절 읽기

    key = self.get_key(timeout=dt)    # 2. 키 입력 (non-blocking)
    if key:
        self.process_key(key)         # 3. twist 업데이트

    if np.any(self.current_twist != 0.0):
        twist = self.current_twist * self.speed_scale
        dq = self.ik.compute_joint_delta(             # 4. DLS IK
            self.current_positions, twist, dt,
            damping=DAMPING, local=self.use_local_frame,
        )
        target = self.ik.clamp_positions(             # 5. 한계 클램핑
            self.current_positions + dq
        )
        self.send_command(target)                      # 6. 명령 전송
        self.current_twist = np.zeros(6)               # 7. twist 리셋 (one-shot)
    else:
        self.send_command(self.current_positions)       # 입력 없으면 현재 위치 유지

    time.sleep(dt)  # 50Hz (SERVO_RATE_HZ)
```

**핵심 설계**: 키보드 입력은 **one-shot** 방식이다. 키를 한 번 누르면 한 스텝만 이동.
키를 누르고 있으면 반복 입력이 되지만, 각 루프에서 twist가 리셋된다.

### 4.2 keyboard_forward.py — 관절 직접 제어

```python
# keyboard_forward.py:147-153 (관절 증감)
if key in ('w', 'UP'):
    self.target_positions[self.selected_joint] += self.step_size
if key in ('s', 'DOWN'):
    self.target_positions[self.selected_joint] -= self.step_size
```

`self.target_positions`는 `send_command()`에서 매 루프 전송된다.
IK 없이 관절 공간에서 직접 제어하므로, 엔드이펙터가 어디로 가는지는 사용자가 직접 판단해야 한다.

### 4.3 joystick_cartesian.py — Xbox 입력 처리

```python
# joystick_cartesian.py:155-170 (아날로그 입력 → twist)
lx = self._apply_deadzone(axes[AXIS_LEFT_STICK_X])    # 좌 스틱 X
ly = self._apply_deadzone(axes[AXIS_LEFT_STICK_Y])    # 좌 스틱 Y
rx = self._apply_deadzone(axes[AXIS_RIGHT_STICK_X])   # 우 스틱 X
ry = self._apply_deadzone(axes[AXIS_RIGHT_STICK_Y])   # 우 스틱 Y
lt = (1.0 - axes[AXIS_LT]) / 2.0                       # 좌 트리거 (0~1)
rt = (1.0 - axes[AXIS_RT]) / 2.0                       # 우 트리거 (0~1)

self.current_twist = np.array([
    ly, lx, rt - lt, roll, ry, rx,   # [vx, vy, vz, wx, wy, wz]
])
```

키보드와 달리 **아날로그 연속 값**이다. 스틱을 살짝 밀면 천천히, 끝까지 밀면 빠르게 이동.
**데드존 (0.1)**: 스틱이 정확히 0으로 돌아오지 않는 하드웨어 특성을 보상.

> **참고**: `joystick_cartesian.py`는 ROS2 `/joy` 토픽을 구독한다.
> 따라서 sim 모드에서도 `ros2 run joy joy_node`을 별도 실행해야 한다.

### 4.4 keyboard_servo_admittance.py — 간이 어드미턴스

이 스크립트는 `keyboard_cartesian.py`에 **간이 어드미턴스 제어**를 추가한 것이다.
core/ 모듈을 사용하지 않고, 어드미턴스 로직을 **스크립트 내부에 직접 구현**한다.

```python
# keyboard_servo_admittance.py:460-463 (텔레옵 + 어드미턴스 결합)
# 1) 키보드 → DLS → dq_teleop
# 2) F/T → 어드미턴스 → dq_admittance
# 3) 결합
target = self.ik.clamp_positions(
    current_q + dq_teleop + dq_admittance
)
```

이 스크립트는 **teleop_admittance 모듈의 프로토타입**이다.
독립적으로 동작하지만, 안전 모니터나 설정 파일이 없다.
본격적인 어드미턴스 텔레옵은 → [teleop_admittance/ 매뉴얼](../../teleop_admittance/docs/manual.md) 참조.

---

## 5. 공통 패턴

### 5.1 Backend Context Manager

모든 스크립트가 동일한 패턴을 사용한다:

```python
backend = create_backend(args.mode, robot_ip=args.robot_ip)
with backend:
    ctrl = SomeController(backend)
    ctrl.run()
# disconnect() 자동 호출
```

### 5.2 Non-blocking 키 입력 (termios)

```python
# keyboard_cartesian.py:120-139
def setup_terminal(self):
    self._old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())     # 입력 즉시 전달 (Enter 불필요)

def get_key(self, timeout=0.02):
    if select.select([sys.stdin], [], [], timeout)[0]:  # timeout 내 입력 있으면
        ch = sys.stdin.read(1)
        ...
    return None  # timeout → 입력 없음
```

`tty.setcbreak()`: 터미널을 raw 모드로 전환. 키 하나를 누르면 즉시 프로그램에 전달된다 (Enter 불필요).
종료 시 반드시 `restore_terminal()`로 원래 설정을 복원해야 한다.

### 5.3 Signal Handler

```python
def signal_handler(sig, frame):
    ctrl.running = False    # Ctrl+C로 루프 종료
signal.signal(signal.SIGINT, signal_handler)
```

`Ctrl+C` 시 즉시 종료 대신 `running = False`로 설정하여, finally 블록에서 터미널 복원이 정상 실행되도록 한다.

---

## 6. servo/ → teleop_admittance로의 진화

servo/에서 부족한 점과 teleop_admittance가 해결하는 것:

| servo/의 한계 | teleop_admittance의 해결 |
|-------------|----------------------|
| DLS IK: 관절 한계 미고려 | Pink QP IK: 관절 한계를 QP 제약으로 보장 |
| 안전 시스템 없음 | SafetyMonitor: 작업 공간, 속도, 특이점, 힘 검사 |
| F/T 어드미턴스 프로토타입 (인라인) | core/compliant_control.py 모듈화 |
| 입력 처리 인라인 | core/input_handler.py 공유 모듈 |
| 필터링 없음 | core/exp_filter.py EMA + slerp |
| 설정 하드코딩 | YAML 설정 파일 (config/default.yaml) |
| one-shot 이동 (키당 한 스텝) | 연속 이동 (목표 포즈 추적) |

servo/는 **학습과 프로토타이핑**에 적합하고,
teleop_admittance는 **실제 운용**에 적합하다.

---

## 다음 단계

- [cumotion/ 매뉴얼](../../cumotion/docs/manual.md) — 오프라인 경로 계획 (GPU)
- [teleop_admittance/ 매뉴얼](../../teleop_admittance/docs/manual.md) — 어드미턴스 텔레옵 (Pink IK + F/T)

> 실행 방법, CLI 옵션, 트러블슈팅은 [servo/docs/user_guide.md](user_guide.md)를 참조하세요.
