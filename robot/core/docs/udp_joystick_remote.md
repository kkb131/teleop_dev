# UDP 조이스틱 원격 제어 — 학습 가이드

> **난이도**: 기초
> **관련 파일**: `core/joystick_sender.py`, `core/input_handler.py` (NetworkInput)
> **목적**: 서버PC의 조이스틱 입력을 UDP로 로봇PC에 전송하여 원격 텔레옵 수행

---

## 1. 왜 (Why) — 이 구조가 필요한 이유

### 문제 상황

로봇을 텔레옵(원격 조종)하려면 조이스틱 입력이 필요하다. 그런데 조이스틱을 로봇PC에 직접 연결할 수 없는 상황이 자주 발생한다:

| 상황 | 설명 |
|------|------|
| **물리적 거리** | 로봇PC(Jetson AGX Orin 등)가 로봇 내부나 제어 캐비닛에 있어 USB 케이블 연결이 어려움 (USB 최대 5m) |
| **안전 격리** | 작업자는 안전 펜스 밖에서 조작해야 하지만, 로봇PC는 펜스 안에 있음 |
| **헤드리스 환경** | Jetson/산업용 PC에는 모니터·키보드가 없고 SSH로만 접근 가능 |
| **유연성** | 노트북이든 데스크탑이든 조이스틱이 연결된 아무 PC에서 조종 가능 |

### 해결: UDP 네트워크 전송

```
┌─────────────────┐          UDP (port 9870)          ┌─────────────────┐
│   서버 PC        │  ──────────────────────────────►  │   로봇 PC        │
│   (조이스틱 연결) │   JSON 패킷 (50 Hz)              │   (Jetson 등)    │
│                  │                                    │                  │
│  joystick_sender │                                    │  NetworkInput    │
│  (pygame)        │                                    │  → TeleopCommand │
│                  │                                    │  → 제어 루프      │
└─────────────────┘                                    └─────────────────┘
```

**왜 UDP인가?**

- **낮은 지연시간**: TCP의 3-way handshake, 재전송 대기가 없음. 로봇 제어는 ms 단위의 지연도 민감함
- **패킷 유실 허용**: 조이스틱 데이터는 50Hz로 계속 갱신되므로 한두 패킷이 유실되어도 바로 다음 패킷이 보상함
- **단순 구현**: 연결 관리(connect/accept)가 불필요. 송신측은 `sendto()`, 수신측은 `bind()` + `recvfrom()`이면 끝

---

## 2. 무엇을 (What) — 전체 아키텍처

### 2.1 구성 요소

시스템은 딱 2개의 컴포넌트로 구성된다:

```
[서버 PC]                                      [로봇 PC]
joystick_sender.py                             input_handler.py :: NetworkInput
━━━━━━━━━━━━━━━━                               ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
pygame.Joystick                                 socket.recvfrom()
    ↓                                               ↓
축/버튼 읽기                                     JSON 파싱
    ↓                                               ↓
JSON 직렬화                                      deadzone + 스케일링
    ↓                                               ↓
socket.sendto() ──── UDP (9870) ────►           TeleopCommand 생성
                                                     ↓
                                                제어 루프 (servoJ / 토크)
```

### 2.2 데이터 흐름 상세

1. **서버PC**: pygame으로 조이스틱의 raw 축/버튼 값을 읽음
2. **직렬화**: `{"axes": [...], "buttons": [...], "hat": [hx, hy]}` JSON 문자열로 인코딩
3. **전송**: UDP 소켓으로 로봇PC IP:9870에 송신 (50Hz)
4. **수신**: 로봇PC의 NetworkInput이 non-blocking 소켓으로 수신
5. **변환**: raw 값 → deadzone 적용 → 스케일링 → `TeleopCommand` 생성
6. **소비**: teleop_admittance 또는 teleop_impedance의 제어 루프가 `get_command()`로 소비

### 2.3 InputHandler 추상화

NetworkInput은 `InputHandler` ABC를 구현한다. 동일 인터페이스를 KeyboardInput, XboxInput도 구현하므로 `--input network`만 바꾸면 코드 변경 없이 입력 소스를 교체할 수 있다:

```python
# create_input() 팩토리 (input_handler.py:404-417)
def create_input(input_type: str, ...) -> InputHandler:
    if input_type == "keyboard":
        return KeyboardInput(...)
    elif input_type == "xbox":
        return XboxInput(...)
    elif input_type == "network":
        return NetworkInput(port=network_port, ...)
```

세 입력 모두 동일한 `TeleopCommand`를 반환하므로 제어 루프 코드는 입력 방식을 전혀 모른다.

---

## 3. 어떻게 (How) — 코드 상세 분석

### 3.1 Sender: `core/joystick_sender.py`

79줄짜리 단일 스크립트. 서버PC에서 독립 실행한다.

#### 초기화

```python
# pygame 조이스틱 초기화
pygame.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)  # 첫 번째 조이스틱
js.init()

# UDP 소켓 생성 (연결 설정 불필요)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = (args.target_ip, args.port)  # 로봇 PC 주소
dt = 1.0 / args.hz                    # 기본 50Hz → 20ms 주기
```

#### 메인 루프

```python
while True:
    t_start = time.perf_counter()

    pygame.event.pump()  # pygame 이벤트 큐 갱신 (필수!)

    # raw 값 수집 — 가공 없이 그대로
    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
    hat = list(js.get_hat(0)) if js.get_numhats() > 0 else [0, 0]

    # JSON 직렬화 + UDP 전송
    pkt = json.dumps({"axes": axes, "buttons": buttons, "hat": hat})
    sock.sendto(pkt.encode(), target)

    # 정확한 주기 유지
    elapsed = time.perf_counter() - t_start
    remaining = dt - elapsed
    if remaining > 0:
        time.sleep(remaining)
```

**핵심 설계 결정:**

- **raw 값 전송**: deadzone·스케일링을 Sender에서 하지 않음 → 수신측에서 조정 가능
- **`pygame.event.pump()`**: 호출하지 않으면 OS가 pygame을 "응답 없음"으로 처리하여 축 값이 갱신되지 않음
- **5초마다 로그**: `send_count % (hz * 5) == 0`으로 연결 상태를 시각적으로 확인

#### 실행 방법

```bash
# 서버 PC에서 (조이스틱이 연결된 PC)
pip install pygame
python3 joystick_sender.py --target-ip 192.168.0.10 --port 9870 --hz 50
```

---

### 3.2 Receiver: `NetworkInput` (input_handler.py:284-401)

로봇PC의 teleop 프로세스 안에서 실행되는 클래스.

#### 소켓 설정

```python
def setup(self):
    self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._sock.bind(("0.0.0.0", self._port))  # 모든 인터페이스에서 수신
    self._sock.setblocking(False)              # 핵심! non-blocking 모드
```

**`setblocking(False)`의 의미:**

`recvfrom()`을 호출했을 때 데이터가 없으면 블로킹(대기)하지 않고 즉시 `BlockingIOError`를 발생시킨다. 이것이 제어 루프(125Hz)를 멈추지 않게 하는 핵심이다.

#### 버퍼 Drain 패턴

```python
def get_command(self, timeout: float = 0.02) -> TeleopCommand:
    # 버퍼에 쌓인 패킷을 모두 읽고 마지막 것만 사용
    data = None
    try:
        while True:
            data, _ = self._sock.recvfrom(4096)
    except BlockingIOError:
        pass  # 더 이상 패킷이 없음 → 루프 탈출
```

**왜 drain하는가?**

Sender는 50Hz로 보내지만 Receiver의 `get_command()`가 항상 50Hz로 호출되는 것은 아니다. IK 계산이 오래 걸리면 OS 커널의 UDP 수신 버퍼에 패킷이 쌓인다. **오래된 패킷은 의미 없는 과거 입력**이므로, 가장 최신 패킷만 사용하는 것이 올바른 설계다.

```
시간 →  [pkt1] [pkt2] [pkt3] [pkt4]  ← 버퍼에 4개 쌓임
         ↓      ↓      ↓      ↓
        버림   버림   버림   사용!     ← 마지막 것만 TeleopCommand로 변환
```

#### 버튼 처리

```python
axes = pkt.get("axes", [])
buttons = pkt.get("buttons", [])

# 안전 함수: 인덱스 범위 초과 방어
def btn(idx):
    return buttons[idx] if idx < len(buttons) else 0

# 우선순위가 높은 버튼은 먼저 처리하고 즉시 반환
if btn(8):  cmd.estop = True;  return cmd   # E-Stop (최우선)
if btn(7):  cmd.reset = True;  return cmd   # Reset
if btn(6):  cmd.quit = True;   return cmd   # Quit
```

**E-Stop이 최우선**: 버튼 8(Logitech 버튼)이 눌리면 속도 계산 없이 즉시 E-Stop 명령을 반환한다.

#### 축 → 속도 변환

```python
# Deadzone 적용
def dz(val, threshold=0.1):
    return val if abs(val) > threshold else 0.0

# 아날로그 스틱 매핑 (XInput 표준)
lx = dz(axis(0))           # 왼쪽 스틱 X → 로봇 X축
ly = dz(-axis(1))          # 왼쪽 스틱 Y → 로봇 Y축 (Y축 반전!)
lt = (axis(2) + 1.0) / 2.0 # 왼쪽 트리거 → 0~1 정규화
rt = (axis(5) + 1.0) / 2.0 # 오른쪽 트리거 → 0~1 정규화
vz = dz(rt - lt, 0.05)     # 트리거 차이 → Z축 (위/아래)
rx = dz(axis(3))           # 오른쪽 스틱 X → Roll
ry = dz(-axis(4))          # 오른쪽 스틱 Y → Pitch

# 범퍼 → Yaw
lb = 1.0 if btn(4) else 0.0
rb = 1.0 if btn(5) else 0.0
wyaw = rb - lb

# 최종 속도 벡터 [vx, vy, vz, wx, wy, wz]
s = self.speed_scale
cmd.velocity = np.array([
    lx * self._linear_scale * s,    # X 병진
    ly * self._linear_scale * s,    # Y 병진
    vz * self._linear_scale * s,    # Z 병진
    -rx * self._angular_scale * s,  # Roll (부호 반전)
    ry * self._angular_scale * s,   # Pitch
    wyaw * self._angular_scale * s, # Yaw
])
```

**Deadzone이 필요한 이유:**

아날로그 스틱은 손을 떼도 정확히 0.0을 반환하지 않는다 (예: 0.03). deadzone(0.1) 이내의 값을 0으로 처리하여 로봇이 미세하게 drift하는 것을 방지한다.

#### 속도 스케일 (D-pad)

```python
hat = pkt.get("hat", [0, 0])
hy = hat[1]
if hy > 0:    # D-pad ↑ = 속도 증가
    self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
elif hy < 0:  # D-pad ↓ = 속도 감소
    self._speed_idx = max(self._speed_idx - 1, 0)
```

`SPEED_SCALES = [0.5, 1.0, 2.0, 4.0, 8.0]` — D-pad 위/아래로 5단계 속도 조절.

---

### 3.3 패킷 포맷 정리

```json
{
    "axes": [0.0, -0.52, -1.0, 0.13, 0.0, -1.0],
    "buttons": [0, 0, 0, 0, 1, 0, 0, 0, 0],
    "hat": [0, 1]
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `axes` | `float[]` | 아날로그 축 (-1.0 ~ 1.0). 개수는 조이스틱 모델에 따라 다름 (보통 6개) |
| `buttons` | `int[]` | 디지털 버튼 (0 또는 1). 개수는 모델에 따라 다름 (보통 9~11개) |
| `hat` | `int[2]` | D-pad [수평, 수직]. 각각 -1, 0, 1 |

**XInput 축 매핑 (Xbox 호환 게임패드 기준):**

| 인덱스 | 축 | 로봇 동작 |
|--------|-----|----------|
| 0 | 왼쪽 스틱 X | X축 병진 |
| 1 | 왼쪽 스틱 Y | Y축 병진 (반전) |
| 2 | 왼쪽 트리거 | Z축 하강 (정규화 후) |
| 3 | 오른쪽 스틱 X | Roll 회전 |
| 4 | 오른쪽 스틱 Y | Pitch 회전 (반전) |
| 5 | 오른쪽 트리거 | Z축 상승 (정규화 후) |

**버튼 매핑:**

| 인덱스 | 버튼 | 동작 |
|--------|------|------|
| 1 | B | 어드미턴스 프리셋 순환 |
| 3 | Y | F/T 센서 영점 |
| 4 | LB | Yaw 회전 (음) |
| 5 | RB | Yaw 회전 (양) |
| 6 | Back | 종료 (Quit) |
| 7 | Start | 리셋 (Reset) |
| 8 | Logitech/Xbox | 비상 정지 (E-Stop) |

---

### 3.4 Teleop 시스템에서의 사용

#### teleop_admittance 설정 (`config/default.yaml`)

```yaml
input:
  network_port: 9870
  network_linear_scale: 0.005    # xbox(0.02)보다 낮음!
  network_angular_scale: 0.015   # xbox(0.05)보다 낮음!
```

**스케일이 작은 이유:** 네트워크 입력은 패킷이 연속으로 도착하여 누적 효과가 크다. Xbox 직접 연결은 이벤트 기반이지만 네트워크는 50Hz 스트림이므로 같은 스케일을 사용하면 로봇이 너무 빠르게 움직인다.

#### 실행 방법

```bash
# 로봇 PC (수신)
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m standalone.teleop_admittance.main --mode rtde --input network

# 서버 PC (송신)
python3 joystick_sender.py --target-ip <로봇PC_IP> --port 9870 --hz 50
```

---

## 4. 핵심 설계 패턴 요약

### 4.1 관심사 분리

| 레이어 | 역할 | 로봇 제어 로직을 알고 있는가? |
|--------|------|------------------------------|
| `joystick_sender.py` | raw 입력 수집 + 네트워크 전송 | **아니오** — 축/버튼만 전달 |
| `NetworkInput` | 패킷 수신 + TeleopCommand 변환 | **아니오** — 속도 벡터만 생성 |
| `teleop_admittance/main.py` | IK + 안전 + servoJ 실행 | **예** |

Sender가 로봇 로직을 모르기 때문에:
- 어떤 로봇이든 재사용 가능
- 버튼 매핑을 바꾸려면 NetworkInput만 수정하면 됨
- Sender는 pip install pygame만 있으면 아무 PC에서 실행 가능

### 4.2 Non-blocking + Buffer Drain

```
제어 루프 (125Hz)
    │
    ├─ IK 계산 (가변 시간)
    ├─ 안전 검사
    ├─ servoJ 전송
    └─ get_command()  ← 여기서 블로킹되면 루프 전체가 멈춤!
                         → non-blocking + drain으로 즉시 반환
```

### 4.3 Factory 패턴으로 입력 교체

```python
# CLI 인자 하나로 입력 소스 교체
parser.add_argument("--input", choices=["keyboard", "xbox", "network"])

# 내부에서는 Factory가 적절한 InputHandler를 생성
self.input_handler = create_input(input_type=args.input, ...)
```

제어 루프 코드는 `get_command()` 호출만 하면 되므로, 입력 방식이 바뀌어도 수정이 필요 없다.

---

## 5. 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `[ERROR] No joystick found` | 조이스틱이 서버PC에 연결되지 않음 | USB 연결 확인, `ls /dev/input/js*` |
| 로봇이 반응 없음 | 방화벽이 UDP 9870 차단 | `sudo ufw allow 9870/udp` |
| 로봇 움직임이 끊김 | 네트워크 지연/패킷 유실 | 유선 연결 확인, Wi-Fi 대신 이더넷 사용 |
| 로봇이 너무 빠르게 움직임 | `network_linear_scale`이 너무 큼 | YAML에서 스케일 값을 줄임 (기본 0.005) |
| 축 방향이 반대 | 조이스틱 모델마다 축 매핑이 다름 | Sender에서 `axes` 로그 확인 후 NetworkInput 매핑 수정 |
