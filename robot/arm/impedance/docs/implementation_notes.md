# UR10e 임피던스 제어 — 구현 노트 및 디버깅 기록

> **상태**: `test_impedance_bare` 실제 로봇 동작 확인 완료 (2026-03)
> **위치**: `robot/arm/impedance/`
> **관련 문서**: [manual.md](manual.md) (원래 설계), [user_guide.md](user_guide.md) (사용법)

---

## 1. 무엇을 만들었나

UR10e 로봇을 위한 **관절 공간 임피던스 제어** 시스템.

로봇이 외력에 대해 물리적으로 순응(compliant)하게 동작한다:
- 밀면 밀리고, 놓으면 원래 위치로 돌아온다
- 게인(Kp/Kd)으로 강성/순응성을 조절할 수 있다
- F/T 센서 없이 순수 위치 오차 기반으로 동작한다

### PD 토크 제어 공식

```
tau = Kp * (q_desired - q_actual) - Kd * q_dot_actual + C(q, q_dot)
```

- `Kp * (q_d - q)` : 가상 스프링 (목표에서 벗어나면 되돌리는 힘)
- `-Kd * q_dot` : 가상 댐퍼 (진동 방지)
- `C(q, q_dot)` : 코리올리스/원심력 보상 (Pinocchio로 계산)
- 중력 보상 : `direct_torque()` API가 자동 처리

---

## 2. 어떤 방식을 사용했나 — 실제 구현 아키텍처

### 원래 계획 vs 실제 구현

| 항목 | 원래 계획 (manual.md) | 실제 구현 |
|------|----------------------|----------|
| PD 계산 위치 | URScript (500Hz) | **Python** (125Hz) |
| URScript 역할 | PD 법칙 계산 + direct_torque | **단순 릴레이** (레지스터 읽기 + direct_torque) |
| 스크립트 업로드 | RTDEControlInterface | **TCP 소켓 (port 30002)** |
| 레지스터 레이아웃 | double 0~19 (20개) | **double 18~22 + int 18,19** (혼합) |
| 코리올리스 계산 | URScript 내장 함수 | **Python Pinocchio** |
| RTDE 인터페이스 | Receive + Control + IO | **Receive + IO만** (Control 미사용) |

### 왜 계획을 바꿨나

**RTDEControlInterface가 우리 UR10e에서 hang된다.**

`rtde_control.RTDEControlInterface(ip)` 호출 시 무한 대기. 원인 불명 (펌웨어? 네트워크? 다른 RTDE 연결 충돌?). 이 인터페이스 없이는:
- `setCustomScriptFile()` 사용 불가 → TCP 소켓으로 대체
- `servoJ()`, `speedJ()` 사용 불가 → `direct_torque()`는 URScript 내부에서 호출
- `initPeriod()`, `waitPeriod()` 사용 불가 → `time.perf_counter()`로 대체

### 실제 아키텍처 다이어그램

```
Python (125Hz)                              UR Controller (500Hz)
=============                               =====================

[입력] Keyboard/Xbox                        impedance_pd.script
    |                                        ┌──────────────────┐
    v                                        │ thread torqueThread()
[ExpFilter] 위치/자세 스무딩                 │   while running:
    |                                        │     if active:
    v                                        │       direct_torque(cmd_tau)
[Pink IK] Cartesian → q_desired            │     else:
    |                                        │       direct_torque(zero)
    v                                        │   end
[PD 토크 계산]                              │ end
  tau = Kp*(q_d-q) - Kd*qd + coriolis      │
    |                                        │ main loop:
    v                                        │   mode = read_int_reg(19)
[RTDE 레지스터 쓰기]                         │   if mode > 0:
  double 18~22: tau[0..4]                   │     cmd_tau = read_regs()
  int 18: tau[5] * 1000    ──────────────►  │   end
  int 19: mode (0/1/-1)                     │   sync()
    |                                        └──────────────────┘
    v
[RTDEReceiveInterface]
  q_actual, qd_actual     ◄──── RTDE 상태 읽기
```

### 핵심 설계 결정 사항

**1. Python-side PD 계산**
- Pink IK, Pinocchio 등 Python 라이브러리 활용 가능
- 런타임 게인 튜닝 용이 (프리셋 전환, 스케일 조절)
- 125Hz로도 충분 (URScript가 500Hz에서 마지막 토크 명령을 반복 적용)

**2. TCP 소켓 (port 30002) 스크립트 업로드**
- UR Secondary Interface: 어떤 URScript든 즉시 업로드+실행 가능
- RTDEControlInterface 불필요
- 단점: 에러 메시지 없이 실패할 수 있음 (아래 디버깅 섹션 참조)

**3. 혼합 레지스터 레이아웃**
- RTDEIOInterface는 lower range [0-23] 또는 upper range [24-47] 선택
- RTDEControlInterface 없이는 lower range만 안정적 사용 가능
- double 레지스터 5개 (tau[0..4]) + int 레지스터 2개 (tau[5], mode)
- Joint 6 토크: int 레지스터에 millitorque로 인코딩 (x1000, 0.001 Nm 정밀도)

**4. Pinocchio 코리올리스 보상**
- URScript에는 행렬 연산 기능 없음
- `pin.computeCoriolisMatrix(model, data, q, qd)` → `data.C @ qd`
- Python에서 ~1ms 이내 계산, RTDE 레지스터로 전송

---

## 3. 사용 방법

### 최소 테스트 (position hold)

```bash
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m robot.arm.impedance.test_impedance_bare --robot-ip 192.168.0.2 --preset SOFT
```

로봇이 현재 위치를 유지. 손으로 밀면 저항 후 원위치 복귀.

### 전체 텔레옵

```bash
# 실제 로봇
python3 -m robot.arm.impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2

# 시뮬레이션 (위치 제어 fallback)
python3 -m robot.arm.impedance.main --mode sim --input keyboard
```

### 게인 프리셋

| 프리셋 | Kp (J1-J2) | 특성 | 용도 |
|--------|-----------|------|------|
| SOFT | 100 | 쉽게 밀림 | 초기 테스트, 안전 우선 |
| MEDIUM | 400 | 중간 저항 | 일반 사용 |
| STIFF | 800 | 강한 저항 | 정밀 위치 추종 |

런타임 조작: `1/2/3` 프리셋 전환, `[/]` 게인 스케일 (0.25x~2.0x)

---

## 4. 디버깅 기록 — 겪었던 문제들과 해결

이 섹션이 이 문서의 핵심 가치. URScript + RTDE 조합에서 겪을 수 있는 함정들.

### 문제 1: URScript 무반응 (silent parse failure)

**증상**: 스크립트 업로드 후 UR 패널에 아무 로그도 안 뜸. 에러 메시지도 없음.

**원인**: `#` 주석이 포함된 URScript를 port 30002로 전송하면 **조용히 거부**됨.

UR Secondary Interface (port 30002)는 URScript 파싱 에러 시 아무런 피드백 없이 스크립트를 무시한다. 우리가 확인한 파싱 실패 원인:

1. **`#` 주석**: 모든 `#` 주석 라인이 파싱 에러를 유발
2. **조건문 내부 thread 정의**: `if/else` 블록 안에 `thread` 정의하면 실패 가능
3. **`return` 키워드**: URScript에 `return` 없음 → `break` + 제어 흐름으로 대체
4. **다중행 배열 리터럴**: 한 줄로 작성해야 함

**해결**: 동작 확인된 test E (test_torque_diag.py)의 구조를 정확히 복제:
- 주석 전부 제거
- thread를 함수 스코프 최상위에 정의
- 평탄한(flat) 구조 유지

**교훈**: 새 URScript 작성 시, 반드시 동작하는 스크립트를 기반으로 점진적 수정. 한번에 큰 스크립트를 작성하면 어디서 파싱이 실패하는지 알 수 없다.

### 문제 2: str_cat() Type error

**증상**: URScript 실행 중 "Type error" 런타임 에러.

**원인**: `str_cat()`은 **정확히 2개 인자**만 받는다.

```urscript
// 이렇게 쓰면 에러:
str_cat("mode=", to_str(mode), " count=", to_str(count))

// 이렇게 중첩해야 함:
str_cat(str_cat(str_cat("mode=", to_str(mode)), " count="), to_str(count))
```

**교훈**: URScript 내장 함수의 시그니처를 UR 공식 문서에서 확인할 것.

### 문제 3: Non-ASCII 문자

**증상**: 스크립트가 조용히 거부됨 (문제 1과 동일 증상).

**원인**: 스크립트 파일에 em-dash (U+2014: `—`) 같은 non-ASCII 문자 포함.

Claude 등 AI 도구가 코드 생성 시 유니코드 문자를 삽입할 수 있다. `test_script_upload.py`의 non-ASCII 검사가 이를 감지:

```python
print(f"Non-ASCII: {any(ord(c) > 127 for c in script)}")
```

**해결**: `.script` 파일은 반드시 순수 ASCII만 포함해야 한다.

### 문제 4: 레지스터 레이스 컨디션 (가장 어려웠던 버그)

**증상**: 스크립트가 실행되고 "Starting controller", "torqueThread started" 메시지가 뜨지만, 바로 "got stop signal"로 종료. "alive" 메시지 없음.

**디버깅 과정**:

1. **첫 번째 가설 (오진)**: "업로드가 레지스터를 리셋한다"
   - 레지스터 초기화를 업로드 후로 이동 → 여전히 실패
   - 잘못된 가설이었음

2. **근본 원인 발견**: 레이스 컨디션
   ```
   t=0.000  _upload_script()      → 스크립트 즉시 실행 시작
   t=0.001  URScript: register 19 읽기 → -1 (이전 disconnect()의 잔여값)
   t=0.002  mode==-1 → "got stop signal" → 즉시 종료
     ...
   t=0.500  Python: time.sleep(0.5) 완료
   t=0.501  Python: register 19 = 0 설정 ← 이미 늦음!
   ```

   `disconnect()`가 `set_mode(-1)`을 호출하여 register 19에 -1을 씀.
   이 값은 다음 실행까지 **유지됨** (업로드가 레지스터를 리셋하지 않음).
   스크립트는 업로드 즉시 실행되어 stale -1을 읽고 종료.

3. **test E가 동작하는 이유**: register를 업로드 **전에** 0으로 설정하고, 업로드 **후에도** 계속 재설정함.

**해결 (3중 방어)**:

```python
# 1. 업로드 전 레지스터 초기화 (핵심: stale -1 제거)
self._io.setInputIntRegister(19, 0)
time.sleep(0.5)  # 전파 대기

# 2. 스크립트 업로드
self._upload_script()
time.sleep(1.0)

# 3. 업로드 후 재초기화 (안전망: 만약 업로드가 리셋하는 경우)
self._io.setInputIntRegister(19, 0)
```

```urscript
// 4. URScript startup grace period (500 cycles = ~1초)
local startup = 0
while True:
  local mode = read_input_integer_register(19)
  if startup < 500:
    startup = startup + 1
    mode = 0  // 처음 1초간 mode를 강제로 0으로
  end
  if mode == -1:
    break
  end
  // ...
end
```

**교훈**: RTDE 레지스터는 프로세스 간 공유 메모리와 같다. 읽기/쓰기 순서에 대한 가정을 하지 말 것. 방어적 프로그래밍(belt-and-suspenders) 필수.

---

## 5. URScript 문법 규칙 (경험적으로 확인된 사항)

UR Secondary Interface (port 30002)로 업로드하는 URScript에 적용되는 규칙:

| 규칙 | 위반 시 결과 | 에러 메시지 |
|------|-------------|------------|
| `#` 주석 사용 금지 | 스크립트 무시 (무반응) | 없음 |
| Non-ASCII 문자 금지 | 스크립트 무시 (무반응) | 없음 |
| `str_cat()` 인자 2개만 | 런타임 "Type error" | UR 패널에 표시 |
| thread는 함수 스코프에 정의 | 파싱 실패 가능 | 없음 |
| `return` 키워드 없음 | 파싱 실패 | 없음 |
| 배열 리터럴 한 줄 | 파싱 실패 가능 | 없음 |
| `def func() ... end` 필수 | 파싱 실패 | 없음 |
| 마지막에 `func()` 호출 필수 | 실행 안 됨 | 없음 |

**핵심 원칙**: port 30002는 에러를 알려주지 않는다. 동작하는 최소 스크립트에서 시작해서 점진적으로 수정할 것.

---

## 6. 테스트 도구 가이드

### test_impedance_bare.py — 최소 임피던스 테스트

```bash
python3 -m robot.arm.impedance.test_impedance_bare --robot-ip 192.168.0.2 --preset SOFT
```

IK, 입력, 안전 시스템 없이 순수 PD position-hold만 테스트.
성공 기준: UR 패널에 "alive, mode=1" 메시지, 로봇이 위치 유지.

### test_torque_diag.py — 5단계 컴포넌트 테스트

```bash
python3 -m robot.arm.impedance.test_torque_diag --robot-ip 192.168.0.2 --test ALL
```

| 테스트 | 검증 내용 | RTDE 필요 |
|--------|----------|----------|
| A | 스크립트 업로드 확인 | No |
| B | RTDE 레지스터 읽기 에코 | Yes (IO) |
| C | direct_torque + RTDE 연결 | Yes (Recv + IO) |
| D | Threaded direct_torque (RTDE 없음) | No |
| E | Thread torque + register read (전체 아키텍처 최소 재현) | Yes (IO) |

문제가 발생하면 A부터 순서대로 실행하여 어느 단계에서 실패하는지 격리.

### test_script_upload.py — 스크립트 내용 격리 테스트

```bash
python3 -m robot.arm.impedance.test_script_upload --robot-ip 192.168.0.2
```

RTDE 연결 없이 순수 TCP 소켓으로 `impedance_pd.script`를 업로드.
- UR 패널에 메시지 출력 → 스크립트 내용 OK (RTDE 문제 의심)
- 아무것도 안 뜸 → 스크립트 파싱 에러 (내용 수정 필요)

### test_urscript_upload.py — freedrive/torque 기본 테스트

```bash
python3 -m robot.arm.impedance.test_urscript_upload --robot-ip 192.168.0.2 --test torque
```

인라인 최소 스크립트로 `direct_torque()` API 동작 확인.

---

## 7. 기존 문서와의 관계

| 문서 | 설명하는 아키텍처 | 현재 상태 |
|------|------------------|----------|
| `manual.md` | **원래 계획**: URScript-side PD, RTDEControlInterface | 미구현 (참고용) |
| `user_guide.md` | **원래 계획** 기반 사용법 | 레지스터 맵 등 불일치 |
| **`implementation_notes.md` (이 문서)** | **실제 구현**: Python-side PD + URScript 릴레이 | 현재 동작하는 코드 |

주요 차이점:
- PD 계산 위치: URScript (계획) vs Python (실제)
- 레지스터 레이아웃: double 0~19 (계획) vs double 18~22 + int 18,19 (실제)
- 스크립트 업로드: RTDEControlInterface (계획) vs TCP 소켓 30002 (실제)
- 코리올리스: URScript `get_coriolis_and_centrifugal_torques()` (계획) vs Pinocchio (실제)

---

## 8. 커밋 히스토리 (관련 수정)

| 커밋 | 내용 |
|------|------|
| `f715921` | URScript를 test E 구조에 맞게 재작성 (주석 제거, 평탄 구조) |
| `49b8044` | 레지스터 초기화를 업로드 후로 이동 (첫 시도, 불충분) |
| `d8587bf` | 레이스 컨디션 수정: 업로드 전후 초기화 + startup grace period |
