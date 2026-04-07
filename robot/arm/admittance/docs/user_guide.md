# Teleop Admittance 사용자 가이드

UR10e 어드미턴스 원격조종 프로그램의 설치부터 실행, 조작, 실제 로봇 연결까지 단계별로 안내합니다.

---

## 목차

1. [개요](#1-개요)
2. [사전 준비](#2-사전-준비)
3. [빠른 시작 (첫 실행)](#3-빠른-시작-첫-실행)
4. [키보드 조작법](#4-키보드-조작법)
5. [Unified 입력 (원격 sender)](#5-unified-입력-원격-sender)
6. [어드미턴스 (F/T) 사용법](#6-어드미턴스-ft-사용법)
7. [안전 시스템](#7-안전-시스템)
8. [실제 로봇 연결](#8-실제-로봇-연결)
9. [트러블슈팅](#9-트러블슈팅)
10. [다음 단계](#10-다음-단계)

---

## 1. 개요

이 프로그램은 키보드 또는 Unified 프로토콜(Vive/키보드 sender)로 UR10e 로봇의 엔드이펙터(EE)를 실시간으로 움직이는 **원격조종(Teleop)** 도구입니다. 선택적으로 F/T(힘/토크) 센서 기반 **어드미턴스 제어**를 활성화하면, 외부에서 로봇을 손으로 밀어서 움직이는 것도 가능합니다.

**두 가지 모드**:
- **sim 모드**: 실제 로봇 없이 시뮬레이션으로 테스트 (ROS2 mock hardware 사용)
- **rtde 모드**: 실제 UR10e 로봇을 125Hz로 실시간 제어

---

## 2. 사전 준비

### 2.1 Python 의존성 설치

```bash
pip install pin-pink proxsuite
pip install "numpy<2"
```

> **주의**: `pip install pink`는 코드 포매터입니다! 반드시 **`pin-pink`** 를 설치하세요.
>
> **주의**: numpy 2.x는 pinocchio와 호환되지 않습니다. 반드시 `numpy<2`를 설치하세요.

### 2.2 URDF 파일 확인

다음 파일이 존재하는지 확인합니다:

```bash
ls /workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf
```

이 파일은 로봇 모델 정보를 담고 있으며, 프로그램이 자동으로 참조합니다.

### 2.3 모드별 추가 준비

#### sim 모드 (시뮬레이션)

별도의 터미널에서 ROS2 mock hardware 드라이버를 먼저 실행해야 합니다:

```bash
# Terminal 1 — mock hardware 드라이버
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
```

이 터미널은 프로그램 종료 시까지 계속 실행 중이어야 합니다.

#### rtde 모드 (실제 로봇)

- 로봇 IP 주소 확인 (기본: `192.168.0.2`)
- 로봇 전원 ON, 브레이크 해제 완료
- **E-Stop 물리 버튼** 위치 확인 (비상 시 즉시 누를 수 있도록)
- PC와 로봇이 같은 네트워크에 연결되어 있는지 확인

---

## 3. 빠른 시작 (첫 실행)

처음 실행하는 분은 아래 순서대로 따라하세요. **sim 모드**로 시작하면 실제 로봇 없이 안전하게 테스트할 수 있습니다.

### Step 1: 의존성 설치

```bash
pip install pin-pink proxsuite "numpy<2"
```

### Step 2: mock hardware 실행 (Terminal 1)

새 터미널을 열고:

```bash
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
```

약 5~10초 후 `[controller_manager]` 로그가 나오면 준비 완료입니다.

### Step 3: 프로그램 실행 (Terminal 2)

다른 터미널에서:

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m robot.arm.admittance.main --mode sim --input keyboard
```

아래와 같은 출력이 나타나면 성공입니다:

```
[Teleop] Mode: sim | Input: keyboard | Freq: 50Hz | dt: 20.0ms
[Teleop] Initial EE: x=0.3912 y=0.1092 z=0.5940
=== UR10e Teleop Servo (Pink IK) ===
  W/S : Fwd/Back  U/O : Roll +/-
  ...
```

### Step 4: 로봇 움직여보기

- `W` / `S` → 앞/뒤 이동
- `A` / `D` → 좌/우 이동
- `Q` / `E` → 위/아래 이동
- 터미널 상태 표시에서 EE 좌표가 변하는 것을 확인하세요

### Step 5: 종료

`ESC` 또는 `X` 키를 누르면 프로그램이 종료됩니다.

> **축하합니다!** 첫 실행에 성공했습니다. 아래에서 상세 조작법을 확인하세요.

---

## 4. 키보드 조작법

### 이동 (병진)

| 키 | 동작 | 방향 |
|----|------|------|
| `W` | 앞으로 | +Y (로봇에서 멀어지는 방향) |
| `S` | 뒤로 | -Y |
| `A` | 왼쪽 | -X |
| `D` | 오른쪽 | +X |
| `Q` | 위로 | +Z |
| `E` | 아래로 | -Z |
| `C` | EE 앞으로 | +Z (도구 축 기준) |
| `V` | EE 뒤로 | -Z (도구 축 기준) |

> **W/S/A/D/Q/E**는 로봇 베이스 기준으로 움직입니다.
> **C/V**는 현재 EE가 바라보는 방향(도구 Z축)으로 움직입니다. EE 자세가 바뀌면 이동 방향도 바뀝니다.

### 회전

| 키 | 동작 |
|----|------|
| `U` / `O` | Roll + / - (X축 회전) |
| `I` / `K` | Pitch + / - (Y축 회전) |
| `J` / `L` | Yaw + / - (Z축 회전) |

### 속도 조절

| 키 | 동작 |
|----|------|
| `+` (또는 `=`) | 속도 올리기 (0.5x → 1x → 2x → 4x → 8x) |
| `-` | 속도 내리기 |

현재 속도 배율은 터미널 상태 표시의 `Speed:` 항목에서 확인할 수 있습니다.

> **팁**: 처음에는 1x로 시작하고, 익숙해지면 `+`로 올리세요. 정밀한 작업이 필요하면 `-`로 0.5x까지 줄일 수 있습니다.

### 기타

| 키 | 동작 |
|----|------|
| `Space` | **E-Stop** (비상 정지) |
| `R` | E-Stop 해제 + 상태 초기화 |
| `ESC` 또는 `X` | 프로그램 종료 |
| `T` | 어드미턴스 ON/OFF 토글 |
| `Z` | F/T 센서 영점 보정 |
| `1` / `2` / `3` / `4` | 어드미턴스 프리셋 (STIFF/MEDIUM/SOFT/FREE) |

---

## 5. Unified 입력 (원격 sender)

Operator PC에서 Vive Tracker, 키보드, 조이스틱 sender가 UDP로 보내는 절대 포즈를 수신합니다.

```bash
# Unified 프로토콜로 실행 (operator PC의 sender가 UDP 9871로 전송)
python3 -m robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2
```

- Operator PC에서 sender 실행: `python3 -m sender.arm.vive_sender --target-ip <ROBOT_PC_IP>`
- 버튼 매핑은 sender 측에서 처리 (E-Stop, Reset, 속도 조절, 어드미턴스 프리셋 등)
- 로봇 PC는 수신만 하며, sender 시작 시 현재 TCP 포즈를 자동 응답

> **참고**: Unified 모드에서는 입력이 절대 포즈(absolute pose)로 전달되므로, 키보드 모드의 누적 방식과 다릅니다.

---

## 6. 어드미턴스 (F/T) 사용법

어드미턴스 제어를 활성화하면 F/T 센서가 감지한 외력에 따라 로봇이 순응적으로 움직입니다. 예를 들어, EE를 손으로 밀면 밀리는 방향으로 이동합니다.

### 6.1 기본 사용법

1. **활성화**: `T` 키를 눌러 어드미턴스 ON (터미널에 `Admit: ON [MEDIUM]` 표시)
2. **영점 보정**: `Z` 키를 눌러 F/T 센서 영점 맞추기 (로봇이 정지 상태일 때)
3. **프리셋 전환**: `1`~`4` 키로 순응 강도 조절
4. **비활성화**: `T` 키를 다시 눌러 OFF

### 6.2 프리셋 선택 가이드

| 키 | 프리셋 | 느낌 | 언제 쓸까? |
|----|--------|------|-----------|
| `1` | **STIFF** | 단단함, 잘 안 밀림 | 정밀한 위치 유지가 중요할 때 |
| `2` | **MEDIUM** | 중간 | 일반적인 작업 (기본값) |
| `3` | **SOFT** | 부드러움, 잘 밀림 | 섬세한 접촉이 필요한 작업 |
| `4` | **FREE** | 자유롭게 밀림 | 손으로 로봇을 직접 안내할 때 (핸드 가이딩) |

> **STIFF → FREE로 갈수록** 외력에 더 잘 순응합니다. FREE는 강성이 0이라서 밀면 그 자리에 머물고, 원래 위치로 돌아오지 않습니다.

### 6.3 주의사항

- **sim 모드에서는 어드미턴스가 동작하지 않습니다**. 실제 F/T 센서가 없으므로 항상 0이 반환됩니다. 어드미턴스를 테스트하려면 `--mode rtde`로 실제 로봇에 연결하세요.
- `Z` (영점 보정)는 **로봇이 정지한 상태에서** 눌러야 합니다. 움직이는 중에 누르면 관성력이 바이어스로 잡힙니다.
- 큰 힘(100N 이상)이 감지되면 자동으로 상태가 리셋됩니다 (충돌 보호).

---

## 7. 안전 시스템

프로그램에는 4단계 안전 시스템이 내장되어 있습니다.

### E-Stop (비상 정지) — 가장 중요!

| 동작 | 키 |
|------|-----|
| **E-Stop 발동** | `Space` (키보드) / sender 측 E-Stop 버튼 |
| **E-Stop 해제** | `R` (키보드) / sender 측 Reset 버튼 |

E-Stop을 발동하면 로봇이 즉시 정지하고, 어떤 입력도 무시됩니다. 해제하면 현재 위치에서 다시 제어가 시작됩니다.

> **실제 로봇 사용 시**: 소프트웨어 E-Stop 외에 **물리 E-Stop 버튼**도 항상 손이 닿는 곳에 두세요. 소프트웨어는 완전히 신뢰할 수 없습니다.

### 자동 안전 기능

| 기능 | 설명 | 복구 |
|------|------|------|
| **입력 타임아웃** | 200ms 이상 입력이 없으면 현재 위치에서 정지 | 입력 재개 시 자동 복구 |
| **속도 제한** | 관절 속도가 0.5 rad/s를 초과하면 자동 감속 | 자동 (비례 축소) |
| **작업 공간 제한** | EE가 설정된 범위를 벗어나지 못하게 제한 | 자동 (범위 내 클램핑) |

터미널 상태 표시에서 `Safety:` 항목으로 현재 상태를 실시간 확인할 수 있습니다.

---

## 8. 실제 로봇 연결

sim 모드에서 충분히 연습한 후, 실제 로봇에 연결하세요.

### 체크리스트

- [ ] 로봇 전원 ON, 브레이크 해제
- [ ] 로봇 IP 확인 (기본: `192.168.0.2`)
- [ ] PC와 로봇이 같은 네트워크에 연결됨
- [ ] **물리 E-Stop 버튼** 위치 확인 (손이 닿는 곳)
- [ ] 로봇 주변에 사람/장비물 없는지 확인
- [ ] UR 티치펜던트에서 Remote Control 모드 활성화

### 실행

```bash
cd /workspaces/tamp_ws/src/teleop_dev

# 기본 IP (192.168.0.2)
python3 -m robot.arm.admittance.main --mode rtde --input keyboard

# 다른 IP
python3 -m robot.arm.admittance.main --mode rtde --input keyboard --robot-ip 192.168.0.100

# Xbox 컨트롤러
python3 -m robot.arm.admittance.main --mode rtde --input xbox --robot-ip 192.168.0.2
```

### 첫 연결 시 권장 순서

1. **저속으로 시작**: `-` 키를 눌러 0.5x 속도로 설정
2. **작은 동작 테스트**: `W`/`S`로 앞뒤 1~2cm만 이동해보기
3. **회전 테스트**: `U`/`O`로 Roll 소량 테스트
4. **작업 공간 확인**: 로봇이 벽이나 테이블에 닿지 않는지 확인
5. **어드미턴스 테스트** (F/T 사용 시):
   - `Z` 키로 F/T 영점 보정 (로봇 정지 상태에서)
   - `T` 키로 어드미턴스 활성화
   - `1` (STIFF)부터 시작 → 이상 없으면 `2` (MEDIUM) → `3` (SOFT)

### CSV 로깅

동작을 기록하고 싶다면 `--log` 옵션을 추가합니다:

```bash
python3 -m robot.arm.admittance.main --mode rtde --input keyboard --log
```

`teleop_log_YYYYMMDD_HHMMSS.csv` 파일이 생성되며, EE 위치/RPY/관절 각도/속도/안전 상태가 기록됩니다.

---

## 9. 트러블슈팅

### `ModuleNotFoundError: No module named 'pink'`

**원인**: `pip install pink` (코드 포매터)를 설치한 경우.

```bash
pip uninstall pink
pip install pin-pink proxsuite
```

### `ValueError` 또는 segfault (numpy 관련)

**원인**: numpy 2.x와 pinocchio의 ABI 비호환.

```bash
pip install "numpy<2"
```

### `[WARN] controller_manager not available` (sim 모드)

**원인**: Terminal 1의 mock hardware 드라이버가 실행되지 않았거나 아직 초기화 중.

**해결**: Terminal 1을 먼저 실행하고 5~10초 기다린 후 재시도.

### 어드미턴스를 켜도 반응이 없음

**원인**: sim 모드에서는 F/T 센서가 항상 0을 반환합니다.

**해결**: 정상입니다. 어드미턴스를 실제로 테스트하려면 `--mode rtde`로 실행하세요.

### 로봇이 움직이지 않음 (rtde 모드)

확인 사항:
1. 로봇 브레이크가 해제되었는지 확인
2. UR 티치펜던트에서 Remote Control 모드가 활성화되었는지 확인
3. `ping 192.168.0.2` (로봇 IP)로 네트워크 연결 확인
4. 다른 프로그램이 RTDE 포트를 점유하고 있지 않은지 확인

### E-Stop 후 복구가 안 됨

`R` 키를 눌러도 복구되지 않으면:
1. 프로그램을 종료 (`ESC`)하고 다시 실행
2. 실제 로봇인 경우 티치펜던트에서 상태 확인

---

## 10. 다음 단계

기본 사용법을 익혔다면, 아래 문서에서 더 깊이 있는 내용을 확인하세요.

| 문서 | 내용 | 대상 |
|------|------|------|
| [config_guide.md](config_guide.md) | 설정 항목 전체 레퍼런스, 시나리오별 튜닝 가이드 | 설정 커스터마이징 |
| [admittance_theory.md](admittance_theory.md) | 어드미턴스 이론, 좌표축/로테이션 처리, 핵심 코드 | 구현 원리 이해 |
| [manual.md](manual.md) | 코드 구조, 모듈별 학습 매뉴얼 | 코드 수정/확장 |

### 자주 사용하는 실행 명령어 모음

```bash
cd /workspaces/tamp_ws/src/teleop_dev

# sim + 키보드 (기본, 처음 시작할 때)
python3 -m robot.arm.admittance.main --mode sim --input keyboard

# 실제 로봇 + 키보드
python3 -m robot.arm.admittance.main --mode rtde --input keyboard --robot-ip 192.168.0.2

# 실제 로봇 + unified (operator PC sender 수신) + 로깅
python3 -m robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2 --log

# 커스텀 설정 파일 사용
python3 -m robot.arm.admittance.main --config robot/arm/admittance/config/my_config.yaml

# 안전 시스템 없이 테스트 (개발용)
python3 -m robot.arm.admittance.teleop_nosafety --mode sim --input keyboard

# 순수 어드미턴스 테스트 (키보드 입력 없이, 외력만으로 제어)
python3 -m robot.arm.admittance.test_admittance --robot-ip 192.168.0.2
```
