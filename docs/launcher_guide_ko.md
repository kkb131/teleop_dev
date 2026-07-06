# 통합 실행기 (launcher) 사용 가이드

> dg5f 드라이버, UR10e 수신부, D405 카메라, XR sender 등 여러 터미널에
> 흩어져 있던 실행을 **GUI 한 곳에서** 시작/정지/모니터링한다.
> 조종 PC 의 GUI 가 robot PC 의 구성요소까지 원격 관리한다.

---

## 1. 구조

```
조종 PC                                      robot PC
┌───────────────────────────┐               ┌───────────────────────────┐
│  python3 -m launcher gui  │   HTTP 9876   │ python3 -m launcher agent │
│  --profile operator       │ ────────────→ │  (headless 데몬)          │
│                           │               │                           │
│  [operator group 로컬 실행]│               │  [robot group 로컬 실행]  │
│   - adb-reverse           │               │   - dg5f-right/left-driver│
│   - xr-dual-teleop        │               │   - hand-right/left-recv  │
│   - cam-viewer            │               │   - arm-right/left-recv   │
│                           │               │   - cam-server            │
└───────────────────────────┘               └───────────────────────────┘
```

- **launcher 는 기존 코드를 import 하지 않는다.** yaml 에 적힌 명령을
  subprocess 로 띄울 뿐이므로, launcher 의 버그/변경이 teleop 코드에
  영향을 주지 않는다 (역도 성립).
- 의존성: python3 표준 라이브러리 + PyYAML + tkinter (GUI 만).
  Ubuntu 에서 tkinter 가 없으면 `sudo apt install python3-tk`.
- 모든 구성요소/IP/포트는 [launcher/config/teleop_system.yaml](../launcher/config/teleop_system.yaml)
  한 파일에 정의 — **현장 배치 시 이 파일만 수정하면 된다.**

---

## 2. 빠른 시작

### robot PC (1회)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m launcher agent --config launcher/config/teleop_system.yaml
# → [agent] listening on http://0.0.0.0:9876 ...
```

부팅 시 자동 시작을 원하면 systemd/rc.local 에 위 명령 등록.

### 조종 PC

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m launcher gui --config launcher/config/teleop_system.yaml --profile operator
```

- robot PC 패널 = agent 를 통한 원격 제어, Operator 패널 = 이 PC 로컬 실행.
- agent 주소는 yaml `network.robot_pc_ip` + `agent.port` 로 계산.
  다른 주소면 `--agent-url http://<ip>:<port>`.

### 기타 프로파일

```bash
# robot PC 에서 GUI 로 robot 구성요소만 로컬 관리 (agent 불필요)
python3 -m launcher gui --config ... --profile robot

# PC 한 대에서 전부 로컬 (개발/시뮬)
python3 -m launcher gui --config ... --profile local
```

---

## 3. GUI 조작

| 요소 | 동작 |
|------|------|
| ● 상태 dot | 초록=running, 파랑=done(oneshot 완료), 회색=stopped, **빨강=exited(비정상 종료 — 로그 확인!)**, 주황=원격 조회 실패 |
| 행 클릭 | 하단 로그 pane 이 해당 구성요소 tail 로 전환 |
| Start / Stop | 개별 시작/정지 |
| ▶ Start All | robot group → operator group 순서로 전체 시작. 그룹 안에서는 `depends_on` 위상 정렬 + `start_delay_s` 준수 (드라이버 → 수신부 → sender) |
| ■ Stop All | 역순 정지 (sender 먼저 끊고 드라이버 정지) |
| 키 전송 바 | `use_pty: true` 구성요소 선택 시 활성. **xr-dual-teleop 의 r(sync)/p(pause)/c(recal)/Space(E-Stop)/x(quit) 를 GUI 버튼으로 전달** |

정지 시맨틱: SIGINT → (`stop_grace_s` 대기) → SIGTERM → (3s) → SIGKILL.
프로세스 그룹 전체에 신호를 보내므로 `ros2 launch` 의 자식 노드까지 정리된다.

GUI 창을 닫으면 **로컬 구성요소만** 정지된다. robot PC 쪽은 agent 가 계속
관리하므로 GUI 를 재실행하면 그대로 이어서 보인다. robot 쪽까지 내리려면
닫기 전에 Stop All.

---

## 4. 표준 기동 순서 (Galaxy XR 양팔+양손 기준)

1. robot PC: agent 실행 (또는 부팅 자동 시작 확인).
2. 조종 PC: GUI 실행 → robot 패널이 조회되는지 확인 (배너에 오류 없음).
3. **▶ Start All**.
4. `xr-dual-teleop` 행 클릭 → 로그에서 `BridgePoseStore: http://localhost:8013/` 확인.
5. 헤드셋 Chrome → `http://localhost:8013/` → Enter VR/AR.
6. 표준 자세에서 GUI 키 전송 바의 **r (sync)** 클릭 (또는 해당 터미널에서 r).
7. 종료: **■ Stop All**.

---

## 5. 설정 파일 (`teleop_system.yaml`) 스키마

```yaml
network:              # ${var} 치환 소스 + 접속 정보 — IP/포트의 단일 소스
  robot_pc_ip: "192.168.0.10"
  agent_port: "9876"
  right_robot_ip: "192.168.0.2"
  left_robot_ip: "192.168.0.3"        # TODO: 왼팔 실제 IP
  right_delto_ip: "169.254.186.72"
  left_delto_ip: "169.254.186.73"
vars:                 # 추가 치환 변수
  teleop_dir: "/workspaces/tamp_ws/src/teleop_dev"
setups:               # 이름 붙은 shell 스니펫 (환경 소스)
  ros: "source /opt/ros/humble/setup.bash && source ${ws_dir}/install/setup.bash"
agent:
  host: "0.0.0.0"
  port: 9876
  token: ""           # 설정 시 GUI/agent 양쪽 동일 토큰 필요 (X-Auth-Token)
defaults:
  stop_grace_s: 5.0
  start_delay_s: 0.0
components:
  - name: dg5f-right-driver     # 고유 이름
    group: robot                # robot | operator — 어느 PC 담당인지
    setup: [ros]                # 실행 전 소스할 스니펫들
    cwd: "${teleop_dir}"        # 작업 디렉토리
    command: "ros2 launch ..."  # 실제 명령 (${var} 치환됨)
    depends_on: []              # Start All 순서 제약
    start_delay_s: 3.0          # 시작 후 다음 것 시작까지 대기
    stop_grace_s: 5.0           # SIGINT 후 대기 시간
    use_pty: false              # 키 입력 필요한 프로세스 (termios)
    oneshot: false              # true 면 rc=0 종료가 "done" (adb reverse 등)
    env: {}                     # 추가 환경변수
```

### 구성요소 추가 예시

D405 뷰어를 하나 더 띄우고 싶다면:

```yaml
  - name: my-extra-tool
    group: operator
    cwd: "${teleop_dir}"
    command: "python3 -m sender.cam.main --robot-ip ${robot_pc_ip} --http-port 8015"
    depends_on: [adb-reverse]
```

저장 후 GUI/agent 재시작 (설정은 시작 시 1회 로드).

### conda 환경을 쓰는 PC

```yaml
setups:
  conda: "source ~/miniconda3/etc/profile.d/conda.sh && conda activate teleop_operator"
components:
  - name: xr-dual-teleop
    setup: [conda]        # ← 추가
    ...
```

---

## 6. 하드웨어 없이 동작 검증 (데모 구성)

```bash
# 터미널 1 (agent 역할)
python3 -m launcher agent --config launcher/config/demo_local.yaml
# 터미널 2 (GUI) — 같은 PC 에서 원격 흐름까지 검증
python3 -m launcher gui --config launcher/config/demo_local.yaml \
    --profile operator --agent-url http://127.0.0.1:19876
```

demo-driver/receiver (무한 tick), demo-oneshot, demo-pty (키 에코) 로
Start All / 로그 tail / 키 전송 / Stop All 흐름을 안전하게 연습할 수 있다.

---

## 7. 트러블슈팅

| 증상 | 확인 사항 |
|------|-----------|
| 배너 "agent 연결 실패", robot 행 주황색 | robot PC 에서 agent 실행 중? `curl http://<robot_ip>:9876/api/health`. 방화벽에서 9876 허용? `network.robot_pc_ip` 값 정확? |
| 401 오류 | agent yaml 의 `agent.token` 과 GUI 쪽 config 가 다른 파일/값 — 같은 yaml 을 양쪽에서 사용할 것 |
| 구성요소가 시작 직후 빨강(exited) | 행 클릭 → 로그 확인. 대부분: ros 미소스 (setup 스니펫 경로), python 모듈 없음 (cwd 잘못), 장치 미연결 |
| `ros2: command not found` 로그 | `setups.ros` 의 setup.bash 경로가 그 PC 에 맞는지 확인 |
| dg5f 드라이버는 도는데 receiver 가 죽음 | receiver 는 드라이버 초기화 후 떠야 함 — `depends_on` + `start_delay_s` 확인 |
| Stop 눌러도 안 죽음 | `stop_grace_s` 후 SIGTERM→SIGKILL 로 강제 종료됨 (최대 grace+5s). 로그 마지막 줄 `── killed (SIGKILL) ──` 확인 |
| 키 전송 바가 비활성 | 해당 구성요소에 `use_pty: true` 필요 + 행을 클릭해 선택했는지 확인 |
| GUI 자체가 안 뜸 (`no display name`) | SSH 환경 — `ssh -X` 또는 해당 PC 로컬 세션에서 실행. headless robot PC 는 GUI 대신 agent 를 쓰는 구조 |
| adb-reverse 가 빨강 | 헤드셋 USB 연결 + `adb devices` 승인 확인 후 해당 행 Start 재시도 |

로그는 구성요소당 최근 2000줄이 메모리에 유지된다 (파일 저장 없음 —
장기 보관이 필요하면 해당 프로세스를 수동 실행으로 전환).

---

## 8. 보안 주의

agent HTTP API 는 인증 없이(기본값) 프로세스 실행/정지가 가능하다.
**신뢰할 수 있는 로봇 전용 LAN 에서만 사용**하고, 공유 네트워크라면:
1. `agent.token` 설정 (긴 랜덤 문자열),
2. 방화벽에서 9876 을 조종 PC IP 로만 허용.
