# 런처 가이드 — 로봇 PC (9876) + 조종 PC (9877) 웹 대시보드

> 두 개의 웹 대시보드로 전체 시스템을 관리한다:
> - **로봇 PC 런처 (9876)** — 팔/손/캠 프로세스 on-off·상태·E-STOP·초기위치·파라미터 (1부)
> - **조종 PC 런처 (9877)** — XR 러너·adb·웹 키패드·자동 캘리·제스처 (2부, §12~)
>
> 두 런처는 같은 코어(launcher/{config,process,manager}.py)를 공유하며
> 서로 다른 lock 을 사용해 각자 PC 에서 독립 실행된다.

---

# 1부 — 로봇 PC 런처 (launcher.robot, 포트 9876)

## 1. 구조

```
조종 PC (브라우저)                        로봇 PC
┌────────────────────┐    HTTP 9876    ┌─────────────────────────────────┐
│ http://robot:9876/ │ ─────────────→  │ python3 -m launcher.robot.web   │
│  - 파츠 on/off      │                 │  ├─ dg5f-right/left-driver      │
│  - 상태/로그        │                 │  ├─ hand-right/left-receiver    │
│  - 초기 위치 이동   │                 │  ├─ arm-right/left-receiver     │
│  - E-STOP          │                 │  ├─ cam-server                  │
│  - 파라미터 편집    │                 │  └─ move-home-* (oneshot)       │
└────────────────────┘                 └─────────────────────────────────┘
```

- **launcher 는 기존 코드를 import 하지 않는다** — yaml 에 정의된 명령을
  subprocess 로 실행할 뿐. 런처 변경이 teleop 코드에 영향 없음 (역도 성립).
- 의존성: 표준 라이브러리 + PyYAML (+선택 ruamel.yaml — 파라미터 저장 시
  yaml 주석 보존. 로봇 도커 이미지에 포함됨).
- 모든 IP/포트/컴포넌트/파츠는
  [launcher/robot/config/robot.yaml](../launcher/robot/config/robot.yaml)
  한 파일이 단일 소스 — **현장 배치 시 이 파일만 수정.**
- "파츠" = 대시보드 카드 단위: 오른팔 / 왼팔 / 오른손(드라이버+수신부) /
  왼손 / 카메라.

## 2. 실행

### 웹 대시보드 (권장 — 로봇 PC 에서 상시 실행)

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m launcher.robot.web                 # robot.yaml 사용, 포트 9876
# 옵션: --config <yaml> --host 0.0.0.0 --port 9876
```

- 조종 PC 브라우저에서 `http://<robot_pc_ip>:9876/` 접속.
- 데몬을 **종료하면 관리 중이던 컴포넌트가 전부 정지**된다 (고아 프로세스
  방지). 상시 운용은 tmux 나 systemd 에 등록 권장:
  ```bash
  tmux new -s launcher 'cd /workspaces/tamp_ws/src/teleop_dev && python3 -m launcher.robot.web'
  ```

### 터미널 브링업 (웹 없이)

```bash
python3 -m launcher.robot.bringup                      # 전체 (양팔+양손+캠)
python3 -m launcher.robot.bringup --side right         # 오른쪽만
python3 -m launcher.robot.bringup --side left --no-cam # 왼쪽만, 캠 제외
python3 -m launcher.robot.bringup --parts arm-left     # 파츠 직접 지정
python3 -m launcher.robot.bringup --list               # 선택 결과 확인만
python3 -m launcher.robot.bringup --dry-run            # 실행될 명령 확인만
```

- 드라이버 → 수신부 순서(topo + 초기화 지연)로 시작, **Ctrl+C 로 역순 정지**.
- 웹 데몬과 **동시 실행 불가** (flock 상호배제 — 프로세스 이중 관리 방지).
  이미 웹이 떠 있으면 안내 메시지와 함께 거부된다.

## 3. 메인 대시보드 (`/`)

| 요소 | 설명 |
|------|------|
| 파츠 카드 | 오른팔/왼팔/오른손/왼손/카메라 — 상태 pill + HW 도달성 dot |
| 상태 pill | `running` 초록 / `stopped` 회색 / `partial` 주황(일부만 실행) / **`error` 빨강(비정상 종료 — 로그 확인!)** |
| HW dot | 초록=하드웨어 TCP 도달 가능 (UR:30004, DG5F:502), 빨강=도달 불가 (전원/케이블/IP 확인) |
| 라이브 메트릭 | 팔: EE 위치/RPY·Safety·E-Stop·Input age (수신부 로그 파싱) / 캠: 캠별 fps·KB/s·fail |
| 시작/정지/재시작 | 파츠 단위 (손 파츠는 드라이버→수신부 순서 자동) |
| ▶ 전체 시작 / ■ 전체 정지 | 전체 topo 순 / 역순 |
| **E-STOP** (우상단 적색) | §5 참조 — 모든 페이지 공통 |
| 초기 위치 이동 (팔 카드) | §6 참조 |
| 파라미터 → | 팔/캠 파라미터 페이지로 이동 (§7) |
| 하단 로그 | 컴포넌트 선택 → 실시간 tail |

## 4. 표준 운용 순서

1. 로봇 PC: `python3 -m launcher.robot.web` (또는 tmux/systemd 상시 실행 확인)
2. 조종 PC 브라우저: `http://<robot_pc_ip>:9876/`
3. HW dot 이 초록인지 확인 (로봇/손 전원·네트워크)
4. **▶ 전체 시작** (필요 없는 파츠는 개별 정지)
5. 조종측: 조종 PC 런처(9877, 2부)에서 **조종 시작** — 또는 터미널
   `python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml --target-ip <robot_pc_ip>`
6. 작업 종료: 러너 정지 (9877 또는 `x`) → 로봇 대시보드 **■ 전체 정지**

## 5. E-STOP (비상정지)

버튼 클릭 시 (확인창 없이 즉시):

1. 각 팔 수신부의 localhost UDP 포트로 **estop 패킷 burst** (200Hz×0.5s) —
   수신부가 즉시 `stopScript()` 로 로봇 정지 + E-Stop latch.
2. 손 수신부 정지 (손은 마지막 자세 유지).
3. 1초 안에 수신부 로그에서 latch 가 확인되지 않으면 해당 수신부를
   SIGINT — servoJ 스트림이 끊겨 UR 컨트롤러가 자체 안전 정지.

해제: 팔 수신부의 E-Stop latch 는 sender 의 `r`(reset) 또는 수신부
재시작으로 해제된다. 배너에 전송 결과/경고가 표시된다.

> ⚠️ **소프트 스톱이다.** 티치펜던트의 물리 E-Stop 을 대체하지 않는다 —
> 실기 운용 시 물리 E-Stop 접근 가능한 인원 배치는 여전히 필수.

## 6. 초기 위치 이동

팔 카드(또는 팔 페이지)의 `초기 위치 이동` 버튼:

- 팔 yaml 의 `initial_pose.joint_values` 로 **moveJ** 하는 oneshot 프로세스
  (`move-home-<side>`) 를 실행. 진행/결과는 카드 메트릭과 로그에 표시.
- **수신부가 실행 중이면 거부** (RTDE 제어 연결은 하나만 가능) —
  확인창에서 "정지 후 이동" 을 선택하면 수신부를 정지하고 진행한다.
- **`initial_pose.enabled: false` 면 이동 자체가 거부**된다 (exit 2).
  왼팔은 home 실측 전까지 이 상태가 기본 —
  [xr_dual_arm_left_tuning_ko.md](xr_dual_arm_left_tuning_ko.md) §6 절차로
  실측 후 파라미터 페이지에서 joint_values 기입 + enabled 전환.
- 속도/가속은 robot.yaml `move_home:` 섹션 (왼팔은 보수적 기본값).

## 7. 파라미터 페이지

메인 카드의 `파라미터 →` 로 진입. **저장 → 해당 파츠 재시작** 시 적용된다
(모든 config 는 프로세스 시작 시 1회만 로드됨 — 저장 후 배너의 안내에 따라
재시작 버튼 사용).

### 팔 (`/arm?side=right|left`) — 대상: `robot/arm/admittance/config/{right,left}.yaml`

| 파라미터 | 의미 |
|----------|------|
| robot.ip | UR10e 주소 |
| input.unified_port | UDP 수신 포트 — sender(xr_dual.yaml)의 해당 팔 port 와 일치 필요 |
| filter.alpha_position / orientation | 필터 반응성 (0~1, 높을수록 빠름) |
| safety.max_joint_vel / max_ee_velocity ⚠ | 속도 한계 (주황 = 안전 관련) |
| safety.workspace.x/y/z ⚠ | base_link 기준 작업영역 [min, max] |
| admittance.default_preset | STIFF/MEDIUM/SOFT/FREE |
| admittance.max_displacement_trans/rot | 외력 순응 최대 변위 |
| initial_pose.enabled / joint_values / move_duration_s ⚠ | 초기 자세 (§6) |

편집 화이트리스트 밖의 키(IK cost 등)는 yaml 직접 편집. 저장은 주석을
보존한다 (ruamel.yaml — 미설치 환경이면 주석 유실 경고 배너 표시).

### 캠 (`/cam`) — 대상: `robot/cam/config/default.yaml`

- stream.port (조종측 sender.cam zmq.port 와 일치), stream.jpeg_quality
- cameras 행 편집: name(ZMQ topic)/serial(빈값=자동)/width/height/fps —
  행 추가/삭제로 다중 캠 구성. 시리얼 확인: `python3 -m robot.cam.list_cameras`
- 상태 표: 캠별 fps/KB/s/fail (2s 주기 갱신), stats 정지 시 stale 표시
- 영상 미리보기는 ZMQ 라 브라우저 불가 — 조종 PC 의
  `python3 -m sender.cam.main` → `http://localhost:8014/` 사용

## 8. HTTP API (자동화/스크립트용)

```
GET  /api/health                    파츠/컴포넌트 목록
GET  /api/status                    전체 상태 스냅샷 (1s 폴링용)
POST /api/parts/<name>/start|stop|restart
POST /api/start_all | /api/stop_all
POST /api/estop                     body {"sides": ["right","left"]} (생략 시 양팔)
POST /api/arms/<side>/move_home     body {"stop_receiver": true|false} → 200/409
GET  /api/components/<name>/log?since=<seq>&max=200
GET  /api/params/<arm-right|arm-left|cam>
PUT  /api/params/<target>           body {"values": {"safety.max_joint_vel": 0.4}}
```

인증: robot.yaml `web.token` 설정 시 `X-Auth-Token` 헤더 또는 `?token=`
쿼리 필수 (브라우저는 최초 1회 `?token=...` 접속 → localStorage 저장).
HTTPS: `web.tls_cert`/`web.tls_key` 지정 시.

## 9. 하드웨어 없이 검증 (데모 구성)

```bash
python3 -m launcher.robot.web --config launcher/robot/config/demo.yaml
# → http://127.0.0.1:19876/
```

더미 컴포넌트(admittance 상태블록 모사 포함)와 합성 카메라 2대로 카드
on/off, 메트릭 파싱, E-STOP(백스톱), 파라미터 저장, 로그 tail 을 모두
연습할 수 있다. 단위테스트:
`python3 -m unittest launcher.robot.tests.test_status_params`

## 10. 트러블슈팅

| 증상 | 확인 |
|------|------|
| 브라우저 접속 안 됨 | 로봇 PC 에서 web 데몬 실행 중? 방화벽 9876 허용? `curl http://<ip>:9876/api/health` |
| 401 오류 | `web.token` 설정됨 — `?token=...` 로 접속하거나 헤더 지정 |
| "다른 런처 관리자가 관리 중" | bringup CLI 와 web 은 동시 실행 불가 — 하나를 종료 |
| 파츠가 error(빨강) | 로그 확인. 흔한 원인: ROS 미소스(setups.ros 경로), 모듈 없음(cwd), 장치 미연결 |
| HW dot 빨강 | 로봇/손 전원·케이블·IP (robot.yaml network 섹션) 확인 |
| 팔 메트릭이 "(라이브 상태 없음)" | 수신부가 상태블록을 아직 안 찍음 (기동 직후) — 몇 초 대기 |
| 캠 stale 표시 | 캠 프로세스는 살아있으나 stats 정지 — USB/전원 확인 후 재시작 |
| E-STOP 후 팔이 안 움직임 | 정상 (latch) — sender `r` 또는 수신부 재시작으로 해제 |
| move_home 409 | 수신부 실행 중 — 확인창에서 "정지 후 이동" 또는 수신부 정지 후 재시도 |
| move_home exit 2 | `initial_pose.enabled: false` — 왼팔 튜닝 가이드 §6 선행 |
| 파라미터 저장했는데 동작 그대로 | 재시작해야 적용 — 배너의 재시작 버튼 |

## 11. 보안

기본값(token 없음)에서는 접속자 누구나 로봇 프로세스를 제어할 수 있다.
**신뢰할 수 있는 로봇 전용 LAN 에서만 사용**하고, 공유망이면
`web.token` 설정 + 방화벽에서 9876 을 조종 PC IP 로 제한할 것.

---

# 2부 — 조종 PC 런처 (launcher.sender, 포트 9877)

## 12. 구조

```
헤드셋 (Galaxy XR)                        조종 PC
┌─────────────────────┐   USB (adb)    ┌──────────────────────────────────┐
│ Chrome 탭 1:        │ ─────────────→ │ python3 -m launcher.sender.web   │
│  localhost:9877     │  reverse       │  ├─ xr-dual-runner        (pty)  │
│  (이 대시보드)       │  8013/8014/    │  ├─ xr-single-right-runner(pty)  │
│ Chrome 탭 2:        │  9877          │  ├─ xr-single-left-runner (pty)  │
│  localhost:8013     │                │  └─ cam-viewer                   │
│  (WebXR pose 송신)   │                │  + adb 감시 / 자동 캘리 FSM       │
└─────────────────────┘                └──────────────────────────────────┘
```

- **러너 슬롯**: 듀얼(양팔+양손) / 싱글 오른쪽 / 싱글 왼쪽 — 모두
  BridgePoseStore(8013)와 UDP 포트를 쓰므로 **동시에 하나만** 실행.
  다른 모드 시작 시 확인 후 자동 전환.
- 러너는 pty 로 실행되어 **웹 키패드의 키가 실제 키보드처럼 전달**된다.
- 설정 단일 소스: [launcher/sender/config/sender.yaml](../launcher/sender/config/sender.yaml)
  (러너/adb reverse 포트/자동 캘리 타이밍). XR 조종 파라미터는
  [scripts/config/xr_dual.yaml](../scripts/config/xr_dual.yaml) — 파라미터 페이지에서 편집.
- 로봇 PC 런처와 lock 이 달라 (robot/sender 별도) 양쪽을 동시에 운용한다.

## 13. 실행

```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m launcher.sender.web        # sender.yaml 사용, 포트 9877
```

- 조종 PC 브라우저: `http://localhost:9877/`
- **헤드셋 브라우저**: adb reverse 후 헤드셋 Chrome 에서 `http://localhost:9877/`
  — VR 진입 전 헤드셋 안에서 직접 "조종 시작"을 누를 수 있다 (혼자 운용).

터미널 브링업 (웹 없이):

```bash
python3 -m launcher.sender.bringup --mode xr-dual              # adb reverse 후 러너 foreground 실행
python3 -m launcher.sender.bringup --mode xr-dual --auto-calib # + 카운트다운 후 'r' 자동
python3 -m launcher.sender.bringup --list / --dry-run
```

웹 데몬과 bringup 은 동시 실행 불가 (flock 상호배제).

## 14. 대시보드 구성

| 카드 | 기능 |
|------|------|
| **조종 세션** | 모드 선택 + **▶ 조종 시작 (자동 캘리)** — §15 시퀀스. 카운트다운 숫자, 시도 i/N, 확인된 팔, 사유, 진행 히스토리, 취소 |
| **XR 러너** | 시작/정지 (모드 전환 시 confirm), 팔별 READY/SYNC/PAUSE + 캘리 횟수, 손별 TRACK/LOST, robot 응답 집계, 경고/제스처 표시, **키패드** |
| 키패드 | `r`(캘리/시작) `p`(일시정지) `c`(즉시 재캘리) `Space`(**E-Stop**) `+/−`(속도) `x`(종료) — 헤드셋 브라우저에서도 누르기 쉬운 큰 버튼 |
| **헤드셋 연결 (adb)** | devices/reverse 상태 (3s 갱신), `reverse 설정` `adb 서버 재시작` 버튼 + 명령 출력 |
| **카메라 뷰어** | sender.cam 시작/정지, 캠별 fps/KB/s/지연, 8014 영상 링크 |
| 로그 | 컴포넌트 선택 → 실시간 tail |

헤더의 dot = 헤드셋 WebSocket 연결 상태 (러너 로그 기반).

## 15. "조종 시작" 자동 캘리브레이션 (혼자 운용)

버튼 한 번으로: **adb 확인/reverse → 러너 시작 → 4초 카운트다운 → 'r' 자동
전송 → 캘리 성공 확인** (실패 시 2초 간격 자동 재시도, 최대 10회).

```
idle → adb → starting → countdown(4s) → calibrating(시도 1..N) → active
                                   └────────────→ failed / cancelled
```

- 혼자 운용 절차: 헤드셋 Chrome 탭1 `localhost:9877` → **조종 시작** →
  탭2 `localhost:8013` → **Enter VR** → 표준 자세로 손을 시야에 →
  카운트다운/재시도가 알아서 캘리를 잡는다 (헤드셋 데이터가 없으면
  "헤드셋 ws msg 없음" 사유로 재시도하며 기다려 줌).
- 러너가 이미 실행 중이면 러너 기동을 건너뛰므로 **같은 버튼이 "재캘리"**
  로도 동작한다.
- 실패 사유가 "로봇 PC 무응답 (Pose query failed)" 이면 로봇측 수신부가
  꺼져 있는 것 — 로봇 대시보드(9876)에서 팔 수신부를 켠 뒤 다시 시작.
- 타이밍은 sender.yaml `session:` (countdown_s / retry_delay_s / max_retries).

## 16. 제스처 명령 (opt-in, 기본 off)

헤드셋 안에서 키보드 없이 명령 — **파라미터 페이지에서 `gestures.enabled`
활성 후 러너 재시작**:

| 제스처 (양손 동시, 1.5s 유지) | 키 | 동작 |
|------|----|------|
| 양손 pinch (엄지-검지 붙임) | `r` | 캘리브레이션/시작 |
| 양손 squeeze (주먹 쥐듯 엄지-중지) | `p` | 일시정지 토글 |

⚠️ **오발동 주의**: 조종 중 물건을 잡는 동작에서 pinch/squeeze 가 자연
발생한다. 방어 장치(양손 동시 + 1.5s 유지 + 3s 불응기 + 손별 트래킹
신선도)가 있지만, **양손으로 동시에 물체를 집는 작업이 많다면 끄고
웹 키패드를 사용**할 것. hold_s/refractory_s 는 파라미터 페이지에서 조정.

## 17. 파라미터 페이지 (`/params`)

대상: `scripts/config/xr_dual.yaml` (저장 → **러너 재시작** 시 적용, 주석 보존)

- 팔별: enabled / port / hz / **scale** ⚠ / **remap RPY** ⚠ / workspace ⚠
- 손별: enabled / port / hz / convention (오므림↔벌림 반전 시 manus)
- 공통: robot_pc_ip, watchdog, 제스처 (enabled ⚠ / hold_s / refractory_s)

왼팔 튜닝(scale·remap)은 [xr_dual_arm_left_tuning_ko.md](xr_dual_arm_left_tuning_ko.md)
절차를 따르고, 값 수정만 이 페이지에서 하면 된다.

## 18. HTTP API (조종 PC, 자동화용)

```
GET  /api/health | /api/status | /api/session
POST /api/runner/start {mode} → 200|409   /api/runner/stop
POST /api/runner/key {key: r|p|c|space|+|-|x}
POST /api/adb/reverse | /api/adb/restart   (출력 캡처 반환)
POST /api/session/start {mode?, skip_adb?} | /api/session/cancel
POST /api/cam/start | /api/cam/stop
GET/PUT /api/params/xr-dual
GET  /api/components/<n>/log?since=&max=
```

인증/HTTPS 는 로봇측과 동일 (sender.yaml `web.token`/`tls_*`).

## 19. 하드웨어 없이 검증 (데모)

```bash
python3 -m launcher.sender.web --config launcher/sender/config/demo.yaml
# → http://127.0.0.1:19877/
```

mock 러너 3종: 듀얼(캘리 2회 거부 후 성공 — 재시도 경로), 싱글 오른쪽
(즉시 성공), 싱글 왼쪽(로봇 무응답 — hard-fail 경로). 자동 캘리/키패드/
모드 전환을 안전하게 연습할 수 있다. 단위테스트:
`python3 -m unittest discover -s launcher/sender/tests` +
`python3 -m unittest sender.xr_common.tests.test_gesture_commands`

## 20. 트러블슈팅 (조종 PC)

| 증상 | 확인 |
|------|------|
| adb 카드 "adb 없음" | platform-tools 설치 + PATH ([xr_input_guide.md](xr_input_guide.md) §2) |
| devices 에 unauthorized | 헤드셋 화면에서 USB 디버깅 승인 |
| reverse 누락 표시 | `reverse 설정` 버튼 — 실패 시 `adb 서버 재시작` 후 재시도 |
| 헤드셋 dot 빨강/회색 | 헤드셋 Chrome `localhost:8013` 접속 + Enter VR 상태인지 |
| 세션 failed "헤드셋 ws msg 없음" 반복 | 위와 동일 — VR 진입 + 손을 시야에 |
| 세션 failed "로봇 PC 무응답" | 로봇 대시보드(9876)에서 팔 수신부 시작 |
| 키패드 눌러도 반응 없음 | 러너 실행 중인지 (러너 카드), 로그에서 키 처리 확인 |
| 모드 시작 409 | 다른 러너 실행 중 — confirm 으로 전환 또는 정지 후 시작 |
| 제스처가 안 먹음 | gestures.enabled + 러너 재시작, 양손 모두 시야에, 1.5s 유지 |
| 제스처 오발동 | hold_s/refractory_s 상향 또는 비활성 (§16 주의) |
