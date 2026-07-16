# 로봇 PC 런처 가이드 — 브링업 CLI + 웹 대시보드

> 로봇 PC 의 팔(UR10e×2)·손(DG5F×2)·캠(D405) 프로세스를
> **웹 브라우저 한 곳에서** 켜고 끄고 모니터링한다.
> 조종 PC 에서는 브라우저로 `http://<robot_pc_ip>:9876/` 만 열면 된다.

---

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
5. 조종 PC 에서 XR sender 실행:
   `python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml --target-ip <robot_pc_ip>`
6. 작업 종료: sender 종료 (`x`) → 대시보드 **■ 전체 정지**

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
