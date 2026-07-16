# 왼팔 (미검증 팔) 좌표계 튜닝 & 트러블슈팅 가이드

> 대상: 양팔 UR10e 를 미러/각도 장착한 시스템에서 **왼팔을 처음 가동**하는 사람.
> 오른팔은 단일팔 운용으로 검증 완료된 상태를 전제한다.
> 관련 설정 파일: [scripts/config/xr_dual.yaml](../scripts/config/xr_dual.yaml) (sender 측),
> [robot/arm/admittance/config/left.yaml](../robot/arm/admittance/config/left.yaml) (robot 측).

---

## 0. 왜 왼팔은 별도 튜닝이 필요한가

XR sender 는 사용자 손목의 이동량(WebXR 좌표계)을 로봇 base_link 좌표계로
회전시키는 **remap 행렬** 을 사용한다:

```
robot_delta = R_remap @ webxr_delta        (위치)
R_delta_robot = R_remap @ R_delta_webxr @ R_remapᵀ   (회전, conjugation)
```

오른팔의 `R_remap = R_x(+90°)` (`remap_rpy_deg: [90, 0, 0]`) 은 **오른팔의
base_link 방향 기준으로 실측**된 값이다. 왼팔이 오른팔과 다른 방향으로
장착되어 있으면 (대면/각도 장착) 왼팔의 base_link 축이 다른 곳을 향하므로
**같은 remap 을 쓰면 손 이동이 엉뚱한 방향으로 매핑된다.** 이것이 왼팔에서
발생할 수 있는 좌표계 이슈의 근원이고, 팔별 `remap_rpy_deg` / `remap_matrix`
로 해결한다.

또 한 가지: **WebXR 좌표계 자체는 'r' 캘리브레이션 시점의 헤드셋 자세로
정해진다** (local-floor: +x 오른쪽, +y 위, −z 정면). 따라서:

> ⚠️ **표준 자세 규칙** — Chrome 창을 띄우고 'r' 을 누를 때는 항상
> **오른팔 검증 때와 같은 방향** (오른팔 base_link +x 정면) 을 보고 시작한다.
> 왼팔 remap 은 이 표준 자세를 전제로 실측·확정하므로, 매 세션 같은 자세로
> 시작해야 확정한 remap 이 재현된다. 세션마다 다른 방향을 보고 'r' 을 누르면
> "어제는 맞았는데 오늘은 어긋난다" 형태의 문제가 생긴다.

---

## 1. 첫 가동 전 안전 설정 (필수)

| 항목 | 설정 | 파일 |
|------|------|------|
| 위치 scale | **0.2~0.3** (손 10cm → 로봇 2~3cm) | `xr_dual.yaml` `arms.left.scale` |
| sender workspace | 좁게 (±0.5m, z 0.6m 이하) | `xr_dual.yaml` `arms.left.workspace` |
| robot 측 joint 속도 | `max_joint_vel: 0.3` | `left.yaml` `safety.max_joint_vel` |
| robot 측 EE 속도 | `max_ee_velocity: 0.08` | `left.yaml` `safety.max_ee_velocity` |
| 초기 자세 이동 | **`initial_pose.enabled: false`** | `left.yaml` |
| admittance 프리셋 | STIFF | `left.yaml` `admittance.default_preset` |

`initial_pose` 를 끄는 이유: 기본 `joint_values` 는 **오른팔에서 실측한
HOME_JOINTS** 다. 미러 장착 왼팔에 그대로 적용하면 시작하자마자 팔이 예상
밖 방향으로 크게 이동할 수 있다. 왼팔 home 은 §6 에서 실측 후 채운다.

물리 안전:
- 티치펜던트 E-Stop 에 한 손을 올려둔 사람 배치 (2인 운용 권장).
- 왼팔 주변 장애물/케이블 제거, 충돌 반경 확보.
- 키보드 `Space` (소프트 E-Stop) 위치 확인 — sender 에서 즉시 정지 송신.

---

## 2. 왼팔 단독 실행 (튜닝 모드)

robot PC (왼팔 수신부만):
```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m robot.arm.admittance.main --mode rtde --input unified \
    --config robot/arm/admittance/config/left.yaml --robot-ip <왼팔 IP>
```

조종 PC (왼팔 sender 만, 손 비활성):
```bash
cd /workspaces/tamp_ws/src/teleop_dev
python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml \
    --target-ip <robot PC IP> --only-arm left --no-hands
```

헤드셋 Chrome → `http://localhost:8013/` → Enter VR → 표준 자세(§0)에서 `r`.

---

## 3. 축별 검증 절차 (핵심)

`r` 캘리브레이션 직후, **한 번에 한 축씩** 천천히 손을 움직이고 로봇 TCP
가 움직이는 방향을 기록한다. 각 축 확인 후 손을 원위치로 되돌린다.

| # | 손 동작 (WebXR 축) | 기대 (mirror [90,0,180] 기준) | 실제 로봇 방향 (기록) |
|---|--------------------|------------------------------|----------------------|
| 1 | 오른쪽으로 10cm (+x) | base −x | ________ |
| 2 | 위로 10cm (+y)       | base +z | ________ |
| 3 | 몸쪽으로 10cm (+z)   | base +y | ________ |

관찰 요령:
- 로봇 방향은 **왼팔 base_link 축 기준** 으로 기록한다 (robot 수신부
  터미널의 `EE Pos: x= y= z=` 값 변화를 읽는 것이 가장 정확).
- 각 축이 "로봇의 어느 축 방향으로, +인지 −인지" 만 기록하면 된다.
- 대각선으로 움직이면 판정이 안 되므로 반드시 한 축씩.

### 기록표 → remap 행렬 결정 규칙

remap 행렬의 **j번째 열 = "손 j축(+) 이동이 만들어야 할 로봇 방향 단위벡터"**:

```
        손+x가 만든 방향   손+y가 만든 방향   손+z가 만든 방향
R    =  [    열 1       |      열 2       |      열 3      ]
```

예: 실측 결과가 "손+x → 로봇−x, 손+y → 로봇+z, 손+z → 로봇+y" 라면:

```yaml
remap_matrix: [[-1, 0, 0],
               [ 0, 0, 1],
               [ 0, 1, 0]]
```

**반드시 확인**: 열들이 서로 직교하고 det = +1 (proper rotation) 이어야
한다. det = −1 (reflection) 이 나오면 어딘가 기록이 잘못된 것 —
3개 열 중 하나의 부호가 틀렸을 가능성이 높다 (§5 증상 3 참조).
sender 가 로드 시 `validate_remap` 으로 검증하며 det=−1 이면 기동 거부한다.

### 대표 장착 케이스 조견표

| 장착 형태 (오른팔 기준) | `remap_rpy_deg` | 손+x → | 손+y → | 손+z → |
|--------------------------|-----------------|--------|--------|--------|
| 같은 방향 나란히          | `[90, 0, 0]`    | +x     | +z     | −y     |
| 대면 (180° 미러)          | `[90, 0, 180]`  | −x     | +z     | +y     |
| 왼쪽으로 90° 돌아감       | `[90, 0, 90]`   | +y     | +z     | +x     |
| 오른쪽으로 90° 돌아감     | `[90, 0, -90]`  | −y     | +z     | −x     |
| 그 외 각도 (예: 45° 벌어짐) | `[90, 0, ±45]` 등 yaw 조정 | (실측) | +z | (실측) |
| 임의 장착                 | `remap_matrix` 직접 기입 | 실측 열 1 | 실측 열 2 | 실측 열 3 |

> yaw 값의 의미: 왼팔 base_link 가 오른팔 base_link 대비 **위(+z)에서 봤을 때
> 반시계로 몇 도 돌아 앉아있는지** 와 일치한다. 장착 도면이 있으면 도면의
> 각도를 그대로 넣고 §3 절차로 확인만 해도 된다.

수정 후: `xr_dual.yaml` 저장 → sender 재시작 → §3 재검증. 3축 모두 기대와
일치할 때까지 반복한다.

---

## 4. 회전 검증

위치 3축이 맞으면 회전도 같은 행렬로 conjugation 되므로 대개 함께 맞는다.
확인 절차:

1. `r` 재캘리브레이션.
2. 손목만 천천히 좌우로 비틀기 (요) → TCP 가 base z축 중심으로 같은
   방향 회전하는지.
3. 손목 상하 굽히기 (피치) → TCP 회전 방향 확인.
4. 손목 롤 → TCP 롤 방향 확인.

위치는 맞는데 회전만 반대라면 → 기록표의 det 를 다시 확인 (reflection 을
회전으로 잘못 근사하면 위치 1~2축이 우연히 맞으면서 회전이 반대로 나오는
패턴이 전형적). remap 을 처음부터 다시 실측한다.

---

## 5. 증상별 처방표

| # | 증상 | 원인 | 처방 |
|---|------|------|------|
| 1 | 좌우가 거울처럼 반대로 움직임 | yaw 오류 (0↔180 또는 ±90 반전) | §3 재실측, yaw 부호/값 수정 |
| 2 | 위아래는 맞는데 앞뒤/좌우가 서로 바뀜 | yaw ±90 케이스를 0/180 으로 설정 | 조견표에서 ±90 행 적용 |
| 3 | 위치 일부 축은 맞는데 회전이 반대 | 기록표 det=−1 (reflection) | 열 부호 재확인, det=+1 로 재작성 |
| 4 | 캘리 직후엔 맞는데 점점 어긋남 | 'r' 시점 헤드셋이 표준 자세가 아니었음 | §0 표준 자세로 `r` 재캘리 |
| 5 | 모든 축이 N° 돌아간 채 일관되게 어긋남 | 헤드셋이 N° 돌아간 자세로 `r` | §0 표준 자세로 `r` 재캘리 |
| 6 | `c` 직후 로봇이 점프 | `c` 는 pause 없이 즉시 재캘리 (정상 동작) | 손을 새 위치로 옮길 땐 `p` → 이동 → `p` |
| 7 | 로봇이 간헐적으로 멈춤 (WARN stale) | 헤드셋 tracking 끊김 / 손 시야 밖 | 손을 시야 안에, USB 케이블/adb 확인 |
| 8 | 시작하자마자 원점 방향으로 기어감 | (구버전 버그 — 수정됨) 캘리 안 된 상태로 target 송신 | 코드 최신화. 현재는 캘리 전 target 을 보내지 않음 |
| 9 | 로봇이 아예 안 움직임 | 수신부 포트 불일치 (9875 vs sender 설정) | `xr_dual.yaml` `arms.left.port` == `left.yaml` `input.unified_port` 확인 |
| 10 | 반대쪽(오른)팔이 왼손 따라 움직임 | 좌우 sender port 를 서로 바꿔 설정 | 포트 표 (§7) 대로 재설정 |

---

## 6. 검증 완료 후 정상 운용 전환

3축 위치 + 3축 회전이 모두 확인되면:

1. **왼팔 home 실측**: 티치펜던트 freedrive 로 왼팔을 안전한 작업 시작
   자세로 이동 → 펜던트에서 joint 값 (rad) 읽기 → `left.yaml`
   `initial_pose.joint_values` 에 기입 → `enabled: true`.
2. **속도 상향**: `left.yaml` `max_joint_vel: 0.3 → 0.5`,
   `max_ee_velocity: 0.08 → 0.1` (오른팔과 동일).
3. **scale 상향**: `xr_dual.yaml` `arms.left.scale: 0.3 → 0.5` (오른팔과 동일).
4. **workspace 확장**: 작업 영역에 맞게 (오른팔 값 참고).
5. **확정 remap 기록**: 실측 확정한 `remap_rpy_deg` (또는 `remap_matrix`)
   값을 이 문서 하단 "실측 기록" 에 날짜와 함께 남길 것.
6. 양팔 동시 가동: `--only-arm left` 제거 후
   `python3 -m scripts.run_xr_dual_teleop --config scripts/config/xr_dual.yaml`
   (또는 웹 대시보드에서 파츠 시작 — [launcher_guide_ko.md](launcher_guide_ko.md)).

---

## 7. 참고: 양팔 포트/IP 표

| 구성요소 | IP | 수신 포트 |
|----------|-----|-----------|
| 오른팔 UR10e | 192.168.0.2 | robot PC 9871 (UDP) |
| 왼팔 UR10e | (미정 — placeholder 192.168.0.3) | robot PC 9875 (UDP) |
| 오른손 DG5F | 169.254.186.72 (Modbus) | robot PC 9872 (UDP) |
| 왼손 DG5F | 169.254.186.73 (Modbus) | robot PC 9874 (UDP) |

(9873 은 카메라 ZMQ, 9876 은 런처 웹 대시보드가 사용)

---

## 8. 첫 가동 체크리스트

- [ ] `left.yaml` `robot.ip` 를 왼팔 실제 IP 로 수정했다
- [ ] `initial_pose.enabled: false` 확인
- [ ] `xr_dual.yaml` `arms.left.scale` ≤ 0.3 확인
- [ ] 펜던트 E-Stop 담당자 배치
- [ ] 왼팔 주변 충돌 반경 확보
- [ ] `--only-arm left --no-hands` 로 실행
- [ ] 표준 자세(§0)에서 `r`
- [ ] §3 축별 검증 3축 기록 완료
- [ ] remap 수정 → 재검증 → 3축 일치
- [ ] §4 회전 검증 완료
- [ ] §6 전환 절차 수행 + 실측 기록 작성

---

## 실측 기록

| 날짜 | 확정 remap | 왼팔 IP | 왼팔 home joints | 비고 |
|------|-----------|---------|------------------|------|
| (기입) | | | | |
