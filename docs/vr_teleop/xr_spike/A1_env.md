# Phase A · Unit A1 — environment.yaml 갱신

## 목적

`teleop_dev/environment.yaml` 에 XR sender 의존성 추가. 기존 패키지는 그대로 둠.

## 변경

| 의존성 | 용도 | 추가 위치 |
|---|---|---|
| `aiohttp>=3.8` | `sender.xr_common.bridge_pose_store` HTTP+WS server | pip 섹션 마지막 |

## 검증

```bash
# 기존 env 에 의존성 추가
conda activate teleop_operator
conda env update -f environment.yaml --prune  # 또는: pip install 'aiohttp>=3.8'

# import 확인
python3 -c "import aiohttp; print(aiohttp.__version__)"
# → 3.8 이상이면 OK

# (참고) 이미 install 되어 있는 dex_retargeting / pyyaml 둘 다 그대로 사용
python3 -c "import dex_retargeting; import yaml; print('ok')"
```

기존 conda env 가 없는 경우:

```bash
cd /workspaces/tamp_ws/src/teleop_dev
conda env create -f environment.yaml
conda activate teleop_operator
```

## 영향 받는 파일

- `src/teleop_dev/environment.yaml` (수정 1줄)
