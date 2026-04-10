# RealSense sender — 설치/하드웨어 가이드

`sender/hand/realsense_sender`를 동작시키기 위한 환경 구축 + Intel RealSense
D405 하드웨어 셋업.

알고리즘/런타임 사용은 [`sender/hand/docs/realsense_sender_usage.md`](../../docs/realsense_sender_usage.md) 참조.

---

## 1. 사전 설치 (조종 PC)

`environment.yaml`을 사용하면 자동 설치됨. 수동 설치 시:

```bash
pip install pyrealsense2          # Intel RealSense SDK Python binding
pip install "mediapipe==0.10.21"  # 0.10.22+ 는 mediapipe.framework 모듈 제거됨
pip install opencv-python         # BGR↔RGB 변환 + 시각화
pip install "numpy<2"             # pinocchio / eigenpy ABI 호환
pip install dex_retargeting       # gen3a_dex_retarget 의존성
```

> **참고: `git clone dex-retargeting` 불필요.** 이 모듈은 DG5F URDF
> (`sender/hand/gen3a_dex_retarget/config/dg5f_right_retarget.urdf`)를 자체
> 보관하고 `DexRetargetWrapper`가 `RetargetingConfig.set_default_urdf_dir()`로
> 그 디렉터리를 지정하므로, dex_retargeting 라이브러리 내장 `assets/robots/hands/`는
> 사용하지 않습니다.

### 버전 제약

| 패키지 | 제약 | 이유 |
|---|---|---|
| `numpy` | < 2.0 | dex_retargeting / pinocchio / eigenpy 모두 numpy 1.x ABI |
| `mediapipe` | == 0.10.21 | 0.10.22+ 에서 `mediapipe.framework` 모듈 제거 |
| `pyrealsense2` | ≥ 2.55 | D405 펌웨어 호환 |

### 설치 검증

```bash
python3 -c "
import pyrealsense2 as rs
import mediapipe as mp
import cv2, numpy
print(f'pyrealsense2: {rs.__version__}')
print(f'mediapipe:    {mp.__version__}')
print(f'opencv:       {cv2.__version__}')
print(f'numpy:        {numpy.__version__}')
print('OK')
"
```

---

## 2. RealSense D405 하드웨어 셋업

### 2.1 udev 규칙 (Linux)

D405를 sudo 없이 접근하려면 udev 규칙이 필요. `librealsense2` apt 패키지가
자동 설치하지만, 누락된 경우:

```bash
# Intel 공식 librealsense apt repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(. /etc/os-release; echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dev
```

설치 후 USB 케이블 재연결.

### 2.2 D405 검출 확인

```bash
rs-enumerate-devices | grep -A2 "Intel RealSense D405"
# 다음과 같은 출력이 나와야 함:
#   Device info:
#     Name                          : Intel RealSense D405
#     Serial Number                 : 1234ABCDEF
#     ...
```

검출이 안 되면:
- USB-C 케이블 / USB 3.0 포트 확인 (D405는 USB 3 필수)
- `dmesg | tail -20`으로 USB 인식 메시지 확인
- udev 규칙 재적용: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### 2.3 카메라 단독 테스트

```bash
# Intel 공식 viewer (옵션, librealsense2-utils 포함)
realsense-viewer
# → GUI 창에 컬러 + depth 스트림 표시되어야 정상
```

---

## 3. MediaPipe 모델 파일

`sender/hand/realsense/models/hand_landmarker.task` (7.5 MB) — 자체 보관.

다른 모델을 쓰고 싶으면 `RealSenseReader(model_path=...)` 또는
`HandDetector(model_path=...)`로 경로 지정. 기본은 번들된 모델.

별도 다운로드 불필요.

---

## 4. 자주 발생하는 문제

### `ModuleNotFoundError: No module named 'pyrealsense2'`
```bash
pip install pyrealsense2
```

### `ModuleNotFoundError: No module named 'mediapipe.framework'`
mediapipe 다운그레이드:
```bash
pip install "mediapipe==0.10.21"
```

### `_ARRAY_API not found` 또는 import 시 segfault
numpy 2.x 설치되어 있음:
```bash
pip install "numpy<2"
```

### `RuntimeError: No device connected`
- `rs-enumerate-devices`로 D405 검출 확인
- USB 케이블 재연결
- 다른 RealSense 앱 (예: `realsense-viewer`)이 D405를 점유 중인지 확인 → 종료

### MediaPipe 모델 파일 없음
```
FileNotFoundError: Bundled model not found at .../hand_landmarker.task
```
번들 파일이 누락된 경우 (드물게 git LFS 미설정 등):
```bash
wget https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task \
  -O sender/hand/realsense/models/hand_landmarker.task
```

---

## 5. 다음 단계

설치/하드웨어가 준비되면 사용 가이드로:
[`sender/hand/docs/realsense_sender_usage.md`](../../docs/realsense_sender_usage.md)
