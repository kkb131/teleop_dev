# teleop_dev 환경 설치 가이드

조종 PC(Operator)와 로봇 PC(AGX Orin)의 환경 구축 방법.
각 PC는 Docker 또는 Conda로 설치 가능하다.

---

## 빠른 선택 가이드

| PC | 추천 방법 | 이유 |
|----|----------|------|
| **조종 PC** | **Conda** | SteamVR이 호스트에서 실행되어야 하므로 Docker보다 Conda가 편리 |
| **로봇 PC (AGX Orin)** | **Docker** | Isaac ROS + CUDA + ROS2 의존성이 복잡, Docker가 재현성 보장 |

---

## 1. 조종 PC — Conda 설치 (추천)

### 1.1 사전 요건

- Ubuntu 22.04 (또는 Windows WSL2)
- Miniforge/Miniconda 설치됨
- SteamVR 설치됨 (Vive Tracker 사용 시)

### 1.2 Conda 환경 생성

```bash
cd /path/to/teleop_dev
conda env create -f environment.yaml
conda activate teleop_operator
```

설치되는 패키지:
| 패키지 | 용도 |
|--------|------|
| openvr | Vive Tracker SteamVR API |
| numpy (<2.0) | 좌표 변환, 쿼터니언 연산 (dex_retargeting / pinocchio ABI 호환) |
| pynput | 키보드 상태 읽기 (vive_sender) |
| pyyaml | YAML 설정 파싱 |
| pygame | 조이스틱/게임패드 입력 |
| libusb | Manus SDK 의존 |
| dex_retargeting | `manus_sender --retarget dex` 및 `realsense_sender` (DexPilot/Vector) |
| mediapipe==0.10.21 | MediaPipe HandLandmarker (realsense_sender) |
| pyrealsense2 | Intel RealSense D405 (realsense_sender) |
| opencv-python | 시각화 + BGR↔RGB (realsense_sender) |

### 1.3 SteamVR 설정 (Vive Tracker 사용 시)

```bash
# 1. Steam + SteamVR 설치 (Ubuntu)
sudo apt install steam
# Steam → Library → Tools → SteamVR 설치

# 2. SteamVR 실행
steam steam://run/250820

# 3. Vive Tracker 페어링
#    SteamVR 설정 → 컨트롤러 → 트래커 페어링 → 동글 연결
```

### 1.4 Manus SDK 빌드 (Manus 글러브 사용 시)

Manus 글러브 사용 시 두 가지 빌드 산출물이 필요합니다:

| 산출물 | 빌드 방법 | 사용처 |
|---|---|---|
| `SDKClient_Linux.out` (C++ subprocess) | `cd sender/hand/sdk && ./build.sh` | `manus_sender --sdk-mode subprocess` (legacy) |
| `manus_data_publisher` (ROS2 노드) | colcon 빌드 (별도 단계 필요) | `manus_sender --sdk-mode ros2` (권장), retarget_dev 의 `--sensing manus-ros2` |

자세한 빌드 절차 (`.so` 라이브러리 다운로드, Integrated vs Remote mode, colcon 워크스페이스 구성, CMakeLists 수정 등) 는 **[`hand_manus_guide.md` §2](hand_manus_guide.md#2-manus-sdk-설치-및-빌드)** 참조.

### 1.5 실행 테스트

```bash
conda activate teleop_operator

# 키보드 sender (가장 간단한 테스트)
python3 -m sender.arm.keyboard_sender --target-ip 192.168.0.10

# Vive Tracker 목록 확인
python3 -m sender.arm.vive_sender --list-trackers

# Vive sender
python3 -m sender.arm.vive_sender --target-ip 192.168.0.10

# Manus sender (Mode A: raw — receiver 측에서 retarget)
python3 -m sender.hand.manus_sender --target-ip 192.168.0.10

# Manus sender ([1A] Ergonomics Direct Mapping — sender 측에서 retarget)
python3 -m sender.hand.manus_sender --target-ip 192.168.0.10 --retarget ergo-direct --sdk-mode ros2

# Manus sender ([1A] + 2포즈 캘리브레이션)
python3 -m sender.hand.manus_sender --target-ip 192.168.0.10 --retarget ergo-direct --sdk-mode ros2 --calibrate

# Manus sender ([3A] dex_retarget — DexPilot fingertip 최적화, manus_data_publisher 필요)
python3 -m sender.hand.manus_sender --target-ip 192.168.0.10 --retarget dex --sdk-mode ros2

# RealSense sender ([3A] dex_retarget — D405 + MediaPipe + DexPilot)
python3 -m sender.hand.realsense_sender --target-ip 192.168.0.10 --hand right
```

### 1.6 PYTHONPATH 설정

teleop_dev 패키지를 import하려면 부모 디렉토리가 PYTHONPATH에 있어야 한다:

```bash
# teleop_dev가 /home/user/src/teleop_dev에 있는 경우:
export PYTHONPATH="/home/user/src:${PYTHONPATH}"

# 또는 .bashrc에 추가:
echo 'export PYTHONPATH="/home/user/src:${PYTHONPATH}"' >> ~/.bashrc
```

---

## 2. 조종 PC — Docker 설치 (대안)

SteamVR이 필요 없는 경우 (키보드/조이스틱만 사용):

```bash
cd teleop_dev/docker/operator

# 빌드
./build_image.sh

# 실행 (키보드 sender)
./run_container.sh python3 -m sender.arm.keyboard_sender --target-ip 192.168.0.10

# 대화형 셸
./run_container.sh
```

> Vive Tracker 사용 시 Docker 내부에서 SteamVR 연결이 어려우므로 Conda 방법을 추천.

---

## 3. 로봇 PC (AGX Orin) — Docker 설치 (추천)

### 3.1 사전 요건

- NVIDIA Jetson AGX Orin (JetPack 6.x)
- Docker + NVIDIA Container Toolkit:
  ```bash
  sudo apt install nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```

### 3.2 이미지 빌드

```bash
cd teleop_dev/docker/robot

# arm64 자동 감지 (Jetson에서 실행)
./build_image.sh

# 또는 개발 PC에서 arm64 크로스 빌드
./build_image.sh --arch arm64
```

### 3.3 컨테이너 실행

```bash
# teleop_dev 소스를 마운트하여 실행
./run_container.sh

# 컨테이너 내부에서:
cd /workspaces/teleop_ws/src/teleop_dev

# 팔 어드미턴스 teleop 실행
python3 -m robot.arm.admittance.main \
  --mode rtde --input unified --robot-ip 192.168.0.2

# 손 receiver 실행
python3 -m robot.hand.receiver --hand-ip 169.254.186.72
```

### 3.4 추가 터미널 접속

```bash
./run_container.sh --join
```

---

## 4. 로봇 PC — Conda 설치 (대안)

Docker 없이 네이티브 환경에서 실행하는 경우.

### 4.1 ROS2 Humble 설치 (필수)

```bash
# Ubuntu 22.04
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### 4.2 ROS2 apt 패키지 설치

```bash
sudo apt install -y \
  ros-humble-pinocchio \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ur-msgs \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### 4.3 Python 가상 환경 (venv 추천, Conda도 가능)

```bash
# venv (ROS2 apt 패키지와 호환이 좋음)
python3 -m venv ~/teleop_robot_env --system-site-packages
source ~/teleop_robot_env/bin/activate

# pip 패키지 설치
pip install \
  "pin-pink>=4.0.0" \
  "proxsuite>=0.6.0" \
  "ur-rtde>=1.5.0" \
  "pymodbus>=3.10,<4" \
  "PyYAML>=6.0" \
  "numpy<2.0"
```

> `--system-site-packages`가 중요: ROS2 apt로 설치한 pinocchio, rclpy 등을 venv에서 사용 가능하게 함.

> Conda를 사용하는 경우 pinocchio를 `conda install -c conda-forge pinocchio` 로 설치할 수 있지만, ROS2 패키지(rclpy, sensor_msgs)와 충돌 가능. venv + `--system-site-packages`를 추천.

### 4.4 실행

```bash
source /opt/ros/humble/setup.bash
source ~/teleop_robot_env/bin/activate
export PYTHONPATH="/path/to/src:${PYTHONPATH}"

cd /path/to/src/teleop_dev
python3 -m robot.arm.admittance.main --mode rtde --input unified --robot-ip 192.168.0.2
```

---

## 의존성 요약

### 조종 PC (sender/)

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip/conda | 좌표 변환 + dex_retargeting/pinocchio ABI 호환 |
| openvr | pip | Vive Tracker SteamVR API |
| pynput | pip | 키보드 상태 (vive_sender, manus_sender, realsense_sender) |
| pyyaml | pip | YAML 설정 파싱 |
| pygame | pip | 조이스틱 입력 |
| libusb | apt/conda | Manus SDK |
| dex_retargeting | pip | manus_sender (dex 모드) + realsense_sender |
| mediapipe==0.10.21 | pip | MediaPipe HandLandmarker (realsense_sender) |
| pyrealsense2 | pip | Intel RealSense D405 (realsense_sender) |
| opencv-python | pip | 시각화 + BGR↔RGB (realsense_sender) |

### 로봇 PC (robot/)

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip | 수치 연산 (pinocchio ABI 호환) |
| pinocchio | apt (`ros-humble-pinocchio`) | FK/Jacobian/IK |
| pin-pink | pip | QP 기반 IK (Pink) |
| proxsuite | pip | QP solver 백엔드 |
| ur-rtde | pip | UR10e RTDE 통신 |
| pymodbus | pip | Tesollo DG5F Modbus TCP |
| PyYAML | pip | YAML 설정 파싱 |
| rclpy | apt (ROS2) | SimBackend, ControllerSwitcher |
| sensor_msgs | apt (ROS2) | JointState 메시지 |

---

## 주의사항

- **`pip install pink` 금지**: `pink`는 코드 포매터. IK 라이브러리는 **`pip install pin-pink`**
- **numpy < 2.0 필수**: pinocchio가 numpy 1.x ABI로 컴파일됨
- **pymodbus >= 3.10 필수**: `device_id=` 파라미터 API 사용
- **RTDEControlInterface 사용 금지** (impedance 모드): UR10e에서 hang 발생. RTDEIOInterface + TCP socket 조합 사용
