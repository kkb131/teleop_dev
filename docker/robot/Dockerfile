# =============================================================================
# Teleop Robot PC — 로봇 PC Docker 환경
# Isaac ROS (ROS2 Humble + CUDA) 기반
# Target: arm64 (Jetson AGX Orin) / amd64 (개발 PC)
# =============================================================================

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# =============================================================================
# Layer 0: Fix base image apt sources
# =============================================================================
RUN sed -i 's/ external-main//' /etc/apt/sources.list.d/nvidia-isaac-ros.list 2>/dev/null || true \
    && rm -f /etc/apt/sources.list.d/yarn.list 2>/dev/null || true

# =============================================================================
# Layer 1: System tools
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim nano htop tmux gdb less tree \
    bash-completion \
    iproute2 iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 2: ROS2 robot control dependencies
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-diagnostic-updater \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 3: Python pip dependencies (teleop robot)
#
#   pin-pink    - QP 기반 IK (pinocchio 위에 구축)
#                 주의: "pip install pink"는 코드 포매터!
#   proxsuite   - QP solver 백엔드
#   ur-rtde     - UR 로봇 RTDE 통신 (servoJ + URScript socket)
#   PyYAML      - YAML 설정 파싱
#   numpy <2.0  - pinocchio ABI 호환
#   pyzmq       - robot/cam D405 컬러 스트리밍 ZMQ PUB (JPEG)
#   opencv-python-headless - JPEG 인코딩 (robot/cam, GUI 불필요)
#   pyrealsense2 - D405 컬러 캡처 (robot/cam)
#                 주의: 컨테이너에서 D405 접근하려면 run_container.sh 에
#                 USB 장치 전달 필요 (예: -v /dev:/dev --device-cgroup-rule
#                 또는 --privileged)
#
# 참고: DG5F 하드웨어 제어는 dg5f_driver (ROS2) 가 담당.
# robot/hand/* 는 ROS2 토픽으로만 통신하므로 pymodbus 불필요.
# =============================================================================
RUN pip install --no-cache-dir \
    "pin-pink>=4.0.0" \
    "proxsuite>=0.6.0" \
    "ur-rtde>=1.5.0" \
    "PyYAML>=6.0" \
    "numpy<2.0" \
    "pyzmq>=25" \
    "opencv-python-headless" \
    "pyrealsense2"

# =============================================================================
# Layer 4: rosdep cache
# =============================================================================
RUN rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro=${ROS_DISTRO} 2>/dev/null || true

# =============================================================================
# Layer 5: Workspace
# =============================================================================
RUN mkdir -p /workspaces/teleop_ws/src

WORKDIR /workspaces/teleop_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
