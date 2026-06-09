#!/bin/bash
# UR10e servo (단순 DLS IK) 실행 가이드 — info-only.
#
# Usage: bash robot/arm/servo/launch_servo.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="/workspaces/tamp_ws"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

cat <<EOF
${CYAN}═══════════════════════════════════════════════════${NC}
${CYAN}     UR10e Servo Control (DLS IK / Pinocchio)${NC}
${CYAN}═══════════════════════════════════════════════════${NC}

robot/arm/servo 모듈은 네트워크 입력 없이 로컬 키보드/조이스틱으로
UR10e 를 직접 제어. 자세한 옵션은 각 모듈의 --help 참조.

${GREEN}[1] Joint-space (forward) — 키보드${NC}
    python3 -m robot.arm.servo.keyboard_forward --mode sim
    python3 -m robot.arm.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2

${GREEN}[2] Cartesian (DLS IK) — 키보드${NC}
    python3 -m robot.arm.servo.keyboard_cartesian --mode sim
    python3 -m robot.arm.servo.keyboard_cartesian --mode rtde --robot-ip 192.168.0.2

${GREEN}[3] Cartesian (DLS IK) — Xbox joystick (ROS2 /joy 필요)${NC}
    ros2 run joy joy_node
    python3 -m robot.arm.servo.joystick_cartesian --mode sim

${GREEN}[4] Cartesian + F/T admittance — 키보드 (rtde 모드 전용)${NC}
    python3 -m robot.arm.servo.keyboard_servo_admittance --mode rtde --robot-ip 192.168.0.2

${YELLOW}sim 모드 ROS2 전제:${NC}
    ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
    또는 Isaac Sim 의 UR10e + DG5F scene.

${CYAN}═══════════════════════════════════════════════════${NC}
EOF
