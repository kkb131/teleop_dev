#!/bin/bash
# Servo 기능 실행 가이드
#
# 사용법: bash robot/arm/servo/launch_servo.sh [mode]
#   mode: keyboard_forward  - Forward Position Controller 키보드 테스트
#         keyboard_servo    - MoveIt Servo 키보드 Cartesian 텔레옵
#         joystick_servo    - MoveIt Servo Xbox 조이스틱 텔레옵
#         info              - 실행 방법 안내 (기본값)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="/workspaces/tamp_ws"
MODE="${1:-info}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

show_info() {
    echo -e "${CYAN}═══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}          UR10e Servo Control Guide${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${GREEN}[1] Forward Position Controller (키보드)${NC}"
    echo "    추가 설치 없이 바로 테스트 가능. Joint 단위 직접 제어."
    echo ""
    echo -e "    ${YELLOW}# T1: UR Driver${NC}"
    echo "    ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0"
    echo ""
    echo -e "    ${YELLOW}# T2: MoveIt + RViz${NC}"
    echo "    ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e"
    echo ""
    echo -e "    ${YELLOW}# T3: Keyboard teleop${NC}"
    echo "    cd ${WS_DIR}/src/tamp_dev && python3 -m robot.arm.servo.keyboard_forward"
    echo ""
    echo -e "${GREEN}[2] MoveIt Servo (키보드 Cartesian)${NC}"
    echo "    Cartesian 제어. 충돌 체크 + 특이점 감지 내장."
    echo ""
    echo -e "    ${YELLOW}# T1: UR Driver${NC}"
    echo "    ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0"
    echo ""
    echo -e "    ${YELLOW}# T2: MoveIt + RViz + Servo${NC}"
    echo "    ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e launch_servo:=true"
    echo ""
    echo -e "    ${YELLOW}# T3: Keyboard teleop${NC}"
    echo "    cd ${WS_DIR}/src/tamp_dev && python3 servo/keyboard_servo.py"
    echo ""
    echo -e "${GREEN}[3] MoveIt Servo (Xbox 조이스틱)${NC}"
    echo "    Xbox 컨트롤러로 Cartesian 제어."
    echo ""
    echo -e "    ${YELLOW}# T1~T2: 위와 동일 (launch_servo:=true 필수)${NC}"
    echo ""
    echo -e "    ${YELLOW}# T3: Joy node${NC}"
    echo "    ros2 run joy joy_node"
    echo ""
    echo -e "    ${YELLOW}# T4: Joystick teleop${NC}"
    echo "    cd ${WS_DIR}/src/tamp_dev && python3 servo/joystick_servo.py"
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════${NC}"
    echo -e "  source ${WS_DIR}/install/setup.bash  (moveit_servo 빌드 후)"
    echo -e "${CYAN}═══════════════════════════════════════════════════${NC}"
}

case "$MODE" in
    keyboard_forward)
        echo -e "${GREEN}Starting Forward Position Controller keyboard teleop...${NC}"
        cd "${WS_DIR}/src/tamp_dev"
        python3 -m robot.arm.servo.keyboard_forward
        ;;
    keyboard_servo)
        echo -e "${GREEN}Starting MoveIt Servo keyboard Cartesian teleop...${NC}"
        cd "${WS_DIR}/src/tamp_dev"
        python3 servo/keyboard_servo.py
        ;;
    joystick_servo)
        echo -e "${GREEN}Starting MoveIt Servo joystick teleop...${NC}"
        cd "${WS_DIR}/src/tamp_dev"
        python3 servo/joystick_servo.py
        ;;
    info|*)
        show_info
        ;;
esac
