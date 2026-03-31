#!/bin/bash
# =============================================================================
# run_container.sh — Teleop Operator PC 컨테이너 실행
#
# Usage:
#   ./run_container.sh
#   ./run_container.sh --join
#   ./run_container.sh python3 -m operator.arm.keyboard_sender --target-ip 10.0.0.5
# =============================================================================

set -euo pipefail

IMAGE_NAME="teleop_operator"
IMAGE_TAG="latest"
CONTAINER_NAME="teleop_operator"
JOIN_EXISTING=false
USER_CMD=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --join) JOIN_EXISTING=true; shift ;;
        --name) CONTAINER_NAME="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--join] [--name NAME] [COMMAND]"
            exit 0 ;;
        *) USER_CMD="$*"; break ;;
    esac
done

if [ "${JOIN_EXISTING}" = true ]; then
    docker exec -it "${CONTAINER_NAME}" bash
    exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
fi

DEVICE_ARGS=()
# Joystick devices
for dev in /dev/input/js*; do
    [ -e "${dev}" ] && DEVICE_ARGS+=(--device "${dev}:${dev}")
done
# USB bus (Manus dongle, Vive dongle)
[ -d /dev/bus/usb ] && DEVICE_ARGS+=(-v "/dev/bus/usb:/dev/bus/usb")

docker run -it \
    --name "${CONTAINER_NAME}" \
    --network host \
    --ipc host \
    "${DEVICE_ARGS[@]}" \
    -e "TERM=xterm-256color" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    ${USER_CMD:-bash}
