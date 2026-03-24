#!/bin/bash
# =============================================================================
# run_container.sh — Teleop Robot PC 컨테이너 실행
#
# Usage:
#   ./run_container.sh
#   ./run_container.sh --join
#   ./run_container.sh --no-devices
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"   # teleop_dev/

IMAGE_NAME="teleop_robot"
IMAGE_TAG="latest"
CONTAINER_NAME="teleop_robot"
JOIN_EXISTING=false
MOUNT_DEVICES=true
USER_CMD=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --join) JOIN_EXISTING=true; shift ;;
        --name) CONTAINER_NAME="$2"; shift 2 ;;
        --no-devices) MOUNT_DEVICES=false; shift ;;
        -h|--help)
            echo "Usage: $0 [--join] [--name NAME] [--no-devices] [COMMAND]"
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

# GPU passthrough
GPU_ARGS=""
IS_JETSON=false
if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON=true
    GPU_ARGS="--runtime nvidia --gpus all"
elif [ -e /dev/nvidia0 ] || docker info 2>/dev/null | grep -qE "nvidia"; then
    GPU_ARGS="--gpus all"
fi

# Device mounting
DEVICE_ARGS=()
if [ "${IS_JETSON}" = true ]; then
    DEVICE_ARGS+=(--privileged)
    MOUNT_DEVICES=false
fi

if [ "${MOUNT_DEVICES}" = true ]; then
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "${dev}" ] && DEVICE_ARGS+=(--device "${dev}:${dev}")
    done
    [ -d /dev/bus/usb ] && DEVICE_ARGS+=(-v "/dev/bus/usb:/dev/bus/usb")
fi

# X11
DISPLAY_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    xhost +local:root 2>/dev/null || true
    DISPLAY_ARGS+=(-e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
fi

echo "============================================"
echo "  Starting: ${CONTAINER_NAME}"
echo "  Image:    ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================"

docker run -it \
    --name "${CONTAINER_NAME}" \
    ${GPU_ARGS} \
    --network host \
    "${DEVICE_ARGS[@]}" \
    "${DISPLAY_ARGS[@]}" \
    -v "${PROJECT_DIR}:/workspaces/teleop_ws/src/teleop_dev:rw" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -e "TERM=xterm-256color" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    ${USER_CMD:-bash}
