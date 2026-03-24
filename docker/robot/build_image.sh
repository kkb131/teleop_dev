#!/bin/bash
# =============================================================================
# build_image.sh — Teleop Robot PC Docker 이미지 빌드
#
# Usage:
#   ./build_image.sh                  # Auto-detect arch
#   ./build_image.sh --arch amd64     # Force amd64 (dev PC)
#   ./build_image.sh --arch arm64     # Force arm64 (Jetson AGX Orin)
#   ./build_image.sh --no-cache
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME="teleop_robot"
IMAGE_TAG="latest"
ARCH=""
DOCKER_BUILD_ARGS=""

declare -A BASE_IMAGES=(
    ["amd64"]="nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907"
    ["arm64"]="nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c"
)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arch) ARCH="$2"; shift 2 ;;
        --no-cache) DOCKER_BUILD_ARGS="${DOCKER_BUILD_ARGS} --no-cache"; shift ;;
        --tag) IMAGE_TAG="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--arch amd64|arm64] [--no-cache] [--tag TAG]"
            exit 0 ;;
        *) echo "ERROR: Unknown argument: $1"; exit 1 ;;
    esac
done

if [ -z "${ARCH}" ]; then
    case "$(uname -m)" in
        x86_64)  ARCH="amd64" ;;
        aarch64) ARCH="arm64" ;;
        *) echo "ERROR: Unknown arch $(uname -m). Use --arch."; exit 1 ;;
    esac
    echo "[build] Auto-detected: ${ARCH}"
fi

BASE_IMAGE="${BASE_IMAGES[${ARCH}]}"
FULL_TAG="${IMAGE_NAME}:${IMAGE_TAG}-${ARCH}"

echo "============================================"
echo "  Building Teleop Robot Image"
echo "  Base:  ${BASE_IMAGE}"
echo "  Tag:   ${FULL_TAG}"
echo "  Arch:  ${ARCH}"
echo "============================================"

docker build \
    ${DOCKER_BUILD_ARGS} \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "${FULL_TAG}" \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "  Build Complete: ${FULL_TAG}"
echo "  Next: ./run_container.sh"
