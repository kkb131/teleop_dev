#!/bin/bash
# =============================================================================
# build_image.sh — Teleop Operator PC Docker 이미지 빌드
#
# Usage:
#   ./build_image.sh
#   ./build_image.sh --no-cache
#   ./build_image.sh --tag v1.0
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"   # teleop_dev/

IMAGE_NAME="teleop_operator"
IMAGE_TAG="latest"
DOCKER_BUILD_ARGS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache) DOCKER_BUILD_ARGS="${DOCKER_BUILD_ARGS} --no-cache"; shift ;;
        --tag) IMAGE_TAG="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--no-cache] [--tag TAG]"
            exit 0 ;;
        *) echo "ERROR: Unknown argument: $1"; exit 1 ;;
    esac
done

FULL_TAG="${IMAGE_NAME}:${IMAGE_TAG}"

echo "============================================"
echo "  Building Teleop Operator Image"
echo "  Tag:  ${FULL_TAG}"
echo "  Context: ${PROJECT_DIR}"
echo "============================================"

docker build \
    ${DOCKER_BUILD_ARGS} \
    -t "${FULL_TAG}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${PROJECT_DIR}"

echo ""
echo "  Build Complete: ${FULL_TAG}"
echo "  Next: ./run_container.sh"
