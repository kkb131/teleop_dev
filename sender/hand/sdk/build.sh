#!/usr/bin/env bash
# Build SDKClient_Linux with --stream-json support
# Usage: bash sender/hand/sdk/build.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/SDKClient_Linux"

echo "Building SDKClient_Linux..."
make clean 2>/dev/null || true
make

echo ""
echo "Build complete: $(pwd)/SDKClient_Linux.out"
echo ""
echo "Test headless mode:"
echo "  cd $(pwd)"
echo "  ./SDKClient_Linux.out --stream-json"
