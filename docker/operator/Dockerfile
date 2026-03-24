# =============================================================================
# Teleop Operator PC — 조종 PC Docker 환경
# 경량 Python 이미지 (ROS2 불필요)
# Target: amd64 (Ubuntu/Windows WSL2)
# =============================================================================

FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive

# --- System dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# --- Python dependencies ---
RUN pip install --no-cache-dir \
    "openvr>=1.26.701" \
    "numpy" \
    "pynput>=1.7.6" \
    "pyyaml>=6.0" \
    "pygame>=2.0.0"

# --- Workspace ---
WORKDIR /app
COPY . /app/teleop_dev/

ENV PYTHONPATH="/app:${PYTHONPATH}"

# NOTE: SteamVR must run on the HOST, not inside Docker.
# openvr connects to SteamVR via shared memory / IPC.
# For Vive Tracker, run this container with:
#   --network host --ipc host

CMD ["bash"]
