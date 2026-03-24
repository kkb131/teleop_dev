#!/bin/bash
set -e

# Source ROS2
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay
if [ -f /workspaces/teleop_ws/install/setup.bash ]; then
    source /workspaces/teleop_ws/install/setup.bash
    echo "[entrypoint] Sourced workspace overlay."
fi

# Generate URDF if ur_description available
URDF_TARGET=/workspaces/teleop_ws/ur10e.urdf
if [ ! -f "${URDF_TARGET}" ]; then
    if command -v xacro &>/dev/null && ros2 pkg prefix ur_description &>/dev/null 2>&1; then
        XACRO=$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro
        xacro "${XACRO}" ur_type:=ur10e name:=ur > "${URDF_TARGET}" 2>/dev/null && \
            echo "[entrypoint] Generated ur10e.urdf."
    fi
fi

exec "$@"
