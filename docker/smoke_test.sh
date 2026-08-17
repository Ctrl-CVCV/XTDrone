#!/usr/bin/env bash

echo "========== OS =========="
grep PRETTY_NAME /etc/os-release

echo "========== ROS =========="
echo "ROS_DISTRO=${ROS_DISTRO}"
rosversion -d

echo "========== Gazebo =========="
gazebo --version

echo "========== PX4 =========="
cd "$HOME/PX4-Autopilot" || exit 1
git describe --tags --always

echo "========== XTDrone =========="
cd "$HOME/XTDrone" || exit 1
git rev-parse HEAD

echo "========== MAVROS =========="
rospack find mavros

echo "========== Navigation =========="
rospack find move_base

echo "========== GPU/OpenGL =========="
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total \
        --format=csv,noheader || true
fi

glxinfo -B 2>/dev/null | \
    grep -E "OpenGL vendor|OpenGL renderer|OpenGL version" || true

echo "========== Done =========="
