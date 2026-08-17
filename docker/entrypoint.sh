#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

PX4_DIR="$HOME/PX4-Autopilot"

if [ -d "$PX4_DIR" ]; then
    export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR"

    # PX4 v1.13.x 路径
    if [ -d "$PX4_DIR/Tools/sitl_gazebo" ]; then
        export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR/Tools/sitl_gazebo"
    fi

    if [ -d "$PX4_DIR/build/px4_sitl_default" ] && \
       [ -f "$PX4_DIR/Tools/setup_gazebo.bash" ]; then
        source "$PX4_DIR/Tools/setup_gazebo.bash" \
               "$PX4_DIR" \
               "$PX4_DIR/build/px4_sitl_default"
    fi
fi

if [ -f "$HOME/ego_ws/devel/setup.bash" ]; then
    source "$HOME/ego_ws/devel/setup.bash"
fi

if [ -f "/workspace/swarm_defense_ws/devel/setup.bash" ]; then
    source "/workspace/swarm_defense_ws/devel/setup.bash"
fi

export PATH="$HOME/.local/bin:$PATH"

exec "$@"
