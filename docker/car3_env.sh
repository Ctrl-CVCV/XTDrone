#!/usr/bin/env bash
# car3 地面仿真标准环境（docker exec 用，等价 entrypoint 环境子集）
source /opt/ros/noetic/setup.bash
source /home/dev/catkin_ws/devel/setup.bash 2>/dev/null
source /workspace/swarm_defense_ws/devel/setup.bash 2>/dev/null
# XTDrone sitl_config/ugv 中的 car3 描述包（仅 URDF/meshes，不在任何 workspace 内）
export ROS_PACKAGE_PATH=/home/dev/XTDrone/sitl_config/ugv:$ROS_PACKAGE_PATH
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
