#!/usr/bin/env bash
# 车侧 SWARM-LIO 仿真环境(在容器 xtdrone-swarm-lio 内 source)。
# 只依赖: /opt/ros/noetic + ws_livox(swarm_lio, livox CustomMsg 插件) + 挂载的 swarm_defense_ws。
# 注意: 不要 source mid360_car3 devel —— 那里有同名 liblivox_laser_simulation.so(PointCloud2 版),
# 会与 ws_livox 的 CustomMsg 版抢 GAZEBO_PLUGIN_PATH。
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/noetic/setup.bash
fi
source /home/dev/XTDrone-single-car/ws_livox/devel/setup.bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash

export DISPLAY="${HOST_DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1

DEF_WS=/home/dev/XTDrone-single-car/workspace/swarm_defense_ws
LIVOX_WS=/home/dev/XTDrone-single-car/ws_livox
LIVOX_SRC=${LIVOX_WS}/src/livox_laser_simulation_Mid360

# gazebo 模型路径: basic_room_sim(models:nesting_room_mesh/camera_gimbal/…) + livox(models:Mid360 → csv)
export GAZEBO_MODEL_PATH="${DEF_WS}/src/basic_room_sim/models:${LIVOX_SRC}/models"
export GAZEBO_RESOURCE_PATH="${DEF_WS}/src/basic_room_sim:/usr/share/gazebo-11"
export GAZEBO_MODEL_DATABASE_URI=""        # 禁 fuel 下载
# 插件路径顺序关键: ws_livox(CustomMsg) 在前
export GAZEBO_PLUGIN_PATH="${LIVOX_WS}/devel/lib:${DEF_WS}/devel/lib:/opt/ros/noetic/lib"
export ROS_PACKAGE_PATH="${DEF_WS}/src:${LIVOX_WS}/src:${ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="${LIVOX_WS}/devel/lib:${DEF_WS}/devel/lib:${LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="${LIVOX_WS}/devel:${DEF_WS}/devel:${CMAKE_PREFIX_PATH}"
# catkin 的 setup.bash 这里没给 python3 补 dist-packages, 手工补上(否则 import livox_ros_driver2 失败)
export PYTHONPATH="${LIVOX_WS}/devel/lib/python3/dist-packages:${DEF_WS}/devel/lib/python3/dist-packages:${PYTHONPATH}"
