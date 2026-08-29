#!/bin/bash
# 独立 mid-360 小车一键启动（在容器 xtdrone-dev-gpu 内运行）
# 用法:
#   docker exec xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/mid360_car3/run_mid360_car3.sh [red_marker]
#     无参 = 围捕小车变体（无红块，默认）；参数 true/false 切换红块（入侵机变体 true）。
#     例: bash run_mid360_car3.sh true   # 入侵机带红块
# 启动 Gazebo(GUI) + car3 麦轮底盘(带 MID-360 mesh) + livox 点云(/scan, PointCloud2) + RViz。
# 按 Ctrl+C 一键关闭全部仿真进程。前置: 宿主 X 已授权 (xhost +local:)。
set -e

RED_MARKER="${1:-false}"

MID360_WS=/home/dev/XTDrone-single-car/workspace/mid360_car3
source /home/dev/car3_env.sh

export DISPLAY=${HOST_DISPLAY:-:1}
echo "==> 渲染 DISPLAY=$DISPLAY (NVIDIA GPU 直连)"

if ! glxinfo -B >/dev/null 2>&1; then
    echo "!!! 无法连接宿主 X $DISPLAY（需先执行: xhost +local:，且容器需装 mesa-utils 的 glxinfo）"
    exit 1
fi

# 追加 mid360_car3 工作空间路径。不能直接 source 其 devel/setup.bash：
# catkin_tools 的 setup 会覆盖 ROS_PACKAGE_PATH，丢掉 car3_env.sh 里的自定义路径。
export CMAKE_PREFIX_PATH="$MID360_WS/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$MID360_WS/src/livox_laser_simulation:$MID360_WS/src/mid360_car3:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$MID360_WS/devel/lib:$LD_LIBRARY_PATH"
export GAZEBO_PLUGIN_PATH="$MID360_WS/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

echo "==> 清理残留进程"
for p in rosmaster roslaunch gzserver gzclient rviz controller_spawner mecanum_controller ground_truth_odom robot_state_publisher; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
done
sleep 2

echo "==> roslaunch mid360_car3 mid360_car3.launch (GUI, /scan=PointCloud2) red_marker=$RED_MARKER"
exec roslaunch mid360_car3 mid360_car3.launch red_marker:=$RED_MARKER
