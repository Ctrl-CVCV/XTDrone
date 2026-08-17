#!/bin/bash
# 6_6_room 手动建图启动脚本（在容器 xtdrone-dev 内运行）
# 用法:
#   docker exec -it xtdrone-dev bash /workspace/car3_slam_66room.sh
# 一条命令启动 Gazebo(6_6_room) + car3 + gmapping + RViz + 键盘控制。
# 在启动终端里用键盘开车:
#   w/s 前进/后退   a/d 左移/右移   q/e 左转/右转
#   +/- 线速度档   ]/[ 角速度档   空格 停止   t 帮助
# 建图完成后另开终端保存地图:
#   docker exec xtdrone-dev bash -c "source /home/dev/car3_env.sh && \
#     rosrun map_server map_saver -f /workspace/swarm_defense_ws/src/car3_control/maps/6_6_room"
# 终端按 Ctrl+C 一键关闭全部仿真进程。
set -e

source /home/dev/car3_env.sh
export DISPLAY=${HOST_DISPLAY:-:1}

echo "==> 渲染 DISPLAY=$DISPLAY (NVIDIA GPU 直连)"
if ! xdpyinfo >/dev/null 2>&1; then
    echo "!!! 无法连接宿主 X $DISPLAY（需先执行: xhost +local:）"
    exit 1
fi

echo "==> [1/3] 清理残留进程"
for p in rosmaster roslaunch gzserver gzclient nav_to_pose amcl map_server slam_gmapping mecanum_controller ground_truth_odom controller_spawner robot_state_publisher rviz; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
done
sleep 3

CLEANED=0
cleanup() {
    [ "$CLEANED" = "1" ] && exit 0
    CLEANED=1
    echo ""
    echo "==> 收到退出信号，正在关闭仿真..."
    for p in rosmaster roslaunch gzserver gzclient nav_to_pose amcl map_server slam_gmapping mecanum_controller ground_truth_odom controller_spawner robot_state_publisher rviz car3_keyboard; do
        pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
    done
    echo "==> 仿真已全部关闭"
    exit 0
}
trap cleanup INT TERM EXIT

echo "==> [2/3] 启动 Gazebo(6_6_room) + gmapping + 键盘控制 + RViz"
roslaunch car3_control car3_slam_66room.launch gui:=true

echo "==> [3/3] 仿真结束"
