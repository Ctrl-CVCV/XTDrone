#!/bin/bash
# car3 全链路演示启动脚本（在容器 xtdrone-dev 内运行）
# 用法:
#   docker exec -it xtdrone-dev bash /workspace/car3_demo.sh
# 一条命令全自动：Gazebo + ros_control + 三阶段 nav_to_pose + RViz 全部在容器内启动，
# 直连宿主 X(:1) NVIDIA GPU 渲染，窗口直接出现在宿主机桌面（无 VNC）。
# 演示期间终端保持前台占用，按 Ctrl+C 一键关闭全部仿真进程。
# 前置条件: 宿主机 X 已授权容器连接（X 重启后需重跑一次: xhost +local:）
set -e

source /home/dev/car3_env.sh

# 渲染目标：宿主机真实桌面（GPU 直连）
export DISPLAY=${HOST_DISPLAY:-:1}

echo "==> 渲染 DISPLAY=$DISPLAY (NVIDIA GPU 直连)"

if ! xdpyinfo >/dev/null 2>&1; then
    echo "!!! 无法连接宿主 X $DISPLAY（需先执行: xhost +local:）"
    exit 1
fi
glxinfo -B 2>/dev/null | grep -E "direct rendering|OpenGL renderer" | sed 's/^/    /' || true

echo "==> [1/4] 清理残留进程"
for p in rosmaster roslaunch gzserver gzclient move_base nav_to_pose amcl map_server slam_gmapping mecanum_controller ground_truth_odom controller_spawner robot_state_publisher rviz; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
done
sleep 3

# Ctrl+C 信号处理：关闭本次启动的全部仿真进程（幂等，可重复触发）
CLEANED=0
cleanup() {
    [ "$CLEANED" = "1" ] && exit 0
    CLEANED=1
    echo ""
    echo "==> 收到退出信号，正在关闭仿真..."
    if [ -n "${NAV_PID:-}" ]; then kill -INT "$NAV_PID" 2>/dev/null || true; fi
    if [ -n "${RVIZ_PID:-}" ]; then kill -INT "$RVIZ_PID" 2>/dev/null || true; fi
    sleep 3
    for p in rosmaster roslaunch gzserver gzclient move_base nav_to_pose amcl map_server slam_gmapping mecanum_controller ground_truth_odom controller_spawner robot_state_publisher rviz; do
        pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
    done
    echo "==> 仿真已全部关闭"
    exit 0
}
trap cleanup INT TERM EXIT

echo "==> [2/4] 启动 Gazebo + ros_control + mecanum + odom + lidar + map_server + AMCL + nav_to_pose(三阶段)"
roslaunch car3_control car3_nav_to_pose.launch gui:=true > /tmp/car3_demo.log 2>&1 &
NAV_PID=$!
echo "    nav stack PID=$NAV_PID  日志: /tmp/car3_demo.log"

echo "==> [3/4] 等待 nav_to_pose 就绪"
until rostopic list 2>/dev/null | grep -q '^/move_base/goal$'; do sleep 1; done
echo "    nav_to_pose action 接口已出现，等待 AMCL 收敛"
sleep 10

echo "==> [4/4] 启动 RViz + 窗口摆放"
rviz -d "$(rospack find car3_control)/rviz/car3_nav.rviz" > /tmp/car3_rviz.log 2>&1 &
RVIZ_PID=$!
echo "    rviz PID=$RVIZ_PID"
# 等待窗口出现后摆放：Gazebo 左 / RViz 右
for i in $(seq 1 20); do
    xdotool search --name "Gazebo" >/dev/null 2>&1 && xdotool search --name "RViz" >/dev/null 2>&1 && break
    sleep 2
done
xdotool search --name "Gazebo" windowmove 0 0 windowsize 2200 1500 2>/dev/null
xdotool search --name "RViz" windowmove 2200 0 windowsize 2920 1500 2>/dev/null
echo "    窗口已摆放: Gazebo 左 / RViz 右"

echo ""
echo "=============================="
echo " 演示环境已就绪！宿主机桌面直接显示 Gazebo + RViz (NVIDIA GPU 渲染)"
echo "   RViz 内容: 地图 / 激光 / AMCL 粒子 / TF (三阶段导航: 先转向→直线平移→到位后转期望朝向)"
echo "   发目标方法1 : RViz 工具栏 [2D Nav Goal]，在地图上点位置并拖出方向"
echo "   发目标方法2 : python3 /tmp/nav_goal.py '1.5,-1.5,0' '-1.0,-0.5,0' '-2.0,-1.0,0' '0.5,-2.0,0' '0.0,-0.5,0'"
echo "   停止: 按 Ctrl+C 关闭全部仿真进程"
echo "=============================="

# 常驻前台：演示期间终端保持占用；Ctrl+C 触发 cleanup 一键关闭
wait "$NAV_PID"
