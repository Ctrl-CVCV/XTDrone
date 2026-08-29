#!/bin/bash
# 云台相机仿真演示启动脚本（在容器 xtdrone-dev 内运行）
# 用法:
#   docker exec -it xtdrone-dev bash /workspace/gimbal_camera/gimbal_demo.sh
# 一条命令全自动：Gazebo(测试世界:底座+彩色方块) + spawn 云台相机 + ros_control 关节控制
# + 正弦摆动演示 + RViz(相机画面/模型/TF)。窗口直连宿主 X(:1) NVIDIA GPU 渲染。
# 演示期间终端保持前台占用，按 Ctrl+C 一键关闭全部仿真进程。
# 前置条件: 宿主机 X 已授权容器连接（X 重启后需重跑一次: xhost +local:）
#
# 手动控制云台（另开终端）:
#   docker exec xtdrone-dev bash -c 'source /home/dev/car3_env.sh && export DISPLAY=:1 && \
#     rostopic pub -1 /gimbal/gimbal_yaw_controller/command std_msgs/Float64 "data: 0.8" && \
#     rostopic pub -1 /gimbal/gimbal_roll_controller/command std_msgs/Float64 "data: 0.5"'
# 查看相机话题: rostopic hz /gimbal_camera/image_raw
set -e

GC_DIR=/workspace/gimbal_camera
source /home/dev/car3_env.sh

# 渲染目标：宿主机真实桌面（GPU 直连）
export DISPLAY=${HOST_DISPLAY:-:1}

# car3_env.sh 只带 PX4 模型路径；补上 gazebo 自带模型目录（缺了会触发 fuel 下载卡死，v1.2 教训）
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

echo "==> 渲染 DISPLAY=$DISPLAY (NVIDIA GPU 直连)"
if ! xdpyinfo >/dev/null 2>&1; then
    echo "!!! 无法连接宿主 X $DISPLAY（需先执行: xhost +local:）"
    exit 1
fi
glxinfo -B 2>/dev/null | grep -E "direct rendering|OpenGL renderer" | sed 's/^/    /' || true

echo "==> [1/6] 清理残留进程"
for p in rosmaster roslaunch gzserver gzclient rviz robot_state_publisher controller_spawner spawn_model gimbal_move gimbal_cam_check; do
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
    [ -n "${GAZ_PID:-}" ] && kill -INT "$GAZ_PID" 2>/dev/null || true
    [ -n "${CTRL_PID:-}" ] && kill -INT "$CTRL_PID" 2>/dev/null || true
    [ -n "${RVIZ_PID:-}" ] && kill -INT "$RVIZ_PID" 2>/dev/null || true
    [ -n "${MOVE_PID:-}" ] && kill -INT "$MOVE_PID" 2>/dev/null || true
    sleep 3
    for p in rosmaster roslaunch gzserver gzclient rviz robot_state_publisher controller_spawner spawn_model gimbal_move gimbal_cam_check; do
        pkill -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
    done
    echo "==> 仿真已全部关闭"
    exit 0
}
trap cleanup INT TERM EXIT

echo "==> [2/6] 启动 Gazebo 测试世界（底座柱 + 彩色方块环）"
roslaunch gazebo_ros empty_world.launch gui:=true \
    world_name:=$GC_DIR/worlds/gimbal_test.world > /tmp/gimbal_gazebo.log 2>&1 &
GAZ_PID=$!
echo "    gazebo PID=$GAZ_PID  日志: /tmp/gimbal_gazebo.log"

echo "==> [3/6] 等待 gazebo 就绪并 spawn 云台相机模型"
for i in $(seq 1 60); do
    rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model' && break
    sleep 2
done
rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model' || { echo "!!! gazebo 未就绪"; exit 1; }
# 先加载 robot_description 参数（控制器/URDF 插件加载时需要），再 spawn 模型
roslaunch $GC_DIR/launch/gimbal_controllers.launch > /tmp/gimbal_ctrl.log 2>&1 &
CTRL_PID=$!
echo "    controllers PID=$CTRL_PID  日志: /tmp/gimbal_ctrl.log"
sleep 2
rosrun gazebo_ros spawn_model -file $GC_DIR/robot.urdf -urdf \
    -model gimbal_camera -x 0 -y 0 -z 0.5 >> /tmp/gimbal_ctrl.log 2>&1
echo "    云台相机已 spawn 在底座柱顶 (0, 0, 0.5)"

echo "==> [4/6] 等待控制器与关节话题就绪"
for i in $(seq 1 30); do
    rostopic list 2>/dev/null | grep -q '/gimbal/gimbal_yaw_controller/command' && break
    sleep 2
done
rostopic list 2>/dev/null | grep -q '/gimbal/gimbal_yaw_controller/command' || { echo "!!! 控制器未就绪，见 /tmp/gimbal_ctrl.log"; exit 1; }
echo "    控制器就绪: /gimbal/joint_states + yaw/roll command 话题"

echo "==> [5/6] 启动云台正弦摆动演示 + 相机画面验证"
python3 $GC_DIR/scripts/gimbal_move.py > /tmp/gimbal_move.log 2>&1 &
MOVE_PID=$!
for i in $(seq 1 30); do
    rostopic list 2>/dev/null | grep -q '^/gimbal_camera/image_raw$' && break
    sleep 2
done
rostopic list 2>/dev/null | grep -q '^/gimbal_camera/image_raw$' || { echo "!!! 相机话题未出现，见 /tmp/gimbal_gazebo.log"; exit 1; }
python3 $GC_DIR/scripts/gimbal_cam_check.py || echo "    (相机画面检查未通过，请查看 RViz 确认)"

echo "==> [6/6] 启动 RViz + 窗口摆放"
rviz -d $GC_DIR/config/gimbal.rviz > /tmp/gimbal_rviz.log 2>&1 &
RVIZ_PID=$!
echo "    rviz PID=$RVIZ_PID"
for i in $(seq 1 20); do
    xdotool search --name "Gazebo" >/dev/null 2>&1 && xdotool search --name "RViz" >/dev/null 2>&1 && break
    sleep 2
done
xdotool search --name "Gazebo" windowmove 0 0 windowsize 2200 1500 2>/dev/null
xdotool search --name "RViz" windowmove 2200 0 windowsize 2920 1500 2>/dev/null
echo "    窗口已摆放: Gazebo 左 / RViz 右（RViz 含相机画面/模型/TF）"

echo ""
echo "=============================="
echo " 云台相机仿真已就绪！"
echo "   Gazebo  : 云台相机在底座柱上，yaw/roll 正弦摆动"
echo "   RViz    : Image 显示 /gimbal_camera/image_raw + RobotModel + TF"
echo "   相机话题 : /gimbal_camera/image_raw (30Hz, 640x480)  /gimbal_camera/camera_info"
echo "   手动控制 : rostopic pub -1 /gimbal/gimbal_yaw_controller/command std_msgs/Float64 \"data: 0.8\""
echo "   停止     : 按 Ctrl+C 关闭全部仿真进程"
echo "=============================="

# 常驻前台：演示期间终端保持占用；Ctrl+C 触发 cleanup 一键关闭
wait "$GAZ_PID"
