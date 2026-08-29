#!/usr/bin/env bash
# =============================================================================
# 空中入侵无人机统一仿真启动脚本（Phase 3）
#
# 用法（宿主终端）:
#   docker exec -it xtdrone-dev bash /workspace/launch_air_intruder_sim.sh [--spawn-mode random|fixed] [--corner CORNER_0|CORNER_1|CORNER_2] [--uav-speed <m/s>] [--uav-acc <m/s2>] [--intruder-speed <m/s>] [--intruder-acc <m/s2>] [--gui|--headless]
#
# 功能:
#   1. 清理残留仿真
#   2. 按 air_intruder_mission.yaml 的 spawn_mode 选择 iris_2 出生角
#      (random=随机会话种子/seed 可复现, fixed=CORNER_0/CORNER_1)
#   3. 用选中角的 iris2_x/y/yaw 启动 multi_vehicle.launch
#   4. 打印所选角落供 air_intruder_pursuit 核对（rosparam /air_intruder/spawn_corner 已由 launch 设置）
# =============================================================================
source /home/dev/car3_env.sh
set -u

WS=/home/dev/XTDrone-single-car/workspace
YAML=$WS/swarm_defense_ws/src/car3_swarm/config/air_intruder_mission.yaml
LAUNCH="px4 multi_vehicle.launch"
LOG=/tmp/air_intruder_sim.log
export DISPLAY=${DISPLAY:-:1}

SPAWN_MODE=fixed
CORNER="CORNER_2"
# EGO 最大速度/加速度：
#   --uav-speed/--uav-acc        作用于 iris_0/iris_1（防御 UAV）
#   --intruder-speed/--intruder-acc  作用于 iris_2（入侵 UAV）
# 默认 0.75/0.8 = 当前生效配置，不传参时与既有 Voronoi 方案行为完全一致
UAV_SPEED=0.75
UAV_ACC=0.8
INTRUDER_SPEED=0.75
INTRUDER_ACC=0.8
GUI=1

usage() { echo "用法: $0 [--spawn-mode random|fixed] [--corner CORNER_0|CORNER_1|CORNER_2] [--uav-speed <m/s>] [--uav-acc <m/s2>] [--intruder-speed <m/s>] [--intruder-acc <m/s2>] [--gui|--headless]"; exit 1; }
while [ $# -gt 0 ]; do
    case "$1" in
        --spawn-mode) SPAWN_MODE="$2"; shift 2 ;;
        --corner) CORNER="$2"; shift 2 ;;
        --uav-speed) UAV_SPEED="$2"; shift 2 ;;
        --uav-acc) UAV_ACC="$2"; shift 2 ;;
        --intruder-speed) INTRUDER_SPEED="$2"; shift 2 ;;
        --intruder-acc) INTRUDER_ACC="$2"; shift 2 ;;
        --gui) GUI=1; shift ;;
        --headless) GUI=0; shift ;;
        *) usage ;;
    esac
done

# ---- 选择出生角 ----
# 注意：read 只读第一行会丢失 X/Y/YAW，须用 mapfile 读全部 4 行
mapfile -t SPAWN_LINES < <(python3 \
    "$WS/swarm_defense_ws/src/car3_swarm/scripts/pick_spawn_corner.py" \
    --yaml "$YAML" --spawn-mode "$SPAWN_MODE" ${CORNER:+--corner "$CORNER"} --yaw)
CORNER_NAME=${SPAWN_LINES[0]:-}
IRIS2_X=${SPAWN_LINES[1]:-}
IRIS2_Y=${SPAWN_LINES[2]:-}
IRIS2_YAW=${SPAWN_LINES[3]:-}
if [ -z "$CORNER_NAME" ]; then
    echo "选角失败：pick_spawn_corner.py 无输出"; exit 1
fi

say() { printf '\n\033[1;36m[I] %s\033[0m\n' "$*"; }
say "iris_2 出生角 = $CORNER_NAME @ ($IRIS2_X, $IRIS2_Y) yaw $IRIS2_YAW"

# ---- 清理残留仿真 ----
# 先优雅 TERM（让 gzserver/px4/mavros 正常释放端口与状态），残留才升级 SIGKILL。
# 注意：对 gzserver/gzclient 直接 kill -9 会污染 gazebo_ros 桥——2026-08-26 实测
# 多次 -9 后新起的 gzserver 物理循环空转(高 CPU)但 /clock、/gazebo/*、mavros 全部静默，
# 表现为桨叶低转速、起飞不了。若容器整栈已僵死，直接 `docker restart xtdrone-dev-gpu`。
say "清理残留仿真进程（TERM->KILL 两级，尽量少强杀 gzserver）..."
pkill -TERM -f 'roslaunch px4 multi_vehicle.launch' 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true
pkill -TERM -x gzclient 2>/dev/null || true
for p in px4 mavros_node traj_server; do
    pkill -TERM -x "$p" 2>/dev/null || true
done
# ego_planner_node 进程名 15 字符超 comm 截断(显示 ego_planner_nod)，-x 匹配不到，须用 -f 匹配命令行
pkill -TERM -f 'ego_planner_node' 2>/dev/null || true
# rosmaster 是 python3 脚本，进程名是 python3，-x 匹配不到；-f 匹配命令行，
# 但本脚本经 bash <path> 调用（命令行不含字面量），不会自匹配。
pkill -TERM -f 'rosmaster --core' 2>/dev/null || true
sleep 5
# 剩余未退出的才强杀（卡住的 gzserver / 孤儿 PX4/MAVROS 占用 SITL UDP 14540-14542）。
for p in gzserver gzclient px4 mavros_node traj_server; do
    pkill -9 -x "$p" 2>/dev/null || true
done
pkill -9 -f 'ego_planner_node' 2>/dev/null || true
pkill -9 -f 'rosmaster --core' 2>/dev/null || true
sleep 2
rm -f /tmp/px4-sock-*

# ---- 启动 ----
say "启动 multi_vehicle.launch（GUI=$GUI）..."
GUI_ARG="gui:=false"
[ "$GUI" = 1 ] && GUI_ARG="gui:=true"
nohup roslaunch $LAUNCH $GUI_ARG start_car_nav:=true start_ego:=true \
    uav_pursuit_max_vel:=$UAV_SPEED uav_pursuit_max_acc:=$UAV_ACC \
    uav_intruder_max_vel:=$INTRUDER_SPEED uav_intruder_max_acc:=$INTRUDER_ACC \
    iris2_x:=$IRIS2_X iris2_y:=$IRIS2_Y iris2_yaw:=$IRIS2_YAW \
    iris2_corner:=$CORNER_NAME >"$LOG" 2>&1 &

echo "$CORNER_NAME" > /tmp/air_intruder_spawn_corner.txt
say "已启动（PID $!），日志 $LOG"
say "防御 UAV(iris_0/1) EGO max_vel=$UAV_SPEED acc=$UAV_ACC；入侵 iris_2 max_vel=$INTRUDER_SPEED acc=$INTRUDER_ACC"
say "iris_2 出生角写入 /tmp/air_intruder_spawn_corner.txt"
say "继续: get_local_pose.py iris 3 -> ego_swarm_transfer.py iris 3 -> uav_offboard_takeoff.py --altitude 4.0 --timeout 180"
say "然后: roslaunch car3_swarm air_intruder_pursuit.launch"
