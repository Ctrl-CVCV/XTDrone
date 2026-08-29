#!/usr/bin/env bash
# =============================================================================
# 两机两车围捕一机（V 型封控） - 一键仿真脚本
#
# 复用 run_air_intruder_sim.sh 的整条仿真启动流程（世界/桥/起飞一致），
# 仅第 6/7 步改为启动 air_vblock_capture 的 vblock_capture 节点（独立方案）。
#
# 用法（宿主终端，容器内执行）:
#   docker exec -it xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/run_vblock_capture_sim.sh
#
# 可选参数:
#   --corner CORNER_0|CORNER_1|CORNER_2  iris_2 出生角（默认 CORNER_2=右下角东南角停机坪）
#   --entry DOWN|LEFT|...         入侵方向（默认 DOWN，与 CORNER_2 直连方向匹配）
#   --uav-speed <m/s>             防御 UAV(iris_0/1) EGO 最大速度（默认 1.6，需足够快在防墙边收拢前截住）
#   --uav-acc <m/s2>              防御 UAV EGO 最大加速度（默认 0.8，防小房内启动过猛）
#   --intruder-speed <m/s>        入侵 UAV(iris_2) EGO 最大速度（默认 0.5，略慢于防御 UAV）
#   --intruder-acc <m/s2>         入侵 UAV EGO 最大加速度（默认 0.8）
#   --alt <高度>                   起飞高度（默认 1.33）
#   --headless                     不弹 GUI
# =============================================================================
source /home/dev/car3_env.sh
set -u

WS=/home/dev/XTDrone-single-car/workspace
CORNER="CORNER_2"
ENTRY="DOWN"
ALT=1.33
UAV_SPEED=1.6
UAV_ACC=0.8
INTRUDER_SPEED=0.5
INTRUDER_ACC=0.8
GUI=1

usage() { echo "用法: $0 [--corner CORNER_0|CORNER_1|CORNER_2] [--entry UP|DOWN|LEFT|RIGHT] [--uav-speed <m/s>] [--uav-acc <m/s2>] [--intruder-speed <m/s>] [--intruder-acc <m/s2>] [--alt 高度] [--headless]"; exit 1; }
while [ $# -gt 0 ]; do
    case "$1" in
        --corner) CORNER="$2"; shift 2 ;;
        --entry) ENTRY="$2"; shift 2 ;;
        --uav-speed) UAV_SPEED="$2"; shift 2 ;;
        --uav-acc) UAV_ACC="$2"; shift 2 ;;
        --intruder-speed) INTRUDER_SPEED="$2"; shift 2 ;;
        --intruder-acc) INTRUDER_ACC="$2"; shift 2 ;;
        --alt) ALT="$2"; shift 2 ;;
        --headless) GUI=0; shift ;;
        *) usage ;;
    esac
done

say() { printf '\n\033[1;36m[RUN] %s\033[0m\n' "$*"; }

# ---- 0. 清理上一次运行遗留的 vblock / 起飞进程（防多代堆叠）----
# 旧的 vblock_capture 节点会继续向 iris 发围捕目标，与新运行抢控制权；
# 旧的 takeoff 脚本会一直挂着。这里在 launch 之前就清掉。
# 注意 [x] 括号防 pkill 自匹配（脚本路径本身含 vblock_capture）。
say "清理遗留 vblock_capture / takeoff 进程..."
pkill -9 -f '[v]block_capture.launch' 2>/dev/null || true
pkill -9 -f '[v]block_capture_node' 2>/dev/null || true
pkill -9 -f '[u]av_offboard_takeoff' 2>/dev/null || true
sleep 2

# ---- 1. 清理残留仿真并启动（复用 launch 脚本）----
say "清理残留仿真并启动 multi_vehicle.launch（iris_2 出生角=$CORNER, 防御UAV速度=$UAV_SPEED）..."
GUI_ARG="--gui"; [ "$GUI" = 0 ] && GUI_ARG="--headless"
bash "$WS/launch_air_intruder_sim.sh" --corner "$CORNER" --uav-speed "$UAV_SPEED" --uav-acc "$UAV_ACC" --intruder-speed "$INTRUDER_SPEED" --intruder-acc "$INTRUDER_ACC" $GUI_ARG

# ---- 2. 等待 gazebo 世界就绪 ----
say "等待 gazebo 世界加载..."
until timeout -k 2 3 rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do sleep 2; done
until timeout -k 2 3 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q "iris_0"; do sleep 2; done
echo "世界就绪"

# ---- 3. 启动 XTDrone 支持栈 + 3 通信桥 ----
say "启动 get_local_pose / ego_swarm_transfer / 3 通信桥..."
cd "$WS"
pkill -9 -f "[m]ultirotor_communication.py" 2>/dev/null || true
pkill -9 -f "[g]et_local_pose.py" 2>/dev/null || true
pkill -9 -f "[e]go_swarm_transfer.py" 2>/dev/null || true
sleep 1
nohup python3 /home/dev/XTDrone/sensing/pose_ground_truth/get_local_pose.py iris 3 >/tmp/getlocal.log 2>&1 &
echo "get_local_pose PID $!"
sleep 2
nohup python3 /home/dev/XTDrone/motion_planning/3d/ego_swarm_transfer.py iris 3 >/tmp/egotransfer.log 2>&1 &
echo "ego_swarm_transfer PID $!"
sleep 2
for id in 0 1 2; do
    nohup python3 /home/dev/XTDrone/communication/multirotor_communication.py iris "$id" >/tmp/bridge$id.log 2>&1 &
    echo "bridge iris_$id PID $!"
done

# ---- 4. 等待 PX4/mavros 就绪（最长 ~3 分钟）----
say "等待 PX4/mavros 连接..."
READY=0
for i in $(seq 1 60); do
    c=$(timeout -k 2 3 rostopic echo -n1 /iris_0/mavros/state 2>/dev/null | grep -m1 "connected:" | awk '{print $2}')
    v=$(timeout -k 2 3 rostopic echo -n1 /iris_0/mavros/vision_pose/pose 2>/dev/null | grep -c "seq:")
    if [ "$c" = "True" ] && [ "$v" -ge 1 ]; then READY=1; break; fi
    sleep 3
done
if [ "$READY" != 1 ]; then
    echo "错误：mavros 就绪超时。请检查 PX4/通信桥是否正常"
    exit 1
fi
echo "mavros 就绪"

# ---- 5. 三机起飞到 $ALT m ----
say "三机起飞到 ${ALT}m..."
nohup python3 "$WS/swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff.py" \
    --altitude "$ALT" --timeout 120 --no-start-bridge >/tmp/takeoff.log 2>&1 &
echo "takeoff PID $!"
for i in $(seq 1 90); do
    MS=$(timeout -k 2 4 rostopic echo -n1 /gazebo/model_states 2>/dev/null)
    Z=$(echo "$MS" | python3 -c '
import yaml, sys
txt = sys.stdin.read().split("---")[0]
d = yaml.safe_load(txt)
p = dict(zip(d["name"], d["pose"]))
zs = [p[k]["position"]["z"] for k in ("iris_0", "iris_1", "iris_2") if k in p]
print(" ".join("%.2f" % z for z in zs) if zs else "")
' 2>/dev/null)
    if [ -n "$Z" ]; then
        OK=$(echo "$Z" | awk '{n=split($0,a," ");ok=1;for(j=1;j<=n;j++) if(a[j]<1.2) ok=0; print ok}')
        if [ "$OK" = "1" ]; then echo "起飞完成 z=($Z)"; break; fi
    fi
    sleep 4
done

# ---- 6. 启动 vblock_capture 围捕节点（独立方案）----
say "启动 vblock_capture（corner=$CORNER entry=$ENTRY）..."
nohup roslaunch air_vblock_capture vblock_capture.launch \
    entry_side:="$ENTRY" >/tmp/vblock_node.log 2>&1 &
sleep 5
echo "捕获参数:"
rosparam get /vblock_capture/capture 2>/dev/null

# ---- 7. 监视指引 ----
say "全部就绪！任务已自动开始。"
echo "  状态: docker exec xtdrone-dev-gpu bash -c \"source /home/dev/car3_env.sh && rostopic echo -n1 /vblock_capture/result\""
echo "  或: docker exec xtdrone-dev-gpu tail -f ~/.ros/log/latest/vblock_capture-*.log"
echo "  FSM 推进: docker exec xtdrone-dev-gpu bash -c \"grep 'state ->' ~/.ros/log/latest/vblock_capture-*.log | tail -5\""
