#!/usr/bin/env bash
# =============================================================================
# 空中入侵无人机围捕 - 一键仿真脚本
#
# 功能（自动串起整条流程，可重复使用）：
#   1. 清理残留仿真进程并启动 multi_vehicle.launch（iris_2 固定左下角 CORNER_1 出生）
#   2. 等待 gazebo 世界加载
#   3. 启动 XTDrone 支持栈：get_local_pose + ego_swarm_transfer + 3 个通信桥
#   4. 等待 PX4/mavros 连接就绪
#   5. 三架 iris 起飞到 1.33m
#   6. 启动 air_intruder_pursuit 围捕节点
#   7. 打印监视方法与结果话题
#
# 用法（宿主终端，容器内执行）:
#   docker exec -it xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/run_air_intruder_sim.sh
#
# 可选参数:
#   --corner CORNER_0|CORNER_1    iris_2 出生角（默认 CORNER_1=左下角）
#   --alt <高度>                   起飞高度（默认 1.33）
#   --headless                     不弹 GUI
#
# 进程都挂在后台，脚本跑完返回，任务自动继续。
# =============================================================================
source /home/dev/car3_env.sh
set -u

WS=/home/dev/XTDrone-single-car/workspace
CORNER="CORNER_1"
ALT=1.33
GUI=1

usage() { echo "用法: $0 [--corner CORNER_0|CORNER_1] [--alt 高度] [--headless]"; exit 1; }
while [ $# -gt 0 ]; do
    case "$1" in
        --corner) CORNER="$2"; shift 2 ;;
        --alt) ALT="$2"; shift 2 ;;
        --headless) GUI=0; shift ;;
        *) usage ;;
    esac
done

say() { printf '\n\033[1;36m[RUN] %s\033[0m\n' "$*"; }

# ---- 1. 清理残留仿真并启动（复用 launch 脚本）----
say "清理残留仿真并启动 multi_vehicle.launch（iris_2 出生角=$CORNER）..."
GUI_ARG="--gui"; [ "$GUI" = 0 ] && GUI_ARG="--headless"
bash "$WS/launch_air_intruder_sim.sh" --corner "$CORNER" $GUI_ARG

# ---- 2. 等待 gazebo 世界就绪 ----
say "等待 gazebo 世界加载..."
until timeout 3 rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do sleep 2; done
until timeout 3 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q "iris_0"; do sleep 2; done
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
    c=$(timeout 3 rostopic echo -n1 /iris_0/mavros/state 2>/dev/null | grep -m1 "connected:" | awk '{print $2}')
    v=$(timeout 3 rostopic echo -n1 /iris_0/mavros/vision_pose/pose 2>/dev/null | grep -c "seq:")
    if [ "$c" = "True" ] && [ "$v" -ge 1 ]; then READY=1; break; fi
    sleep 3
done
if [ "$READY" != 1 ]; then
    echo "错误：mavros 就绪超时。请检查 PX4/通信桥是否正常（docker exec xtdrone-dev-gpu bash -c \"source /home/dev/car3_env.sh && rostopic echo -n1 /iris_0/mavros/state\")"
    exit 1
fi
echo "mavros 就绪"

# ---- 5. 三机起飞到 $ALT m ----
say "三机起飞到 ${ALT}m..."
nohup python3 "$WS/swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff.py" \
    --altitude "$ALT" --timeout 120 --no-start-bridge >/tmp/takeoff.log 2>&1 &
echo "takeoff PID $!"
for i in $(seq 1 90); do
    MS=$(timeout 4 rostopic echo -n1 /gazebo/model_states 2>/dev/null)
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

# ---- 6. 启动围捕节点 ----
say "启动 air_intruder_pursuit..."
nohup roslaunch car3_swarm air_intruder_pursuit.launch >/tmp/air_intruder_node.log 2>&1 &
sleep 5
echo "捕获参数:"
rosparam get /air_intruder_pursuit/capture 2>/dev/null

# ---- 7. 监视指引 ----
say "全部就绪！任务已自动开始。"
echo "  状态: docker exec xtdrone-dev-gpu bash -c \"source /home/dev/car3_env.sh && rostopic echo -n1 /air_intruder_pursuit/result\""
echo "  或: docker exec xtdrone-dev-gpu tail -f ~/.ros/log/latest/air_intruder_pursuit-*.log"
echo "  FSM 推进: docker exec xtdrone-dev-gpu bash -c \"grep 'state ->' ~/.ros/log/latest/air_intruder_pursuit-*.log | tail -5\""
