#!/usr/bin/env bash
# =============================================================================
# V 型封控围捕 —— 一键启动脚本
#
# 用法（宿主终端，容器内执行）:
#   docker exec -it xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/run_vblock_capture_oneclick.sh [参数]
#
# 功能:
#   1. 复用 run_vblock_capture_sim.sh 完成整套仿真启动（清理->世界->桥->mavros->起飞）
#   2. 启动 vblock_capture 围捕节点后，自动等待其到达终态
#   3. 实时打印 FSM 状态转移，结束时给出最终结果（CAPTURED / FAILED_ESCAPE）
#   4. 任意阶段按 Ctrl+C（SIGINT/SIGTERM）即结束全部仿真进程（TERM->KILL 两级清理，
#      gazebo/PX4/mavros/EGO/通信桥/vblock 节点一并关闭）
#
# 参数透传给 run_vblock_capture_sim.sh，缺省即定稿配置：
#   --corner CORNER_2 --entry DOWN --uav-speed 1.6 --uav-acc 1.6 --intruder-speed 0.5 --intruder-acc 0.8
#   （防御 UAV 1.6 m/s，入侵 iris_2 0.5 m/s；CORNER_2=右下角东南角停机坪）
# =============================================================================
source /home/dev/car3_env.sh
set -u

WS=/home/dev/XTDrone-single-car/workspace
LOG_POLL_SEC=${LOG_POLL_SEC:-360}   # 最长等待节点出终态（秒）；可通过环境变量覆盖

say() { printf '\n\033[1;36m[ONECLICK] %s\033[0m\n' "$*"; }

# ---- 结束仿真进程（Ctrl+C / 收到 TERM 时调用）----
# 与 launch_air_intruder_sim.sh 清理策略一致：先 TERM 优雅停（让 gzserver/px4/mavros
# 释放端口与状态），残留才 SIGKILL。注意各 pkill 均用 [x] 括号防自匹配（本脚本路径
# 含 vblock_capture，不能用裸字面量）。
cleanup_sim() {
    echo ""
    say "收到中断信号，正在结束仿真进程..."
    pkill -TERM -f 'roslaunch px4 multi_vehicle.launch' 2>/dev/null || true
    pkill -TERM -x gzserver 2>/dev/null || true
    pkill -TERM -x gzclient 2>/dev/null || true
    for p in px4 mavros_node traj_server; do
        pkill -TERM -x "$p" 2>/dev/null || true
    done
    pkill -TERM -f 'ego_planner_node' 2>/dev/null || true
    pkill -TERM -f 'rosmaster --core' 2>/dev/null || true
    pkill -TERM -f '[v]block_capture.launch' 2>/dev/null || true
    pkill -TERM -f '[v]block_capture_node' 2>/dev/null || true
    pkill -TERM -f '[u]av_offboard_takeoff' 2>/dev/null || true
    pkill -TERM -f '[m]ultirotor_communication.py' 2>/dev/null || true
    pkill -TERM -f '[g]et_local_pose.py' 2>/dev/null || true
    pkill -TERM -f '[e]go_swarm_transfer.py' 2>/dev/null || true
    sleep 3
    pkill -9 -f 'roslaunch px4 multi_vehicle.launch' 2>/dev/null || true
    pkill -9 -x gzserver gzclient px4 mavros_node traj_server 2>/dev/null || true
    pkill -9 -f 'ego_planner_node' 2>/dev/null || true
    pkill -9 -f 'rosmaster --core' 2>/dev/null || true
    pkill -9 -f '[v]block_capture.launch' 2>/dev/null || true
    pkill -9 -f '[v]block_capture_node' 2>/dev/null || true
    pkill -9 -f '[u]av_offboard_takeoff' 2>/dev/null || true
    pkill -9 -f '[m]ultirotor_communication.py' 2>/dev/null || true
    pkill -9 -f '[g]et_local_pose.py' 2>/dev/null || true
    pkill -9 -f '[e]go_swarm_transfer.py' 2>/dev/null || true
    rm -f /tmp/px4-sock-*
    say "仿真进程已结束。"
}

# Ctrl+C（INT）与 TERM 均触发清理；正常出结果后不自动清理（可再 Ctrl+C 收尾）
trap 'cleanup_sim; exit 130' INT TERM

say "步骤 1/2：启动整套仿真（参数: $*）..."
bash "$WS/run_vblock_capture_sim.sh" "$@"
RC=$?
if [ "$RC" != 0 ]; then
    echo "错误：仿真启动失败（rc=$RC）。请检查 run_vblock_capture_sim.sh 输出。"
    exit 1
fi

# ---- 等待 vblock_capture_node 进程出现，并取其 __log 路径 ----
NODE_LOG=""
for i in $(seq 1 30); do
    NODE_LOG=$(ps -eo args 2>/dev/null \
        | grep '[v]block_capture_node' \
        | grep -o '__log:=[^ ]*' | sed 's/__log:=//' | head -1)
    if [ -n "$NODE_LOG" ] && [ -f "$NODE_LOG" ]; then
        break
    fi
    NODE_LOG=""
    sleep 2
done
if [ -z "$NODE_LOG" ]; then
    echo "错误：未找到 vblock_capture 节点日志（节点未启动？）"
    exit 1
fi

say "步骤 2/2：等待围捕终态，实时打印状态转移..."
echo "节点日志: $NODE_LOG"
echo "----------------------------------------------------------------------"

# ---- 轮询节点日志：打印新增状态行，直到出现终态 ----
SEEN=0
RESULT_LINE=""
END_EPOCH=$(( $(date +%s) + LOG_POLL_SEC ))
while [ "$(date +%s)" -lt "$END_EPOCH" ]; do
    TOTAL=$(wc -l < "$NODE_LOG" 2>/dev/null || echo 0)
    # 日志被轮转/清空时重置游标
    if [ "$TOTAL" -lt "$SEEN" ]; then
        SEEN=0
    fi
    if [ "$TOTAL" -gt "$SEEN" ]; then
        tail -n +$((SEEN + 1)) "$NODE_LOG" \
            | grep -E --line-buffered 'state ->|CAPTURED|FAILED_ESCAPE|INVALID_ESCAPE|Traceback'
        SEEN=$TOTAL
    fi
    RESULT_LINE=$(grep -E 'CAPTURED|FAILED_ESCAPE|INVALID_ESCAPE' "$NODE_LOG" 2>/dev/null | tail -1)
    if [ -n "$RESULT_LINE" ]; then
        break
    fi
    sleep 3
done

echo "----------------------------------------------------------------------"
if [ -n "$RESULT_LINE" ]; then
    SUMMARY=$(echo "$RESULT_LINE" | sed -E 's/^\[[^]]+\]\[(WARNING|INFO)\] [0-9:, -]+: //')
    echo "==================== 最终结果 ===================="
    echo "  $SUMMARY"
    echo "=================================================="
    case "$RESULT_LINE" in
        *CAPTURED*) say "捕获成功 ✓（两机围捕完成）" ;;
        *)          say "围捕未成功（FAILED_ESCAPE / INVALID_ESCAPE）" ;;
    esac
    echo "结果消息也发布在话题 /vblock_capture/result"
    echo "完整日志: $NODE_LOG"
    exit 0
else
    echo "超时：${LOG_POLL_SEC}s 内未出终态。请检查:"
    echo "  tail -f $NODE_LOG"
    exit 2
fi
