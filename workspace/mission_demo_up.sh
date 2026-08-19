#!/usr/bin/env bash
# =============================================================================
# 一体化任务单门入侵演示脚本：上门（UP）
#
# 用法（宿主终端，一条命令）:
#   docker exec -it xtdrone-dev bash /workspace/mission_demo_up.sh
#
# 功能:
#   1. 清理残留仿真 -> 启动 multi_car3_mission.launch（GUI 直连宿主 X :1）
#   2. 入侵车固定从上门入侵（entry_mode=fixed, entry_gate=UP）
#   3. 自动调用 /mission/start，演示完整闭环:
#      巡检 -> 入侵 -> 围捕 -> 捕获 -> FINAL_ALIGN -> SUCCESS
#   4. 单轮单门: 到达终态后停止仿真并退出（无复位、无多轮循环）
#
# 可选参数: 第 1 个参数传 false 可无界面运行（headless 测试）:
#   docker exec xtdrone-dev bash /workspace/mission_demo_up.sh false
# =============================================================================
# 先加载 ROS 环境（profile.d 脚本不兼容 set -u，必须在其之前 source）
source /home/dev/car3_env.sh
set -u

GATE=UP
GATE_CN=上门
LAUNCH="car3_swarm multi_car3_mission.launch"
LOG=/tmp/mission_demo_up.log
GUI=${1:-true}
export DISPLAY=${DISPLAY:-:1}

say() { printf '\n\033[1;36m[D] %s\033[0m\n' "$*"; }
warn() { printf '\n\033[1;33m[W] %s\033[0m\n' "$*"; }
err() { printf '\n\033[1;31m[E] %s\033[0m\n' "$*"; }

# 当前任务状态（/mission/state 为 latch 发布，取 -n 1 即时返回）
mstate() {
    timeout 5 rostopic echo -n 1 /mission/state 2>/dev/null \
        | grep -o 'M_[A-Z_]*' | head -1
}

simt() {
    timeout 5 rostopic echo -n 1 /clock 2>/dev/null \
        | grep -o 'secs: [0-9]*' | grep -o '[0-9]*$' | head -1
}

cleanup() {
    say "停止仿真..."
    [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null
    sleep 2
    pkill -f 'gzserver' 2>/dev/null || true
    pkill -f 'gzclient' 2>/dev/null || true
    pkill -f 'rosmaster' 2>/dev/null || true
}
trap cleanup EXIT

# ---------------------------------------------------------------- 1. 清理 + 启动
say "清理残留仿真进程..."
pkill -f 'multi_car3_mission' 2>/dev/null || true
sleep 3
pkill -f 'gzserver' 2>/dev/null || true
pkill -f 'gzclient' 2>/dev/null || true
sleep 2

say "启动仿真（GUI=$GUI，入侵门 = $GATE_CN/$GATE）..."
nohup roslaunch $LAUNCH gui:=$GUI entry_mode:=fixed entry_gate:=$GATE \
    >"$LOG" 2>&1 &
LAUNCH_PID=$!

# ---------------------------------------------------------------- 2. 等待就绪
say "等待仿真就绪（Gazebo 世界 + 三车 + 任务服务，约 30~90s）..."
READY=0
for _ in $(seq 1 150); do
    if timeout 5 rosservice list 2>/dev/null | grep -q '/mission/start' && \
       timeout 5 rostopic echo -n 1 /car1/scan 2>/dev/null | grep -q 'ranges'; then
        READY=1
        break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        err "roslaunch 已退出，日志 $LOG:"
        tail -20 "$LOG"
        exit 1
    fi
    sleep 2
done
if [ "$READY" != 1 ]; then
    err "就绪等待超时，日志 $LOG:"
    tail -20 "$LOG"
    exit 1
fi
say "仿真就绪，等待车辆落稳..."
sleep 12
if command -v xdotool >/dev/null 2>&1; then
    xdotool search --name "Gazebo" windowmove 0 0 windowsize 2200 1500 \
        >/dev/null 2>&1 || true
fi

# ---------------------------------------------------------------- 3. 单轮演示
say "========== 单轮演示开始（$GATE_CN/$GATE 入侵）=========="
rosservice call /mission/start '{}' >/dev/null 2>&1
t0=$(simt)
PREV=""
RC=2
DEADLINE=$(( $(date +%s) + 1200 ))
while true; do
    st=$(mstate)
    if [ -n "$st" ] && [ "$st" != "$PREV" ]; then
        say "任务状态 -> $st"
        PREV="$st"
    fi
    case "$st" in
        M_SUCCESS)
            echo
            say "演示成功: 巡检 -> 入侵 -> 封控 -> 捕获 -> 最终对齐 -> SUCCESS"
            RC=0
            break ;;
        M_FAILED_ESCAPE)
            echo
            warn "入侵车从合法门逃逸 (FAILED_ESCAPE，己方未追出内墙)"
            RC=1
            break ;;
        M_INVALID_ESCAPE)
            echo
            warn "入侵车从原入口门逃逸 (INVALID_ESCAPE)"
            RC=1
            break ;;
    esac
    if [ "$(date +%s)" -ge "$DEADLINE" ]; then
        err "等待终态超时（1200s），日志 $LOG 末尾:"
        tail -20 "$LOG"
        RC=2
        break
    fi
    sleep 1
done
t1=$(simt)
res=$(timeout 5 rostopic echo -n 1 /mission/result 2>/dev/null \
    | grep -o '"[A-Z_]*"' | head -1)
say "本轮用时 ~$(( ${t1:-0} - ${t0:-0} ))s (sim 时间)，result=$res"
exit $RC
