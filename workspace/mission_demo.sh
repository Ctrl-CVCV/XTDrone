#!/usr/bin/env bash
# =============================================================================
# 一体化任务 GUI 一键演示脚本（巡检—入侵—区域封控—捕获/逃逸 闭环）
#
# 用法（宿主终端，一条命令）:
#   docker exec -it xtdrone-dev bash /workspace/mission_demo.sh
#
# 功能:
#   1. 清理残留仿真 -> 启动 multi_car3_mission.launch（GUI 直连宿主 X :1）
#   2. 自动调用 /mission/start，演示完整闭环:
#      巡检 -> 入侵 -> 围捕 -> 捕获 -> FINAL_ALIGN -> SUCCESS
#   3. 每轮结束后提示: 回车/r = 复位重开下一轮（仿真不重启）, q = 退出
#      复位由 /mission/reset 完成: 入侵车自动经最近门驶出、双车自动回出生点，
#      下一轮从标准状态重新开始（已实测连续 3 轮通过，见一体化任务文档 §4.5）
# =============================================================================
# 先加载 ROS 环境（profile.d 脚本不兼容 set -u，必须在其之前 source）
source /home/dev/car3_env.sh
set -u

LAUNCH="car3_swarm multi_car3_mission.launch"
LOG=/tmp/mission_demo.log
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

say "启动仿真（GUI -> 宿主 X $DISPLAY）..."
nohup roslaunch $LAUNCH >"$LOG" 2>&1 &
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

# ---------------------------------------------------------------- 3. 演示循环
ROUND=0
while true; do
    ROUND=$((ROUND + 1))
    echo
    say "========== 第 $ROUND 轮演示开始 =========="
    rosservice call /mission/start '{}' >/dev/null 2>&1
    t0=$(simt)
    PREV=""
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
                break ;;
            M_FAILED_ESCAPE)
                echo
                warn "入侵车从合法门逃逸 (FAILED_ESCAPE，己方未追出内墙)"
                break ;;
            M_INVALID_ESCAPE)
                echo
                warn "入侵车从原入口门逃逸 (INVALID_ESCAPE)"
                break ;;
        esac
        sleep 1
    done
    t1=$(simt)
    res=$(timeout 5 rostopic echo -n 1 /mission/result 2>/dev/null \
        | grep -o '"[A-Z_]*"' | head -1)
    say "本轮用时 ~$(( ${t1:-0} - ${t0:-0} ))s (sim 时间)，result=$res"

    if [ ! -t 0 ]; then
        warn "非交互终端（无 TTY），仅演示一轮"
        break
    fi
    printf '\n\033[1;33m[回车 或 r] 复位重开下一轮（仿真不重启）   [q] 退出\033[0m\n> '
    read -r ANS || break
    case "${ANS:-r}" in
        q|Q|quit|exit)
            break ;;
        *)
            say "复位中（/mission/reset：入侵车驶出门外 + 双车回出生点）..."
            rosservice call /mission/reset '{}' >/dev/null 2>&1
            for _ in $(seq 1 30); do
                [ "$(mstate)" = "M_IDLE" ] && break
                sleep 1
            done
            say "复位完成，准备下一轮..."
            sleep 2
            ;;
    esac
done

say "演示结束，共 $ROUND 轮"
exit 0
