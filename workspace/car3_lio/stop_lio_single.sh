#!/usr/bin/env bash
# 停止单车 LIO 冒烟(按 pid 文件杀, 避免 pkill 自杀)
LOG=/tmp/car3_lio
for f in "$LOG"/lio.pid "$LOG"/bringup.pid; do
  [ -e "$f" ] && kill "$(cat "$f")" 2>/dev/null
done
sleep 2
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f "laserMapping_car0" 2>/dev/null
echo "[stop] done; residual: $(pgrep -f 'roslaunch|gzserver|swarm_lio' | tr '\n' ' ')"
