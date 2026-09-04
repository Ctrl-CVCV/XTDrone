#!/usr/bin/env bash
# 杀掉容器内全部 ROS/gazebo/本车 LIO 仿真进程(脚本文件运行, argv 无目标字样, 不自杀)
pkill -f roslaunch  2>/dev/null
pkill -f rosmaster  2>/dev/null
pkill -f rosout     2>/dev/null
pkill -f gzserver   2>/dev/null
pkill -f gzclient   2>/dev/null
pkill -f swarm_lio  2>/dev/null
pkill -f laserMapping 2>/dev/null
sleep 2
echo cleaned
ps -eo pid,comm | grep -E 'ros|gz' | grep -v grep || echo "no ros/gz left"
