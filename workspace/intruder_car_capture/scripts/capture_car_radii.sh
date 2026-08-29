#!/usr/bin/env bash
# 小车视角采集 - 阶段2: 半球批量采集。
# r_50-250cm -> 16 点; 300-500cm -> 32 点; 550-750cm -> 64 点 (半球=球形一半密度)。
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/dev/catkin_ws/devel/setup.bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash

SCRIPT=/home/dev/XTDrone-single-car/workspace/intruder_car_capture/scripts/capture_car_views.py
LOG=/home/dev/XTDrone-single-car/workspace/intruder_car_capture/radii_run.log
rm -f "$LOG"

for spec in \
    "50 0.5 16" "100 1.0 16" "150 1.5 16" "200 2.0 16" "250 2.5 16" \
    "300 3.0 32" "350 3.5 32" "400 4.0 32" "450 4.5 32" "500 5.0 32" \
    "550 5.5 64" "600 6.0 64" "650 6.5 64" "700 7.0 64" "750 7.5 64"; do
    set -- $spec
    cm=$1; m=$2; n=$3
    echo "=== r_$cm radius=${m}m num=$n ===" >> "$LOG"
    python3 "$SCRIPT" --radius "$m" --num "$n" --subdir "r_$cm" >> "$LOG" 2>&1
    rc=$?
    echo "=== r_$cm exit=$rc ===" >> "$LOG"
done
echo "ALL DONE" >> "$LOG"
