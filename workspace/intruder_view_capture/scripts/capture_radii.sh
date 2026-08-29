#!/usr/bin/env bash
# Batch capture for the 13 remaining sphere radii (r_100 already done).
# Partition: 100..250 -> 32 pts, 300..500 -> 64 pts, 550..750 -> 128 pts.
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/dev/catkin_ws/devel/setup.bash
SCRIPT=/home/dev/XTDrone-single-car/workspace/intruder_view_capture/scripts/capture_views.py
LOG=/home/dev/XTDrone-single-car/workspace/intruder_view_capture/radii_run.log

for spec in \
    "150 1.5 32" "200 2.0 32" "250 2.5 32" \
    "300 3.0 64" "350 3.5 64" "400 4.0 64" "450 4.5 64" "500 5.0 64" \
    "550 5.5 128" "600 6.0 128" "650 6.5 128" "700 7.0 128" "750 7.5 128"; do
    set -- $spec
    cm=$1; m=$2; n=$3
    echo "=== r_$cm radius=${m}m num=$n ===" >> "$LOG"
    python3 "$SCRIPT" --radius "$m" --num "$n" --subdir "r_$cm" >> "$LOG" 2>&1
    rc=$?
    echo "=== r_$cm exit=$rc ===" >> "$LOG"
done
echo "ALL DONE" >> "$LOG"
