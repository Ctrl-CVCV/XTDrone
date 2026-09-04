#!/usr/bin/env bash
# 诊断: 跑镜像自带 sensors-only mid360 冒烟(test_pattern), 判断 CustomMsg 插件+csv 是否可用
source /home/dev/XTDrone-single-car/workspace/car3_lio/env.sh
pkill -f gzserver 2>/dev/null
sleep 1
rm -f /tmp/diag.log
nohup roslaunch livox_laser_simulation test_pattern.launch rviz:=false >/tmp/diag.log 2>&1 &
sleep 18
echo "=== diag.log tail ==="
tail -50 /tmp/diag.log
echo "=== topics ==="
rostopic list 2>/dev/null | grep -iE 'livox|lidar|scan' || echo none
