#!/usr/bin/env bash
# M1 传感器冒烟: 只起 gazebo + car0(mid360_lio), 验证 /car0/livox/lidar(CustomMsg) 与 /car0/imu
source /home/dev/XTDrone-single-car/workspace/car3_lio/env.sh
rm -f /tmp/car3_lio/bringup.log
nohup roslaunch car3_swarm lio_car_bringup.launch gui:=true >/tmp/car3_lio/bringup.log 2>&1 &
echo "bringup pid $!"
sleep 25
echo "=== gzserver alive? ==="
pgrep -c gzserver || echo "gzserver DEAD (crashed?)"
echo "=== model car0 in states? ==="
rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -c "name: \"car0\"" || true
echo "=== topics ==="
rostopic type /car0/livox/lidar 2>/dev/null && echo "  ^ lidar OK" || echo "lidar topic MISSING"
rostopic type /car0/imu 2>/dev/null && echo "  ^ imu OK" || echo "imu topic MISSING"
echo "=== bringup.log tail (crash check) ==="
grep -nE "Segmentation|core dumped|cannot convert|ros topic name|Spawn status|Spawning model" /tmp/car3_lio/bringup.log | tail -20
