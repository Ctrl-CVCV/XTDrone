#!/usr/bin/env bash
# M2: 假定 bringup(gazebo+car0) 已在跑, 只启动单机 swarm_lio(car0) 并验证 odom
source /home/dev/XTDrone-single-car/workspace/car3_lio/env.sh
LOG=/tmp/car3_lio
pkill -f "laserMappin[g]car0" 2>/dev/null
pkill -f "swarm_li[o]" 2>/dev/null
sleep 1
CFG=/home/dev/XTDrone-single-car/workspace/car3_lio/car0_sim.yaml
nohup roslaunch swarm_lio livox_mid360.launch \
    node_name:=laserMapping_car0 \
    lid_topic:=/car0/livox/lidar \
    imu_topic:=/car0/imu \
    drone_id:=0 \
    output_prefix:=car0 \
    actual_uav_num:=1 \
    config_file:="$CFG" \
    rviz:=false \
    vision_pose_topic:=/car0/lidar_slam/vision_pose_raw \
    quadstate_tx_topic:=/car0/quadstate_to_teammate \
    quadstate_rx_topic:=/car0/quadstate_from_teammate \
    global_extrinsic_tx_topic:=/car0/global_extrinsic_to_teammate \
    global_extrinsic_rx_topic:=/car0/global_extrinsic_from_teammate \
    >"$LOG/lio.log" 2>&1 &
echo "lio pid $!"
echo "waiting for /car0/lidar_slam/odom (up to 90s)..."
for i in $(seq 1 90); do
  if rostopic type /car0/lidar_slam/odom >/dev/null 2>&1; then echo "odom up after ${i}s"; break; fi
  sleep 1
done
echo "=== odom rate ==="
timeout 8 rostopic hz /car0/lidar_slam/odom 2>&1 | sed -n '5,6p'
echo "=== cloud_registered? ==="
rostopic type /car0/cloud_registered 2>/dev/null || echo "no cloud topic"
echo "=== lio.log tail ==="
tail -30 "$LOG/lio.log"
