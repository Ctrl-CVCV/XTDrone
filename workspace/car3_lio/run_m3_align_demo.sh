#!/usr/bin/env bash
# M3 多 agent SWARM-LIO 世界对齐演示(地面: car0 + car1, 均 mid360_lio)。
# 1) nesting_room 世界 + 两辆车; 2) 各自起 swarm_lio; 3) map_alignment
#    用 Gazebo 真值冻结 car1/world -> car0/world; 4) m3_verify 量化漂移/一致性。
set -e
cd /home/dev/XTDrone-single-car/workspace/car3_lio
bash clean_sim.sh || true
sleep 2
source ./env.sh
LOG=/tmp/car3_lio
mkdir -p "$LOG"

echo "== bringup dual-car =="
nohup roslaunch car3_swarm dual_lio_car_bringup.launch gui:=true >"$LOG/dual_bringup.log" 2>&1 &
sleep 20
for C in car0 car1; do
  rostopic type /$C/livox/lidar >/dev/null 2>&1 && echo "/$C/livox/lidar OK" || echo "/$C/livox/lidar MISSING"
done

echo "== start car0/car1 swarm_lio =="
for C in car0 car1; do
  nohup roslaunch swarm_lio livox_mid360.launch \
    node_name:=laserMapping_$C lid_topic:=/$C/livox/lidar imu_topic:=/$C/imu \
    drone_id:=0 output_prefix:=$C actual_uav_num:=1 \
    config_file:=$PWD/$C\_sim.yaml rviz:=false \
    vision_pose_topic:=/$C/lidar_slam/vision_pose_raw \
    quadstate_tx_topic:=/$C/quadstate_to_teammate \
    quadstate_rx_topic:=/$C/quadstate_from_teammate \
    global_extrinsic_tx_topic:=/$C/global_extrinsic_to_teammate \
    global_extrinsic_rx_topic:=/$C/global_extrinsic_from_teammate \
    >"$LOG/$C\_lio.log" 2>&1 &
done
for C in car0 car1; do
  for i in $(seq 1 60); do
    if rostopic type /$C/lidar_slam/odom >/dev/null 2>&1; then echo "$C odom up (${i}s)"; break; fi
    sleep 1
  done
done
sleep 3

echo "== align car1/world -> car0/world (gazebo truth) =="
nohup rosrun car3_swarm map_alignment.py _source:=simulation_truth \
  _parent_frame:=car0/world _child_frame:=car1/world \
  _model0:=car0 _model1:=car1 \
  _odom0_topic:=/car0/lidar_slam/odom _odom1_topic:=/car1/lidar_slam/odom \
  "_imu_offset0:=-0.07125 -0.00161 0.0806" "_imu_offset1:=-0.07125 -0.00161 0.0806" \
  _samples:=30 >"$LOG/align.log" 2>&1 &
sleep 10
timeout 5 rostopic echo -n1 /map_alignment/transform 2>/dev/null | sed -n '1,16p' || echo "align TF not up"

echo "== verify =="
timeout 40 python3 -u "$PWD/m3_verify.py" 2>&1 | grep -vE "^\[|WARNING" | tail -6
