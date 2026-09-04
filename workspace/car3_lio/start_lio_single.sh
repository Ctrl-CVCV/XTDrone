#!/usr/bin/env bash
# 单车 SWARM-LIO 冒烟启动: gazebo(nesting_room) + car0(mid360_lio) -> swarm_lio(car0)
# 用法(容器内): bash /workspace/car3_lio/start_lio_single.sh [--nogui]
# 日志: /tmp/car3_lio/*.log ; PID 记录 /tmp/car3_lio/*.pid
export ROS_MASTER_URI=http://localhost:11311
WS=/home/dev/XTDrone-single-car/workspace/car3_lio
source "$WS/env.sh"
LOG=/tmp/car3_lio; mkdir -p "$LOG"
GUI=true
[ "${1:-}" = "--nogui" ] && GUI=false

# ---------- 清理先前残留(避免 pkill 自杀: 只按我们记录的 pid 文件杀) ----------
for f in "$LOG"/*.pid; do [ -e "$f" ] || continue; kill "$(cat "$f")" 2>/dev/null; done
sleep 2
rm -f "$LOG"/*.pid "$LOG"/*.log
pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; pkill -f "rosmaster" 2>/dev/null
# rosnode 清 roslaunch 残留
rosnode kill -a >/dev/null 2>&1; sleep 1

echo "[start] bringup gazebo+car0(mid360_lio)... gui=$GUI"
nohup roslaunch car3_swarm lio_car_bringup.launch gui:=$GUI >"$LOG/bringup.log" 2>&1 &
echo $! > "$LOG/bringup.pid"
echo "[start] bringup pid $(cat "$LOG/bringup.pid")"

# ---------- 等 /clock + gazebo 出车 ----------
echo "[wait] gzserver /clock ..."
for i in $(seq 1 60); do rostopic echo -n1 /clock >/dev/null 2>&1 && break; sleep 1; done
echo "[wait] model car0 in gazebo ..."
for i in $(seq 1 60); do
  if rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q "name: \"car0\""; then echo "[ok] car0 spawned"; break; fi
  sleep 1
done

# ---------- M1 检查: CustomMsg lidar + IMU ----------
echo "[M1] check /car0/livox/lidar (expect livox_ros_driver2/CustomMsg)..."
T=$(rostopic type /car0/livox/lidar 2>/dev/null)
echo "[M1]   lidar type = ${T:-NONE}"
echo "[M1] check /car0/imu (expect sensor_msgs/Imu)..."
T2=$(rostopic type /car0/imu 2>/dev/null)
echo "[M1]   imu type   = ${T2:-NONE}"
echo "[M1] lidar rate (5s window):"
timeout 8 rostopic hz /car0/livox/lidar 2>&1 | sed -n '5,6p' || true
echo "[M1] imu rate (5s window):"
timeout 8 rostopic hz /car0/imu 2>&1 | sed -n '5,6p' || true

# ---------- M2: 启动 swarm_lio(car0) ----------
CFG=/home/dev/XTDrone-single-car/workspace/car3_lio/car0_sim.yaml
echo "[start] swarm_lio single(car0)..."
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
echo $! > "$LOG/lio.pid"
echo "[start] lio pid $(cat "$LOG/lio.pid")"

echo "[wait] /car0/lidar_slam/odom (grace ~10s gravity align)..."
ODOM_OK=""
for i in $(seq 1 75); do
  if rostopic type /car0/lidar_slam/odom >/dev/null 2>&1; then ODOM_OK=1; echo "[ok] odom topic up after ${i}s"; break; fi
  sleep 1
done
if [ -z "$ODOM_OK" ]; then echo "[FAIL] odom not up; tail lio.log:"; tail -40 "$LOG/lio.log"; exit 1; fi

echo "[M2] odom rate (6s window):"
timeout 8 rostopic hz /car0/lidar_slam/odom 2>&1 | sed -n '5,6p' || true

echo "[done] bringup=${LOG}/bringup.log  lio=${LOG}/lio.log"
echo "       TF:  $(timeout 3 rostopic echo -n1 /tf 2>/dev/null | grep -m2 'frame_id' | head -4)"
