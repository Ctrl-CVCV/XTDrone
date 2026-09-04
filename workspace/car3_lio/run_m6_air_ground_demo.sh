#!/usr/bin/env bash
# M6: 空地同场总联演示 — iris_0/iris_1(SWARM-LIO, quad0/quad1) + 两辆围捕车 car0/car1(SWARM-LIO)
#      同一 nesting_room Gazebo; car0/world 与 car1/world 各自对齐进 quad0/world;
#      car0、car1 各自 2D 导航(可同场并存、独立发目标)。
# 分段可单独执行:  bash run_m6_air_ground_demo.sh <clean|up_world|up_uavlio|up_carlio|
#                                                  up_scan|up_align|up_nav|up_takeoff|verify|status>
# 必须在容器 xtdrone-swarm-lio 内 source 本脚本头部环境后调用(脚本自动 source)。
WS=/home/dev/XTDrone-single-car/workspace
LIVOX_WS=/home/dev/XTDrone-single-car/ws_livox
LOG=/tmp/m6logs
mkdir -p "$LOG"

setup_env() {
  source /opt/ros/noetic/setup.bash
  [ -f /home/dev/catkin_ws/devel/setup.bash ] && source /home/dev/catkin_ws/devel/setup.bash --extend
  [ -f /home/dev/ego_ws/devel/setup.bash ] && source /home/dev/ego_ws/devel/setup.bash --extend
  source "$LIVOX_WS/devel/setup.bash" --extend
  source "$WS/swarm_defense_ws/devel/setup.bash" --extend
  export DISPLAY="${HOST_DISPLAY:-:1}"
  export QT_X11_NO_MITSHM=1
  export GAZEBO_MODEL_PATH="${WS}/swarm_defense_ws/src/basic_room_sim/models:${LIVOX_WS}/src/livox_laser_simulation_Mid360/models:${GAZEBO_MODEL_PATH:-}"
  export GAZEBO_RESOURCE_PATH="${WS}/swarm_defense_ws/src/basic_room_sim:/usr/share/gazebo-11"
  export GAZEBO_MODEL_DATABASE_URI=""
  export GAZEBO_PLUGIN_PATH="${LIVOX_WS}/devel/lib:${WS}/swarm_defense_ws/devel/lib:/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
  export ROS_PACKAGE_PATH="${WS}/swarm_defense_ws/src:${LIVOX_WS}/src:${ROS_PACKAGE_PATH}"
  export LD_LIBRARY_PATH="${LIVOX_WS}/devel/lib:${WS}/swarm_defense_ws/devel/lib:${LD_LIBRARY_PATH}"
  export PYTHONPATH="${LIVOX_WS}/devel/lib/python3/dist-packages:${WS}/swarm_defense_ws/devel/lib/python3/dist-packages:${PYTHONPATH}"
}
setup_env

say() { printf '\n\033[1;36m[M6/%s] %s\033[0m\n' "$1" "$2"; }
wait_topic() { for i in $(seq 1 "${2:-90}"); do timeout 2 rostopic type "$1" >/dev/null 2>&1 && { echo "ok $1 (${i}s)"; return 0; }; sleep 1; done; echo "TIMEOUT waiting $1"; return 1; }
wait_model() { for i in $(seq 1 "${2:-60}"); do timeout 2 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q "$1" && { echo "ok model $1 (${i}s)"; return 0; }; sleep 2; done; echo "TIMEOUT waiting model $1"; return 1; }

CAR0_YAW="-2.334"

clean() {
  say clean "关闭旧 M1-M5 场景进程(保留 rosmaster)"
  pkill -TERM -f 'car[01]_nav.launch' 2>/dev/null
  pkill -TERM -f 'dual_lio_car_bringup.launch' 2>/dev/null
  sleep 2
  pkill -TERM -f 'laserMapping_car[0-9]' 2>/dev/null
  pkill -TERM -f 'map_alignment.py' 2>/dev/null
  pkill -TERM -f 'mid360_to_scan.py' 2>/dev/null
  pkill -TERM -f 'build_scan_2d_map' 2>/dev/null
  pkill -TERM -f 'controller_manager/spawner' 2>/dev/null   # ros_control spawner 卡 100% 的重试风暴
  pkill -TERM -x gzserver 2>/dev/null
  pkill -TERM -x gzclient 2>/dev/null
  pkill -TERM -f 'px4_sitl_default' 2>/dev/null   # PX4 SITL 常驻 daemon(iris_0/1),否则重起场景时报 "daemon already running" 而 sitl 空跑
  sleep 5
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'controller_manager/spawner' 2>/dev/null
  pkill -9 -f 'px4_sitl_default' 2>/dev/null
  pkill -9 -f 'multi_vehicle.launch' 2>/dev/null          # 场景主 roslaunch(其上其它节点随 gazebo 死而退出,此处兜底)
  pkill -9 -f 'roslaunch car3_control car3_' 2>/dev/null
  sleep 2
  rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
  sleep 1
  echo "cleaned"
}

up_world() {
  say up_world "启动 multi_vehicle.launch(gazebo+iris_0/1 PX4+car0..3, spawn_delay 交错)"
  # multi_vehicle 内已给 car0 增加 lidar:=mid360_lio(见容器内文件 tweak)。
  nohup roslaunch /home/dev/PX4-Autopilot/launch/multi_vehicle.launch \
       gui:=true paused:=false spawn_cars:=true start_car_nav:=false \
       start_ego:=false start_uav_rviz_state:=true \
       >"$LOG/world.log" 2>&1 &
  echo "world pid $!"
  wait_model iris_0 120 || true
  wait_model car0 120 || true
  wait_model iris_1 120 || true
  echo "=== mavros state ==="
  for i in 0 1; do
    for k in $(seq 1 30); do
      c=$(timeout 2 rostopic echo -n1 /iris_$i/mavros/state 2>/dev/null | grep -m1 "connected:" | awk '{print $2}')
      [ "$c" = "True" ] && { echo "iris_$i mavros connected (${k}*2s)"; break; }
      sleep 2
    done
  done
  echo "=== world.log tail ==="; tail -15 "$LOG/world.log"
}

up_uavlio() {
  say up_uavlio "启动双机 UAV SWARM-LIO: quad0+quad1 + dual_map_alignment + pose_guard"
  pkill -f 'dual_mid360_distributed' 2>/dev/null
  sleep 1
  nohup roslaunch swarm_lio dual_mid360_distributed.launch \
       rviz:=false map_alignment:=true alignment_source:=simulation_truth alignment_samples:=30 \
       >"$LOG/uavlio.log" 2>&1 &
  echo "uavlio pid $!"
  wait_topic /quad0/lidar_slam/odom 120 || true
  wait_topic /quad1/lidar_slam/odom 120 || true
  echo "=== vision pose guard out (mavros vision) ==="
  wait_topic /iris_0/mavros/vision_pose/pose 60 || true
  echo "=== hz ==="
  timeout 6 rostopic hz /quad0/lidar_slam/odom 2>&1 | sed -n '5p'
  echo "=== uavlio.log tail ==="; tail -20 "$LOG/uavlio.log"
}

start_car_lio() {
  # $1=car(如 car0) $2=drone_id(car0=0 沿用 quad0 空位;car1=2 取不与 quad0/1 撞的 child 帧)
  local car=$1 id=$2
  nohup roslaunch swarm_lio livox_mid360.launch \
       node_name:=laserMapping_$car lid_topic:=/$car/livox/lidar imu_topic:=/$car/imu \
       drone_id:=$id output_prefix:=$car actual_uav_num:=1 \
       config_file:="$WS/car3_lio/${car}_sim.yaml" rviz:=false \
       vision_pose_topic:=/$car/lidar_slam/vision_pose_raw \
       quadstate_tx_topic:=/$car/quadstate_to_teammate \
       quadstate_rx_topic:=/$car/quadstate_from_teammate \
       global_extrinsic_tx_topic:=/$car/global_extrinsic_to_teammate \
       global_extrinsic_rx_topic:=/$car/global_extrinsic_from_teammate \
       >"$LOG/carlio_${car}.log" 2>&1 &
  echo "$car lio pid $!"
}
up_carlio() {
  say up_carlio "启动两车 SWARM-LIO: laserMapping_car0(id0) + laserMapping_car1(id2), 独立单机"
  pkill -f 'laserMapping_car[0-9]' 2>/dev/null
  sleep 1
  start_car_lio car0 0
  start_car_lio car1 2
  wait_topic /car0/lidar_slam/odom 120 || true
  wait_topic /car1/lidar_slam/odom 120 || true
  echo "=== livox present? ==="
  timeout 4 rostopic type /car0/livox/lidar 2>/dev/null || echo "NO /car0/livox/lidar"
  timeout 4 rostopic type /car1/livox/lidar 2>/dev/null || echo "NO /car1/livox/lidar"
  echo "=== carlio logs tail ==="; tail -10 "$LOG/carlio_car0.log"; tail -10 "$LOG/carlio_car1.log"
}

up_scan() {
  say up_scan "两车 mid360 CustomMsg -> /carN/scan (2D, 喂 AMCL/move_base)"
  pkill -f 'mid360_to_scan.py' 2>/dev/null
  sleep 1
  for car in car0 car1; do
    nohup python3 "$WS/swarm_defense_ws/src/car3_control/scripts/mid360_to_scan.py" \
         __name:=mid360_to_scan_$car \
         _lidar_topic:=/$car/livox/lidar _scan_topic:=/$car/scan _frame_id:=/$car/livox_link \
         >"$LOG/scan_${car}.log" 2>&1 &
    echo "$car scan pid $!"
  done
  wait_topic /car0/scan 60 || true
  wait_topic /car1/scan 60 || true
}

start_car_align() {
  # $1=car(如 car1): map_alignment 第二实例, __name 区分, 把 $car/world 表达进 quad0/world
  local car=$1
  nohup python3 "$WS/swarm_defense_ws/src/car3_swarm/scripts/map_alignment.py" \
       __name:=map_alignment_$car \
       _source:=simulation_truth _parent_frame:=quad0/world _child_frame:=$car/world \
       _model0:=iris_0 _model1:=$car \
       _odom0_topic:=/quad0/lidar_slam/odom _odom1_topic:=/$car/lidar_slam/odom \
       _imu_offset0:="0 0 0" _imu_offset1:="-0.07125 -0.00161 0.0806" \
       _samples:=30 >"$LOG/align_${car}.log" 2>&1 &
  echo "$car align pid $!"
}
up_align() {
  say up_align "对齐两车: car0/world、car1/world 均表达进 quad0/world (model0=iris_0)"
  pkill -f 'map_alignment.py' 2>/dev/null
  sleep 1
  start_car_align car0
  start_car_align car1
  sleep 12
  echo "=== align logs ==="; tail -8 "$LOG/align_car0.log"; tail -8 "$LOG/align_car1.log"
}

up_nav() {
  say up_nav "两车 2D 导航: car0(root, /move_base/*) + car1(/car1 ns, /car1/move_base/*)"
  pkill -f 'car[01]_nav.launch' 2>/dev/null
  sleep 1
  nohup roslaunch car3_control car0_nav.launch init_x:=2.3 init_y:=2.4 init_a:="$CAR0_YAW" \
       >"$LOG/nav.log" 2>&1 &
  echo "car0 nav pid $!"
  nohup roslaunch car3_control car1_nav.launch >"$LOG/nav_car1.log" 2>&1 &
  echo "car1 nav pid $!"
  wait_topic /move_base/goal 60 || true
  wait_topic /car1/move_base/goal 60 || true
  echo "=== nav logs tail ==="; tail -8 "$LOG/nav.log"; tail -8 "$LOG/nav_car1.log"
}

up_takeoff() {
  say up_takeoff "iris_0 起飞(0.7m, SWARM-LIO vision 高度, 保留悬停不交给 EGO)"
  pkill -f 'uav_offboard_takeoff_lio_ref.py' 2>/dev/null
  sleep 1
  nohup python3 "$WS/swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff_lio_ref.py" \
       --uavs iris_0 --altitude 0.7 --height-source vision --no-ego-handover \
       --hold-seconds 120 --timeout 90 --no-start-bridge \
       >"$LOG/takeoff.log" 2>&1 &
  echo "takeoff pid $!"
  for i in $(seq 1 45); do
    Z=$(timeout 3 rostopic echo -n1 /gazebo/model_states 2>/dev/null | python3 -c '
import sys
try:
    import yaml
    d=yaml.safe_load(sys.stdin.read().split("---")[0])
    p=dict(zip(d["name"],d["pose"]))
    print("%.3f"%p["iris_0"]["position"]["z"] if "iris_0" in p else "")
except Exception: print("")
' 2>/dev/null)
    [ -n "$Z" ] && echo "iris_0 z=$Z"
    [ -n "$Z" ] && python3 -c "exit(0 if float('$Z')>=0.5 else 1)" 2>/dev/null && { echo "iris_0 airborne!"; break; }
    sleep 4
  done
  echo "=== takeoff.log tail ==="; tail -25 "$LOG/takeoff.log"
}

verify() {
  say verify "共享 LIO 世界一致性数值核验(双车)"
  # 1) gazebo 真值: iris_0 与 car0/car1 的相对位姿(参考真值)
  MS=$(timeout 4 rostopic echo -n1 /gazebo/model_states 2>/dev/null | python3 -c '
import sys,yaml,numpy as np
d=yaml.safe_load(sys.stdin.read().split("---")[0])
nm=d["name"]; ps={k:np.array([v["position"]["x"],v["position"]["y"],v["position"]["z"]]) for k,v in zip(nm,d["pose"])}
a=ps.get("iris_0"); out=[]
for k in ("car0","car1"):
  b=ps.get(k)
  out.append("%s_gaz=%6.3f %6.3f %5.3f rel_iris0=%.3f"%(k,b[0],b[1],b[2],float(np.hypot(b[0]-a[0],b[1]-a[1]))))
print("iris0_gaz=%6.3f %6.3f %5.3f | "%(a[0],a[1],a[2])+" | ".join(out))
' 2>/dev/null)
  echo "gazebo: $MS"
  # 2) TF 链: quad0/world -> carN/base_footprint (LIO 世界表达的 carN 位姿, 与真值相对位姿比对)
  for car in car0 car1; do
    echo "tf quad0/world->${car}/base_footprint (LIO 世界):"
    timeout 5 rosrun tf tf_echo quad0/world $car/base_footprint 2>/dev/null | grep -E "^Translation|^Rotation" | head -2 || echo "tf 不可用(可能 $car 尚未有 LIO 定位)"
    echo "tf quad0/world->${car}/world (对齐 T):"
    timeout 5 rosrun tf tf_echo quad0/world $car/world 2>/dev/null | grep -E "^Translation|^Rotation" | head -2 || echo "对齐 TF 尚未广播"
  done
}

status() {
  echo "=== nodes ==="; rosnode list 2>/dev/null | grep -E "gazebo|px4|mavros|laserMapping|amcl|move_base|map_server|map_alignment|uav_rviz|ego" | head -40
  echo "=== key topics hz ==="
  for t in /quad0/lidar_slam/odom /car0/lidar_slam/odom /car1/lidar_slam/odom \
           /iris_0/mavros/state /car0/scan /car1/scan; do
    echo -n "$t: "; timeout 4 rostopic hz "$t" 2>&1 | sed -n '5p'
  done
  echo "=== model_states z ==="
  timeout 4 rostopic echo -n1 /gazebo/model_states 2>/dev/null | python3 -c '
import sys,yaml
try:
 d=yaml.safe_load(sys.stdin.read().split("---")[0]); nm=d["name"]
 for k in ("iris_0","iris_1","car0","car1"):
  if k in nm:
   i=nm.index(k); p=d["pose"][i]["position"]; print(k, "x=%.2f y=%.2f z=%.2f"%(p["x"],p["y"],p["z"]))
except Exception as e: print("parse err", e)
'
}

case "${1:-}" in
  clean) clean ;;
  up_world) up_world ;;
  up_uavlio) up_uavlio ;;
  up_carlio) up_carlio ;;
  up_scan) up_scan ;;
  up_align) up_align ;;
  up_nav) up_nav ;;
  up_takeoff) up_takeoff ;;
  verify) verify ;;
  status) status ;;
  *) echo "usage: $0 <clean|up_world|up_uavlio|up_carlio|up_scan|up_align|up_nav|up_takeoff|verify|status>"; exit 1 ;;
esac
