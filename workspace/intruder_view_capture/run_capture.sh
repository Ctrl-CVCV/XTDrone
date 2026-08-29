#!/bin/bash
# Standalone intruder-UAV viewpoint capture:
#   empty world + iris_red(iris_0) -> takeoff to 8 m hover -> 32 viewpoint photos
#   on a 0.5 m sphere around the aircraft, saved to <task>/images/view_XX.jpg.
# GUI is enabled (per project rule, no --headless).
# Non-interactive bash does not run .bashrc; source the ROS toolchain explicitly.
# ROS_MASTER_URI must be set first (catkin profile scripts reference it under set -u).
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/dev/catkin_ws/devel/setup.bash
set -u
DIR=/home/dev/XTDrone-single-car/workspace/intruder_view_capture
cd "$DIR"

echo '[1/7] 清理旧仿真进程'
# NOTE: bracket only the FIRST char ([g]azebo, not [gazebo] which is a char class
# matching any of {g,a,z,e,b,o} and would kill every process including our own shell).
for pat in 'gazebo' 'px4' 'roscore' 'rosmaster' 'empty_view' 'single_vehicle_spawn_xtd' 'multirotor_communication' 'get_local_pose' 'ego_swarm_transfer' 'uav_offboard_takeoff' 'capture_views' 'mavros' 'map_server' 'roslaunch'; do
  pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null || true
done
sleep 2

wait_master() {
  for _ in $(seq 1 40); do
    timeout 3 rostopic list >/dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}

echo '[2/7] 启动空世界 gazebo (GUI)'
export GAZEBO_MODEL_PATH=/home/dev/PX4-Autopilot/Tools/sitl_gazebo/models:/usr/share/gazebo-11/models
roslaunch gazebo_ros empty_world.launch world_name:=$DIR/world/empty_view.world gui:=true \
  > "$DIR/roslaunch_gazebo.log" 2>&1 &
wait_master || { echo 'ERROR: roscore 未就绪'; exit 1; }
timeout 90 rostopic echo -n1 /gazebo/model_states >/dev/null 2>&1 \
  || { echo 'ERROR: gazebo 未就绪'; exit 1; }
echo '    gazebo 就绪'

echo '[3/7] 启动 PX4 SITL + iris_red(iris_0) + MAVROS'
roslaunch "$DIR/launch/iris0_sim.launch" > "$DIR/roslaunch_iris0.log" 2>&1 &
timeout 90 rostopic echo -n1 /iris_0/mavros/state >/dev/null 2>&1 \
  || { echo 'ERROR: MAVROS 未就绪'; exit 1; }
echo '    MAVROS 就绪'

echo '[4/7] 启动 XTDrone 通信桥'
python3 /home/dev/XTDrone/sensing/pose_ground_truth/get_local_pose.py iris 1 &
python3 /home/dev/XTDrone/motion_planning/3d/ego_swarm_transfer.py iris 1 &
python3 /home/dev/XTDrone/communication/multirotor_communication.py iris 0 &
sleep 3

echo '[5/7] 起飞并悬停至 8m'
python3 /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff.py \
  --altitude 8 --uavs iris_0 --no-start-bridge --timeout 120 \
  > "$DIR/takeoff.log" 2>&1 &
TAKEOFF_PID=$!

echo '[6/7] 等待飞机到达 8m 并采集 32 个视角照片'
python3 - <<'PYEOF'
import rospy, sys
from gazebo_msgs.msg import ModelStates
rospy.init_node('wait_airborne_probe', anonymous=True)
t0 = rospy.Time.now().to_sec()
while rospy.Time.now().to_sec() - t0 < 120:
    try:
        ms = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
    except rospy.ROSException:
        continue
    try:
        i = ms.name.index('iris_0')
    except ValueError:
        i = -1
    if i >= 0 and ms.pose[i].position.z > 7.6:
        sys.exit(0)
sys.exit(1)
PYEOF
if [ $? -ne 0 ]; then
  echo 'ERROR: 飞机未在 120s 内到达 8m，见 takeoff.log'
  exit 1
fi

python3 "$DIR/scripts/capture_views.py"
CAP_RC=$?

echo '[7/7] 完成'
echo "输出目录: $DIR/images"
echo "已保存照片数: $(ls -1 "$DIR/images" | wc -l)"
echo '--- 校验汇总 ---'
python3 "$DIR/scripts/verify_capture.py"
echo '仿真进程保持运行，可查看 GUI；如需关闭请告知。'
exit $CAP_RC
