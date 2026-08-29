#!/usr/bin/env bash
# 小车视角采集 - 阶段1: 清理 UAV 仿真并启动全新空世界加载 car3_red。
# 采集阶段(阶段2)由 capture_car_radii.sh 另行执行。
export ROS_MASTER_URI=http://localhost:11311

echo "==> [1/3] 清理旧仿真进程 (gazebo/px4/mavros/桥接)"
for p in roslaunch gzserver gzclient px4 mavros rosmaster get_local_pose ego_swarm_transfer multirotor_communication uav_offboard_takeoff; do
    pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null && echo "    killed: $p" || true
done
sleep 3

echo "==> [2/3] 启动空世界 gazebo (GUI) + 加载 car3_red"
source /opt/ros/noetic/setup.bash
source /home/dev/catkin_ws/devel/setup.bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash

roslaunch gazebo_ros empty_world.launch gui:=true headless:=false \
    world:=/home/dev/XTDrone-single-car/workspace/intruder_car_capture/world/empty_car.world \
    > /home/dev/XTDrone-single-car/workspace/intruder_car_capture/car_world.log 2>&1 &
GAZEBO_PID=$!
echo "    gazebo PID=$GAZEBO_PID  日志: intruder_car_capture/car_world.log"

until rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q 'name:'; do sleep 1; done
echo "    gazebo model_states 已就绪"

roslaunch /home/dev/XTDrone-single-car/workspace/intruder_car_capture/launch/car3_red_spawn.launch \
    > /home/dev/XTDrone-single-car/workspace/intruder_car_capture/car_spawn.log 2>&1 &
SPAWN_PID=$!
echo "    car3_red spawn PID=$SPAWN_PID  日志: intruder_car_capture/car_spawn.log"

echo "==> [3/3] 等待 car3_red 出现在 model_states"
for i in $(seq 1 30); do
    if rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -q 'car3_red'; then
        echo "    car3_red 已加载"
        break
    fi
    sleep 2
done
rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -A4 'car3_red' | head -8
echo "==> 完成: 小车已加载进空世界，可查看 GUI；确认后再开始采集。"
