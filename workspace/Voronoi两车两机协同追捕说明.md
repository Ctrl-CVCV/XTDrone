# 两车两机 Voronoi 协同追捕实验说明

## 1. 实验目标

使用四个追捕者共同追捕一个入侵车辆：

```text
追捕者：car0、car1、iris_0、iris_1
逃逸者：car2
```

实验场景：

- Voronoi 有界区域为小房间中心处的 `6 m × 6 m` 正方形；
- `car2` 从左门进入小房间，进入后朝自身有界 Voronoi 元胞质心持续机动；
- 任意追捕者与 `car2` 的二维平面距离不大于 `capture_distance` 时判定捕获；
- `car2` 的动态目标始终限制在 6 m 区域内，不再判定逃逸。
- 捕获后 Gazebo 删除 `car2`，两车两机返回本次任务开始时记录的位置；
- 全部追捕者进入 `return_tolerance` 后，无人机切换为 `HOVER` 保持。

无人机飞行高度默认为 `1.5 m`。捕获距离采用二维平面距离；如果采用三维距离，空中无人机在 `1.5 m` 高度不可能满足 `0.3 m` 捕获条件。

## 2. MATLAB 逻辑到 ROS 的映射

保留的 MATLAB 追捕逻辑：

1. 对所有追捕者和逃逸者计算正方形区域内的有界 Voronoi 元胞；
2. 如果追捕者与 `car2` 共享 Voronoi 边，追捕者目标为共享边中点；
3. 如果二者不共享 Voronoi 边，追捕者直接朝 `car2` 移动；
4. 每轮更新追捕目标并检查捕获距离。

ROS 执行层与 MATLAB 示例的区别：

- 保留 MATLAB 的逃逸者控制律：`car2` 朝自己有界 Voronoi 元胞的几何质心移动；
- 入场前使用确定性路线让 `car2` 从左门进入，进入后立即切换为动态质心目标；
- MATLAB 中直接积分更新质点位置；ROS 中把目标发送给车辆 Nav 和无人机 EGO-Swarm，由各自控制器执行并避障。

节点不依赖 SciPy 或 Shapely。有界 Voronoi 使用半平面裁剪直接计算，适用于当前 5 个智能体的小规模实时任务。

## 3. 边界和路线

6 m 正方形中心采用房间 SDF 的中心：

```text
center = (-0.11576, -0.00882)
x = [-3.11576, 2.88424]
y = [-3.00882, 2.99118]
```

`car2` 默认路线：

```text
出生点 (-6.5, 6.6)
  -> 西北外圈
  -> 左门外侧
  -> 穿过左门
  -> 小房间中心
  -> 穿过右门
  -> 右侧外圈
```

具体 waypoint 位于：

```text
/home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/launch/voronoi_air_ground_pursuit.launch
```

## 4. 启动顺序

### 4.1 统一仿真

终端 1：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash

roslaunch px4 multi_vehicle.launch \
  gui:=true \
  start_car_nav:=true \
  start_ego:=true \
  car2_max_vel_trans:=1.5
```

### 4.2 Gazebo ground-truth 位姿

终端 2：

```bash
cd /home/dev/XTDrone/sensing/pose_ground_truth
python3 get_local_pose.py iris 2
```

### 4.3 EGO-Swarm 位姿转换

终端 3：

```bash
cd /home/dev/XTDrone/motion_planning/3d
python3 ego_swarm_transfer.py iris 2
```

### 4.4 两机 OFFBOARD、ARM 和起飞

终端 4：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
rosrun car3_swarm uav_offboard_takeoff.py --altitude 1.5
```

等待脚本确认两架飞机都已经：

```text
connected: True
armed: True
mode: OFFBOARD
高度约 1.5 m
```

### 4.5 RViz

终端 5：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
rviz -d /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/rviz/air_ground.rviz
```

RViz 的 `voronoi_pursuit` 图层显示：

- 白色线框：6 m 追捕边界；
- 彩色线框：各智能体的有界 Voronoi 元胞；
- 彩色球：四个追捕者当前追捕目标；
- 红色圆：以 `car2` 为中心、半径为 `capture_distance` 的捕获圈；
- 文本：当前追捕状态。

### 4.6 启动追捕

终端 6：

```bas
```

默认 `auto_start=true`。节点会先检查：

- Gazebo 中存在两车、两机和 `car2`；
- 两架飞机 MAVROS connected；
- 两架飞机均为 `OFFBOARD + armed`；
- 两套 EGO 和三套车辆 Nav 均订阅目标话题。

全部就绪后，`car2` 开始走左门到右门的路线。它进入 6 m 正方形后，四个追捕者才开始执行 Voronoi 围捕。

## 5. 手动触发模式

需要先观察、再手动开始时：

```bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch auto_start:=false
```

确认系统就绪后：

```bash
rosservice call /air_ground/pursuit/start
```

## 6. 状态和结果

当前状态：

```bash
rostopic echo /air_ground/pursuit/state
```

状态含义：

```text
WAITING   等待依赖或手动触发
APPROACH  car2 沿外圈接近左门
PURSUIT   car2 已进入房间，按 Voronoi 元胞质心逃逸，围捕运行中
CAPTURED  任意追捕者进入 capture_distance 捕获圈
RETURNING car2 已删除，两车两机正在返回任务开始位置
RETURNED  四个追捕者均已返回，无人机进入 HOVER
```

最终结果：

```bash
rostopic echo /air_ground/pursuit/result
```

捕获结果会包含执行捕获的追捕者名称和实际距离，例如：

```text
CAPTURED by iris_0 at planar distance 0.287 m
```

返航完成后结果更新为：

```text
CAPTURED by iris_0 at planar distance 0.287 m; capture time 18.627 s; all pursuers returned home
```

返航参数位于 `voronoi_air_ground_pursuit.launch`：

```xml
return_delay="1.0"        <!-- 捕获到开始返航的延迟，秒 -->
return_tolerance="0.25"   <!-- 四个追捕者的返航到位容差，米 -->
return_goal_period="1.0"  <!-- 返航目标重发周期，秒 -->
```

这里的“原位”不是写死的出生坐标，而是启动追捕任务时四个追捕者的 Gazebo 世界坐标。

## 7. 速度参数与调整位置

### 7.1 三辆车

车辆 Nav 的公共默认参数位于：

```text
/home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_control/params/nav_to_pose_params.yaml
```

关键参数：

```yaml
max_vel_trans: 0.7   # 最大平移速度，m/s
max_vel_theta: 0.8   # 最大旋转速度，rad/s
acc_lim_trans: 0.6   # 平移加速度限制，m/s^2
acc_lim_theta: 2.0   # 旋转加速度限制，rad/s^2
```

当前追捕系统只对逃逸车做了 launch 覆盖：

```text
car0 = 0.7 m/s
car1 = 0.7 m/s
car2 = 0.25 m/s
```

`car2` 覆盖参数位于 `multi_car3_nav.launch`，并由统一 `multi_vehicle.launch` 暴露为 `car2_max_vel_trans`。例如进一步降到 `0.20 m/s`：

```bash
roslaunch px4 multi_vehicle.launch gui:=true start_car_nav:=true start_ego:=true car2_max_vel_trans:=0.20
```

这些参数由 `nav_to_pose` 启动时读取，修改后必须重新启动统一仿真。

### 7.2 两架无人机

EGO-Swarm 的轨迹速度、加速度上限位于：

```text
/home/dev/ego_ws/src/ego_planner/plan_manage/launch/run_in_xtdrone.launch
```

当前值：

```xml
<arg name="max_vel" value="1"/>
<arg name="max_acc" value="2"/>
```

两架飞机当前共同使用：

```text
max_vel = 1.0 m/s
max_acc = 2.0 m/s^2
```

这两个值会传入 `advanced_param_xtdrone.xml` 的 `manager/max_vel`、`manager/max_acc`、`optimization/max_vel`、`optimization/max_acc` 和 B-spline 速度/加速度限制。修改后必须重启 EGO-Swarm。

## 8. 捕获判据与捕获后的系统反应

追捕节点只在 `car2` 已进入 6 m 正方形、状态为 `PURSUIT` 后执行捕获判断。每个控制周期读取 Gazebo 世界真实位姿，计算：

```text
d_i = sqrt((x_i-x_car2)^2 + (y_i-y_car2)^2)
i in {car0, car1, iris_0, iris_1}
```

判据为：

```text
min(d_i) <= capture_distance
```

这是二维平面距离，任意一个追捕者进入捕获圈即立即成功；当前没有持续时间去抖，也不要求四个追捕者同时进入捕获圈。

捕获成功后节点会：

1. 将状态置为 `CAPTURED`；
2. 在 `/air_ground/pursuit/result` 发布捕获者名称、实际距离和从入场开始计算的 Gazebo 仿真用时；
3. 停止继续计算和发布新的 Voronoi 追捕目标；
4. 给 `car0`、`car1`、`car2` 发布各自当前位置作为保持目标；
5. 给两架无人机发布各自当前位置作为 EGO 目标，并发送 `HOVER`；
6. RViz 保留最终状态和捕获圈。

系统不会在捕获后自动让无人机降落或 DISARM，两架飞机保持悬停，等待人工结束任务。

## 9. 其他参数调整

直接在启动命令中修改常用参数：

```bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch \
  capture_distance:=0.3 \
  boundary_side:=6.0 \
  uav_altitude:=1.5 \
  goal_period:=1.0

  roslaunch car3_swarm voronoi_air_ground_pursuit.launch \
   return_delay:=1.0 \
   return_tolerance:=0.50
```

主要参数：

```text
capture_distance  二维捕获距离，当前 launch 默认 0.5 m
boundary_side     Voronoi 正方形边长，默认 6.0 m
uav_altitude      追捕期间无人机目标高度，默认 1.5 m
goal_period       追捕目标重算/发布周期，默认 1.0 s
```

## 10. 注意事项

- 不要同时启动 `multi_car3_mission.launch`。旧 `intruder_node`、`patrol_node` 会与新追捕节点争抢车辆控制权。
- 不要同时运行 `air_ground_unified_control.py`，它会持续发布另一套无人机 setpoint。
- `uav_offboard_takeoff.py` 必须保持运行，因为它可能负责维持 XTDrone 通信桥。
- 若 EGO CPU 占用过高，可将 `goal_period` 调大到 `1.5` 或 `2.0` 秒。
- 当前捕获采用 MATLAB 风格的“任意一个追捕者进入捕获距离”判据，不要求四个追捕者同时进入捕获圈。

## 11. 关闭

1. 如果追捕仍在运行，先停止追捕 launch；
2. 两架飞机执行 `AUTO.LAND`；
3. 落地后执行 `DISARM`；
4. 结束一键起飞脚本；
5. 停止 `ego_swarm_transfer.py iris 2`；
6. 停止 `get_local_pose.py iris 2`；
7. 最后关闭统一仿真。

## 12. 暂停、恢复与进程冲突清理

### 12.1 只暂停/恢复 Gazebo 仿真时间

暂停后所有 ROS 节点仍然保留，只是 Gazebo 的物理时间停止，适合临时观察现场：

```bash
rosservice call /gazebo/pause_physics "{}"
```

恢复：

```bash
rosservice call /gazebo/unpause_physics "{}"
```

这不会解决重复 launch 或同名节点冲突。

### 12.2 只暂停协同追捕

先让两架无人机锁定当前位置，再停止追捕节点：

```bash
rostopic pub -1 /xtdrone/iris_0/cmd std_msgs/String "data: 'HOVER'"
rostopic pub -1 /xtdrone/iris_1/cmd std_msgs/String "data: 'HOVER'"
rosnode kill /voronoi_air_ground_pursuit
```

统一 Gazebo、PX4、MAVROS、Nav、EGO、ground-truth 和 transfer 会继续运行。重新启动追捕时执行：

```bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch \
  return_delay:=1.0 \
  return_tolerance:=0.50
```

如果该命令显示 `auto-starting new master`，说明统一仿真已经不在线。立即按 `Ctrl+C`，不要在这个空 Master 上继续运行追捕。

### 12.3 一键正常停止全部相关进程

遇到以下现象时使用：

- `Reason: new node registered with same name`；
- `No ROS master` 或大量 XML-RPC `Connection refused`；
- Gazebo 中模型重复、缺失；
- `pgrep` 显示多份 `multi_vehicle.launch`。

可以把下面整段一次性粘贴到一个终端。它只匹配本项目的明确进程，不会执行 `killall python3`：

```bash
pkill -INT -f '[/]voronoi_air_ground_pursuit.py'
pkill -INT -f '[/]uav_offboard_takeoff.py'
pkill -INT -f '[/]multirotor_communication.py iris [01]'
pkill -INT -f '[/]ego_swarm_transfer.py iris 2'
pkill -INT -f '[/]get_local_pose.py iris 2'
pkill -INT -f '[/]roslaunch px4 multi_vehicle.launch'

sleep 10

pgrep -af 'roslaunch px4 multi_vehicle.launch|rosmaster|gzserver|gzclient|get_local_pose.py|ego_swarm_transfer.py|uav_offboard_takeoff.py|multirotor_communication.py|voronoi_air_ground_pursuit.py'
```

正常情况下，最后一条 `pgrep` 不应显示活动进程。显示 `<defunct>` 或状态 `Z` 的 Gazebo 进程只是由 PID 1 尚未回收的僵尸，不消耗实际 CPU，也无法再通过 `kill` 清除。

只有等待 10 秒后仍有非 `Z` 的明确残留进程时，才执行第二阶段：

```bash
pkill -TERM -f '[/]voronoi_air_ground_pursuit.py'
pkill -TERM -f '[/]uav_offboard_takeoff.py'
pkill -TERM -f '[/]multirotor_communication.py iris [01]'
pkill -TERM -f '[/]ego_swarm_transfer.py iris 2'
pkill -TERM -f '[/]get_local_pose.py iris 2'
pkill -TERM -f '[/]roslaunch px4 multi_vehicle.launch'
pkill -TERM -f '[/]gzserver .*nesting_room.world'
```

清理后必须按“统一仿真 → ground-truth → transfer → 一键起飞 → 追捕”的顺序重新启动。启动统一仿真前先检查：

```bash
pgrep -af 'roslaunch px4 multi_vehicle.launch'
```

没有输出才可以启动；启动后应始终只有一份结果。

## 13. 单车/双车入侵模式切换

围捕节点现在通过 `intruder_mode` 选择本轮有效入侵车辆，默认是双车模式。

单车入侵（只追捕 `car2`）：

```bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch \
  intruder_mode:=single \
  return_delay:=1.0 \
  return_tolerance:=0.50
```

双车入侵（同时追捕 `car2`、`car3`）：

```bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch \
  intruder_mode:=dual \
  return_delay:=1.0 \
  return_tolerance:=0.50
```

统一仿真仍然生成 `car0` 至 `car3` 四辆车。`single` 模式只是让围捕节点忽略 `car3`：不等待它的 Nav 订阅者、不向它发布入场或逃逸目标，也不把它计入捕获完成条件。捕获 `car2` 后会立即进入现有顺序返航流程。`dual` 模式则要求 `car2`、`car3` 均被捕获后才返航。

运行中不能热切换模式。切换前按第 12.2 节停止追捕节点，再使用另一条命令重新启动；无需重启 Gazebo。

## 14. 地图中央固定云台

`nesting_room.world` 已加入固定模型 `camera_gimbal`，位置为：

```text
x = -0.11576
y = -0.00882
z =  0.02
```

模型源自 `/home/dev/XTDrone/camera_urdf`，已连同 mesh 独立放入 `basic_room_sim/models/camera_gimbal`，且设为 `static`，不会受重力掉落或被车辆碰走。重新加载 world 后可检查：

```bash
rosservice call /gazebo/get_world_properties | grep camera_gimbal
rostopic hz /gimbal_camera/image_raw
rostopic echo -n 1 /gimbal_camera/camera_info
```

相机图像和内参话题分别是 `/gimbal_camera/image_raw`、`/gimbal_camera/camera_info`。修改 world 或模型后必须重启统一仿真才会生效。

## 15. EGO 四面虚拟障碍墙

统一 launch 在 `start_ego:=true` 时默认启动 `ego_virtual_boundary.py`，将当前 6 m 围捕边界生成为从地面到 `3.0 m` 高的四面静态点云墙，并分别转换到两架无人机的 MAVROS 本地坐标。每架只注入一次，送达后节点自动退出，不会持续刷新千万级局部体素。这样 EGO 不仅收到边界内的目标点，还会把边界本身作为障碍，避免为绕障规划到小房间外。

相关参数位于 `multi_vehicle.launch`：

```text
use_ego_virtual_boundary = true
ego_boundary_z_max       = 3.0 m
ego_boundary_spacing     = 0.3 m
```

该功能修改了 EGO 栅格 odom remap，因此应用后必须重启统一仿真和两套 EGO；只重启 `voronoi_air_ground_pursuit.launch` 不够。
