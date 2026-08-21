# 两车两机 Voronoi 协同追捕实验说明

## 1. 实验目标

使用四个追捕者共同追捕一个入侵车辆：

```text
追捕者：car0、car1、iris_0、iris_1
逃逸者：car2
```

实验场景：

- Voronoi 有界区域为小房间中心处的 `6 m × 6 m` 正方形；
- `car2` 从左门进入小房间，尝试从右门逃出；
- 任意追捕者与 `car2` 的二维平面距离不大于 `0.3 m` 时判定捕获；
- `car2` 先穿过右门对应的正方形边界时判定逃逸。

无人机飞行高度默认为 `1.5 m`。捕获距离采用二维平面距离；如果采用三维距离，空中无人机在 `1.5 m` 高度不可能满足 `0.3 m` 捕获条件。

## 2. MATLAB 逻辑到 ROS 的映射

保留的 MATLAB 追捕逻辑：

1. 对所有追捕者和逃逸者计算正方形区域内的有界 Voronoi 元胞；
2. 如果追捕者与 `car2` 共享 Voronoi 边，追捕者目标为共享边中点；
3. 如果二者不共享 Voronoi 边，追捕者直接朝 `car2` 移动；
4. 每轮更新追捕目标并检查捕获距离。

与 MATLAB 示例不同的部分：

- MATLAB 中逃逸者朝自己 Voronoi 元胞形心移动；
- 本实验按任务要求，将逃逸者改成确定性的“左门进入、右门逃出”路线；
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
roslaunch px4 multi_vehicle.launch gui:=true start_car_nav:=true start_ego:=true
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
- 红色圆：以 `car2` 为中心、半径 `0.3 m` 的捕获圈；
- 文本：当前追捕状态。

### 4.6 启动追捕

终端 6：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch
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
PURSUIT   car2 已进入房间，Voronoi 围捕运行中
CAPTURED  任意追捕者进入 0.3 m 捕获圈
ESCAPED   car2 穿过右门边界
```

最终结果：

```bash
rostopic echo /air_ground/pursuit/result
```

捕获结果会包含执行捕获的追捕者名称和实际距离，例如：

```text
CAPTURED by iris_0 at planar distance 0.287 m
```

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
min(d_i) <= 0.3 m
```

这是二维平面距离，任意一个追捕者进入捕获圈即立即成功；当前没有持续时间去抖，也不要求四个追捕者同时进入捕获圈。

捕获成功后节点会：

1. 将状态置为 `CAPTURED`；
2. 在 `/air_ground/pursuit/result` 发布捕获者名称和实际距离；
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
```

主要参数：

```text
capture_distance  二维捕获距离，默认 0.3 m
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

