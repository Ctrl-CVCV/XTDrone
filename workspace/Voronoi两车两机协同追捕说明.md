# 两车两机 Voronoi 双入侵协同追捕测试说明

## 系统组成

```text
追捕者：car0、car1、iris_0、iris_1
入侵者：car2、car3
区域：小房间内 6 m × 6 m 有界 Voronoi 区域
捕获判定：任意追捕者与某辆入侵车的二维距离 <= capture_distance
```

`car2` 从西北外圈经左门进入，`car3` 从东南外圈经右门进入。进入房间后，两辆入侵车分别朝自己的有界 Voronoi 元胞质心逃逸。

追捕者遵循原 MATLAB 多逃逸者逻辑：优先前往与存活入侵者共享的最近 Voronoi 边中点；没有共享边时追逐最近的存活入侵者。

## 捕获和返航逻辑

每辆入侵车都有独立状态和计时：

```text
entered
active
capture_time
captured_by
deleted
```

第一辆被捕获后：

1. 终端和 `/air_ground/pursuit/result` 报告捕获者、距离和该目标用时；
2. Gazebo 删除该模型；
3. 该目标立即从活动 Voronoi 点集中移除；
4. 四个追捕者继续追捕另一辆入侵车。

两辆都被捕获并删除后，状态进入 `CAPTURED -> RETURNING -> RETURNED`，两车两机返回追捕任务开始时记录的位置，无人机到位后进入 `HOVER`。

删除模型后，虚拟障碍缓存会在 `0.5 s` 后失效，不会在第一辆入侵车的捕获点留下幽灵障碍。

## 默认出生点和入场路线

```text
car0  ( 2.3,  2.4)  defender
car1  (-2.3, -2.4)  defender
car2  (-6.5,  6.6)  左门入侵
car3  ( 6.5, -6.5)  右门入侵
iris_0 (-2.3,  2.3)
iris_1 ( 2.3, -2.3)
```

路线位于：

```text
/home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/launch/voronoi_air_ground_pursuit.launch
```

## 首次更新后编译

本次修改了 `virtual_obstacle_node.cpp`，需要执行一次：

```bash
cd /home/dev/XTDrone-single-car/workspace/swarm_defense_ws
catkin_make
```

## 启动顺序

终端 1，统一 Gazebo、四车 Nav 和两机 EGO：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
roslaunch px4 multi_vehicle.launch \
  gui:=true \
  start_car_nav:=true \
  start_ego:=true \
  car2_max_vel_trans:=1.5 \
  car3_max_vel_trans:=1.5
```

模型采用错峰生成：`car0=3 s`、`car1=6 s`、`car2=9 s`、`car3=12 s`、`iris_0=15 s`、`iris_1=19 s`。Gazebo GUI 在 23 秒后启动，避免并发生成导致少车少飞机。

终端 2，Gazebo ground truth：

```bash
cd /home/dev/XTDrone/sensing/pose_ground_truth
python3 get_local_pose.py iris 2
```

终端 3，EGO 位姿转换：

```bash
cd /home/dev/XTDrone/motion_planning/3d
python3 ego_swarm_transfer.py iris 2
```

终端 4，两机 OFFBOARD、ARM、起飞：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
rosrun car3_swarm uav_offboard_takeoff.py --altitude 1.5
```

终端 5，RViz：

```bash
rviz -d /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/rviz/air_ground.rviz
```

终端 6，双入侵追捕：

```bash
source /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/devel/setup.bash
roslaunch car3_swarm voronoi_air_ground_pursuit.launch
```

## 状态与结果

```bash
rostopic echo /air_ground/pursuit/state
rostopic echo /air_ground/pursuit/result
```

状态：

```text
WAITING    等待四车两机、Nav/EGO 和两机飞行状态
APPROACH   car2/car3 分别沿外圈前往入口
PURSUIT    至少一辆入侵车已进入，围捕运行中
CAPTURED   两辆入侵车都已捕获，等待模型删除完成
RETURNING  四个追捕者返回任务开始位置
RETURNED   全部返回，无人机 HOVER
```

结果示例：

```text
CAPTURED car2 by iris_0 at planar distance 0.287 m; capture time 12.631 s
CAPTURED car3 by car1 at planar distance 0.311 m; capture time 19.842 s
ALL CAPTURED in 21.005 s | ...
ALL CAPTURED in 21.005 s | ...; all pursuers returned home
```

## 参数

追捕 launch：

```text
capture_distance       默认 0.5 m
return_delay           默认 0.0 s
return_tolerance       默认 0.25 m
evader_lookahead       默认 1.2 m
goal_period            默认 1.0 s
uav_return_retry       默认 10.0 s（无人机无进展时才重发返航目标）
```

统一 launch 独立控制两辆入侵车速度：

```bash
car2_max_vel_trans:=1.5
car3_max_vel_trans:=1.5
```

车辆公共速度和加速度参数：

```text
/home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_control/params/nav_to_pose_params.yaml
```

EGO-Swarm 无人机速度和加速度参数：

```text
/home/dev/ego_ws/src/ego_planner/plan_manage/launch/run_in_xtdrone.launch
```


## 双入侵车端到端实测（2026-08-22）

实测配置：追捕者为 `car0`、`car1`、`iris_0`、`iris_1`，入侵者为 `car2`、`car3`。两辆入侵车从左右两个入口进入，各自按动态 Voronoi 元胞质心方向逃逸。

一次完整无 GUI 测试结果：

```text
CAPTURED car3 by iris_1 at planar distance 0.498 m; capture time 9.000 s
CAPTURED car2 by car0 at planar distance 0.460 m; capture time 14.800 s
ALL CAPTURED in 14.800 s
Sequential UAV return active: iris_1
Sequential UAV return completed: iris_1
Sequential UAV return active: iris_0
Sequential UAV return completed: iris_0
Pursuit state -> RETURNED
... all pursuers returned home
```

捕获后每辆入侵车独立从 Gazebo 删除，系统继续追捕尚存目标。两辆都捕获后，`car0/car1` 并行返航；两架无人机按当时距离各自原位由近到远顺序返航，避免对角返航轨迹触发 EGO-Swarm 机间避碰死锁。等待中的无人机保持当前轨迹，不会反复收到静态目标；活动无人机仅在连续 10 秒无进展时重发目标。
