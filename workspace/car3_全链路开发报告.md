# car3 单车 Gazebo + ros_control + Navigation 全链路开发报告

- 范围：**单辆 car3 麦轮（mecanum）小车**（无机械臂、无无人机、无双车/多车扩展）
- 链路：`/cmd_vel (Twist)` → mecanum 逆运动学节点 → 4× `JointVelocityController` → ros_control → gazebo_ros_control → Gazebo 轮关节
- 里程计：Gazebo ground truth → `/odom`（非轮式里程计）
- 建图：car3 + 激光 → slam_gmapping → map_saver → `basic_room.pgm/.yaml`
- 导航：map_server + AMCL + move_base（DWA 全向局部规划，支持 `linear.y`）
- 日期：2026-08-15

---

## 1. 环境

| 项 | 值 |
|---|---|
| 宿主机 | Ubuntu，Docker（`DOCKER_CONTEXT=default` 系统 docker） |
| 容器 | `xtdrone-dev`（镜像 xtdrone-noetic-px4:1.13.2-v1.3，Ubuntu 20.04.6） |
| ROS | ROS1 Noetic（/opt/ros/noetic） |
| Gazebo | 11.15.1（gazebo_ros 方式启动） |
| 显示 | 容器内 Xvfb `:99` + `LIBGL_ALWAYS_SOFTWARE=1`（llvmpipe 软件渲染） |
| 工作区 | 容器 `/workspace/swarm_defense_ws` ↔ 宿主机 `~/xtdrone_docker/workspace/swarm_defense_ws` |
| 环境脚本 | 容器内 `/home/dev/car3_env.sh`（每次 docker exec 需 source） |

新增安装（阶段 2/4）：`ros-noetic-velocity-controllers`、`ros-noetic-gmapping`（导航栈 ros-noetic-navigation 1.17.3 已预装）。

XTDrone 核心源码改动（已向用户说明）：`sitl_config/ugv/car3/urdf/car3.urdf` ——
① 4 个轮关节 `fixed` → `continuous` + axis/limit + 4 个 `SimpleTransmission`（VelocityJointInterface）；
② 移除 planar_move 插件，替换为 `gazebo_ros_control` 插件；
③ 新增 `car3_mecanum_grip` 麦轮辊子抓地力模型插件；
④ 轮子碰撞面摩擦 `mu1=mu2=0`（ODE 表面摩擦无法表达麦轮辊子各向异性，抓地力完全由自定义插件提供）。

---

## 2. 系统链路图

```
                              ┌────────────────────────────── Gazebo ──────────────────────────────┐
                              │                                                                      │
                              │  model_states (gazebo ground truth)                                 │
                              │       │                                                              │
                              │       ▼                                                              │
                              │  ground_truth_odom ──► /odom (Odometry) ──► odom─►base_footprint TF │
                              │  (car3_control)                                                   │  │
                              │                                                                    │  │
                              │  laser sensor ──► /scan (LaserScan, 720 束 ±90°, 15 Hz)            │  │
                              │                                                                    │  │
                              │  wheel joints ◄── gazebo_ros_control ◄── JointVelocityController×4 │  │
                              │       (r=0.0489, 连续关节)                     (ros_control)        │  │
                              └────────────────────────────────────────────────────────────────────┘
                                              ▲                            ▲
                                              │ wheel 速度指令               │ 关节速度指令
                                              │ (std_msgs/Float64 ×4)       │
                                              │                            │
                                    ┌─────────┴────────────────────────────┴──────────┐
                                    │           mecanum_controller_node (car3_control) │
                                    │           麦轮逆运动学 IK                        │
                                    └──────────────────────▲───────────────────────────┘
                                                           │ /cmd_vel (geometry_msgs/Twist)
                                    ┌──────────────────────┴───────────────────────────┐
                                    │                    move_base                       │
                                    │   global_planner(GlobalPlanner) ──► global_plan   │
                                    │   local_planner(DWAPlannerROS, 全向) ──► /cmd_vel │
                                    │   global_costmap ◄── map_server ◄── basic_room.*  │
                                    │   local_costmap  ◄── /scan (obstacle_layer)       │
                                    └───────────▲──────────────────────────▲────────────┘
                                                │ map─►odom TF              │ map─►base_footprint (定位)
                                    ┌───────────┴──────────┐     ┌─────────┴──────────┐
                                    │  AMCL (omni 模型)     │     │  /odom + TF        │
                                    │  /scan + /map 定位    │     │  ground_truth_odom │
                                    └──────────────────────┘     └────────────────────┘

建图阶段（Phase 4）：/cmd_vel 由键盘/脚本直接发布 → move_base 不参与；
                       slam_gmapping 订阅 /scan + odom─►base_footprint TF，发布 map─►odom；
                       map_saver 保存地图。
```

---

## 3. 目录树

```
/workspace/swarm_defense_ws/src/
├── basic_room_sim/                        # 房间世界（Gazebo 环境）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/basic_room.launch
│   ├── worlds/basic_room.world
│   └── models/basic_room/
│       ├── model.config
│       └── model.sdf                     # 5 面墙 + 1 柱子（12.6 m × 20 m 房间）
└── car3_control/                         # 本项目新增控制/导航包
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── car3_controllers.yaml         # 1 joint_state + 4 JointVelocity 控制器
    │   └── gazebo_pid.yaml               # gazebo_ros_control 关节 pid_gains
    ├── launch/
    │   ├── car3_bringup.launch           # Gazebo + 生成车 + ros_control + odom
    │   ├── car3_control.launch           # 控制器 spawner + mecanum 节点
    │   ├── car3_slam.launch              # bringup + slam_gmapping（建图）
    │   ├── car3_slam_nogazebo.launch     # 同上，不含 gzserver（调试用）
    │   └── car3_nav.launch               # map_server + AMCL + move_base（导航）
    ├── maps/
    │   ├── basic_room.pgm                # 704×704 @ 0.05 m/pix
    │   └── basic_room.yaml
    ├── params/
    │   ├── amcl_params.yaml
    │   ├── costmap_common_params.yaml    # footprint/障碍层/膨胀层（两 costmap 共用）
    │   ├── global_costmap_params.yaml
    │   ├── local_costmap_params.yaml
    │   ├── dwa_local_planner_params.yaml # 全向 DWA（已验证）
    │   └── move_base_params.yaml
    └── src/
        ├── mecanum_controller_node.cpp   # /cmd_vel → 4 轮速度（IK）
        ├── ground_truth_odom_node.cpp    # model_states → /odom + TF
        └── car3_mecanum_grip_plugin.cpp  # Gazebo 麦轮辊子抓地力模型插件

/home/dev/XTDrone/sitl_config/ugv/car3/     # 车体模型（XTDrone 原包，已按约定改造）
├── urdf/car3.urdf                          # 关节/transmission/插件/激光/IMU
└── meshes/                                 # base_link.STL、wheel_*.STL、laser_link.STL、imu_link.STL
```

---

## 4. 几何参数与麦轮运动学

### 4.1 几何参数（由 URDF/STL 实测）

| 参数 | 符号 | 值 |
|---|---|---|
| 车轮半径 | r | 0.0489 m |
| 轮距（x 向半距） | L | 0.0973 m（前后轮中心 x = ±0.0973） |
| 轴距（y 向半距） | W | 0.1058 m（左右轮中心 y = ±0.1058） |
| 等效旋转半径 | L+W | 0.2031 m |
| 底座质量 | m | 20.0 kg（base_link；单轮 0.265 kg） |
| 激光安装偏移 | — | laser_link 在 base_link 的 (0.14621, 0, 0.0874) |
| 轮关节轴 | — | (0, 1, 0)（连续关节） |

### 4.2 逆运动学（X 型麦轮，`/cmd_vel` → 轮角速度）

ROS 约定：+x 前、+y 左、+z 上。轮序 FL/FR/RL/RR（左前/右前/左后/右后）。

```
ω_fl = ( vx − vy − (L+W)·ωz ) / r
ω_fr = ( vx + vy + (L+W)·ωz ) / r
ω_rl = ( vx + vy − (L+W)·ωz ) / r
ω_rr = ( vx − vy + (L+W)·ωz ) / r
```

（符号已通过六项运动测试逐一验证：前进/后退/左移/右移/左转/右转，全部与指令一致；
每轮保留 `sign_*` 参数（默认 1.0）用于关节轴方向修正。）

### 4.3 麦轮辊子抓地力模型（Gazebo 插件）

ODE 表面摩擦的方向固定在轮体上、随轮转动，无法表达麦轮辊子“抓地方向不随轮自转”的
各向异性。插件在每次世界更新时对每个轮子计算接触点滑移：

```
滑移 s = v_B + ω_B × (p_W − p_B) − ω_wheel·r·x_axis
抓地方向 u：LF/RB = (x−y)/√2，RF/LB = (x+y)/√2（基座系，随基座姿态旋转）
抓地力 F = −clamp(c·(s·u), μN) · u    施加于基座（轮中心处）
```

参数：`grip_gain=1500`、`grip_force_max=52.0`（≈ μN），轮碰撞面 `mu=mu2=0` 只保留法向支撑。
（grip_gain 5200 时离散不稳定，1500 稳定。）

---

## 5. 全部文件内容

### 5.1 car3_control/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(car3_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mecanum_controller_node src/mecanum_controller_node.cpp)
target_link_libraries(mecanum_controller_node ${catkin_LIBRARIES})

add_executable(ground_truth_odom_node src/ground_truth_odom_node.cpp)
target_link_libraries(ground_truth_odom_node ${catkin_LIBRARIES})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(car3_mecanum_grip_plugin SHARED src/car3_mecanum_grip_plugin.cpp)
target_link_libraries(car3_mecanum_grip_plugin
  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(car3_mecanum_grip_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS mecanum_controller_node ground_truth_odom_node car3_mecanum_grip_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

### 5.2 car3_control/package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>car3_control</name>
  <version>0.1.0</version>
  <description>Mecanum wheel controller for car3: /cmd_vel to 4 wheel velocity commands</description>
  <maintainer email="liuhanxu@example.com">liuhanxu</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>

  <export>
  </export>
</package>
```

### 5.3 car3_control/config/car3_controllers.yaml

```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

wheel_lf_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: wheel_lf_joint
  pid: {p: 1.0, i: 0.0, d: 0.0}

wheel_rf_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: wheel_rf_joint
  pid: {p: 1.0, i: 0.0, d: 0.0}

wheel_lb_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: wheel_lb_joint
  pid: {p: 1.0, i: 0.0, d: 0.0}

wheel_rb_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: wheel_rb_joint
  pid: {p: 1.0, i: 0.0, d: 0.0}
```

### 5.4 car3_control/config/gazebo_pid.yaml

```yaml
gazebo_ros_control:
  pid_gains:
    wheel_lf_joint: {p: 0.15, i: 0.0, d: 0.00002}
    wheel_rf_joint: {p: 0.15, i: 0.0, d: 0.00002}
    wheel_lb_joint: {p: 0.15, i: 0.0, d: 0.00002}
    wheel_rb_joint: {p: 0.15, i: 0.0, d: 0.00002}
```

（注意：必须在 spawn 之前加载到参数服务器 —— gazebo_ros_control 初始化时读取。）

### 5.5 car3_control/launch/car3_bringup.launch

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <arg name="gui" default="false"/>
    <arg name="x" default="0.0"/>
    <arg name="y" default="0.0"/>
    <arg name="z" default="0.01"/>

    <!-- Gazebo + basic_room world -->
    <include file="$(find basic_room_sim)/launch/basic_room.launch">
        <arg name="gui" value="$(arg gui)"/>
    </include>

    <!-- car3 model -->
    <param name="robot_description" textfile="$(find car3)/urdf/car3.urdf"/>

    <!-- must be on the param server before spawn: gazebo_ros_control
         reads pid_gains while initializing the robot simulation interface -->
    <rosparam file="$(find car3_control)/config/gazebo_pid.yaml" command="load"/>

    <node name="spawn_car3" pkg="gazebo_ros" type="spawn_model"
          args="-x $(arg x) -y $(arg y) -z $(arg z) -param robot_description -urdf -model car3"
          output="screen"/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
        <param name="publish_frequency" value="50.0"/>
    </node>

    <!-- ros_control + mecanum cmd_vel -->
    <include file="$(find car3_control)/launch/car3_control.launch"/>

    <!-- Phase 3: gazebo ground truth -> /odom + odom->base_footprint TF -->
    <node name="ground_truth_odom" pkg="car3_control" type="ground_truth_odom_node"
          output="screen"/>
</launch>
```

### 5.6 car3_control/launch/car3_control.launch

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <!-- load 4 wheel JointVelocityControllers + joint_state_controller -->
    <rosparam file="$(find car3_control)/config/car3_controllers.yaml" command="load"/>

    <node name="controller_spawner" pkg="controller_manager" type="spawner"
          args="joint_state_controller
                wheel_lf_velocity_controller
                wheel_rf_velocity_controller
                wheel_lb_velocity_controller
                wheel_rb_velocity_controller"
          output="screen"/>

    <!-- /cmd_vel -> 4 wheel velocity commands -->
    <node name="mecanum_controller_node" pkg="car3_control" type="mecanum_controller_node"
          output="screen">
        <param name="wheel_radius" value="0.0489"/>
        <param name="wheel_base_x_half" value="0.0973"/>
        <param name="wheel_base_y_half" value="0.1058"/>
        <param name="sign_fl" value="1.0"/>
        <param name="sign_fr" value="1.0"/>
        <param name="sign_rl" value="1.0"/>
        <param name="sign_rr" value="1.0"/>
    </node>
</launch>
```

### 5.7 car3_control/launch/car3_slam.launch（建图）

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <!-- sim + ros_control + odom + lidar -->
    <include file="$(find car3_control)/launch/car3_bringup.launch"/>

    <!-- Phase 4: gmapping SLAM -->
    <node name="slam_gmapping" pkg="gmapping" type="slam_gmapping">
        <param name="base_frame" value="base_footprint"/>
        <param name="odom_frame" value="odom"/>
        <param name="map_frame" value="map"/>
        <param name="map_update_interval" value="1.0"/>
        <param name="maxUrange" value="10.0"/>
        <param name="maxRange" value="10.0"/>
        <param name="sigma" value="0.05"/>
        <param name="kernelSize" value="1"/>
        <param name="lstep" value="0.05"/>
        <param name="astep" value="0.05"/>
        <param name="iterations" value="5"/>
        <param name="lsigma" value="0.075"/>
        <param name="ogain" value="3.0"/>
        <param name="lskip" value="0"/>
        <param name="minimumScore" value="50"/>
        <param name="srr" value="0.1"/>
        <param name="srt" value="0.2"/>
        <param name="str" value="0.1"/>
        <param name="stt" value="0.2"/>
        <param name="linearUpdate" value="0.2"/>
        <param name="angularUpdate" value="0.2"/>
        <param name="temporalUpdate" value="-1.0"/>
        <param name="resampleThreshold" value="0.5"/>
        <param name="particles" value="30"/>
        <param name="xmin" value="-4.0"/>
        <param name="ymin" value="-5.0"/>
        <param name="xmax" value="5.0"/>
        <param name="ymax" value="3.0"/>
        <param name="delta" value="0.05"/>
        <param name="llsamplerange" value="0.01"/>
        <param name="llsamplestep" value="0.01"/>
        <param name="lasamplerange" value="0.005"/>
        <param name="lasamplestep" value="0.005"/>
    </node>
</launch>
```

（`car3_slam_nogazebo.launch` 内容相同，仅去掉 gzserver —— 供外部调试启动。）

### 5.8 car3_control/launch/car3_nav.launch（导航）

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <!-- Phase 5: map_server + AMCL + move_base (omnidirectional DWA) -->
    <arg name="map_file" default="$(find car3_control)/maps/basic_room.yaml"/>
    <arg name="init_x" default="2.544"/>
    <arg name="init_y" default="-0.010"/>
    <arg name="init_a" default="0.0"/>

    <!-- sim + ros_control + odom + lidar (no SLAM) -->
    <include file="$(find car3_control)/launch/car3_bringup.launch"/>

    <node name="map_server" pkg="map_server" type="map_server"
          args="$(arg map_file)"/>

    <node name="amcl" pkg="amcl" type="amcl">
        <rosparam file="$(find car3_control)/params/amcl_params.yaml" command="load"/>
        <param name="initial_pose_x" value="$(arg init_x)"/>
        <param name="initial_pose_y" value="$(arg init_y)"/>
        <param name="initial_pose_a" value="$(arg init_a)"/>
    </node>

    <node name="move_base" pkg="move_base" type="move_base" output="screen">
        <rosparam file="$(find car3_control)/params/costmap_common_params.yaml"
                  command="load" ns="global_costmap"/>
        <rosparam file="$(find car3_control)/params/costmap_common_params.yaml"
                  command="load" ns="local_costmap"/>
        <rosparam file="$(find car3_control)/params/global_costmap_params.yaml"
                  command="load"/>
        <rosparam file="$(find car3_control)/params/local_costmap_params.yaml"
                  command="load"/>
        <rosparam file="$(find car3_control)/params/dwa_local_planner_params.yaml"
                  command="load"/>
        <rosparam file="$(find car3_control)/params/move_base_params.yaml"
                  command="load"/>
    </node>
</launch>
```

### 5.9 car3_control/params/*.yaml（导航参数，验证后最终值）

**amcl_params.yaml**

```yaml
use_map_topic: false

odom_model_type: omni
odom_alpha1: 0.05
odom_alpha2: 0.05
odom_alpha3: 0.05
odom_alpha4: 0.05
odom_alpha5: 0.05

odom_frame_id: odom
base_frame_id: base_footprint
global_frame_id: map

laser_model_type: likelihood_field
laser_min_range: 0.1
laser_max_range: 10.0
laser_max_beams: 120
laser_z_hit: 0.95
laser_z_short: 0.05
laser_z_max: 0.05
laser_z_rand: 0.05
laser_sigma_hit: 0.2
laser_lambda_short: 0.1
laser_likelihood_max_dist: 2.0

min_particles: 200
max_particles: 1000
kld_err: 0.05
kld_z: 0.99
update_min_d: 0.05
update_min_a: 0.1
resample_interval: 1
transform_tolerance: 0.5
recovery_alpha_slow: 0.0
recovery_alpha_fast: 0.0

gui_publish_rate: 10.0
save_pose_rate: 0.5
```

**costmap_common_params.yaml**（global/local 共用）

```yaml
# car3 body mesh: x[-0.164, 0.189] y[-0.119, 0.119]; wheels reach y +-0.155
footprint: [[0.20, 0.17], [0.20, -0.17], [-0.20, -0.17], [-0.20, 0.17]]
robot_base_frame: base_footprint
transform_tolerance: 0.5

update_frequency: 5.0
publish_frequency: 2.0

obstacle_layer:
  enabled: true
  observation_sources: laser
  laser:
    data_type: LaserScan
    topic: /scan
    marking: true
    clearing: true
    obstacle_range: 2.5
    raytrace_range: 3.0
    inf_is_valid: false

inflation_layer:
  enabled: true
  inflation_radius: 0.25
  cost_scaling_factor: 5.0
```

**global_costmap_params.yaml**

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  update_frequency: 2.0
  publish_frequency: 1.0
  static_map: true
  rolling_window: false
  resolution: 0.05
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**local_costmap_params.yaml**

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 4.0
  height: 4.0
  resolution: 0.05
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**dwa_local_planner_params.yaml**（全向 DWA，含本报告第 12 节的关键修复）

```yaml
# Phase 5: omnidirectional DWA (mecanum needs linear.y)
DWAPlannerROS:
  max_vel_x: 0.30
  min_vel_x: -0.20
  max_vel_y: 0.30
  min_vel_y: -0.30
  max_vel_trans: 0.32
  min_vel_trans: 0.05
  max_vel_theta: 0.60
  min_vel_theta: 0.10

  acc_lim_x: 0.6
  acc_lim_y: 0.6
  acc_lim_theta: 2.0

  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.10
  latch_xy_goal_tolerance: false

  sim_time: 1.7
  sim_granularity: 0.05
  vx_samples: 10
  vy_samples: 8
  vth_samples: 21

  use_dwa: false
  oscillation_reset_dist: 0.2

  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  occdist_scale: 0.03
  forward_point_distance: 0.325
  stop_time_buffer: 0.3
  scaling_speed: 0.25
  max_scaling_factor: 0.2

  holonomic_robot: true
  y_vels: [-0.30, -0.15, 0.0, 0.15, 0.30]

  publish_traj_pc: true
  publish_cost_grid_pc: true
```

**move_base_params.yaml**

```yaml
shutdown_costmaps: false

controller_frequency: 12.0
planner_frequency: 1.0
controller_patience: 5.0
planner_patience: 3.0
conservative_reset_dist: 3.0

recovery_behavior_enabled: true
clearing_rotation_allowed: true
oscillation_timeout: 10.0
oscillation_distance: 0.3

base_global_planner: "global_planner/GlobalPlanner"
base_local_planner: "dwa_local_planner/DWAPlannerROS"
max_planning_retries: 1
```

### 5.10 car3_control/src/mecanum_controller_node.cpp

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// car3 X-type mecanum inverse kinematics.
// Geometry (measured from car3.urdf / STL):
//   wheel radius r = 0.0489 m
//   L = 0.0973 m (x half-distance), W = 0.1058 m (y half-distance)
// ROS convention: +x forward, +y left, +z up.
//
// Theoretical starting formulas (signs validated by motion tests):
//   w_fl = (vx - vy - (L+W)*wz) / r
//   w_fr = (vx + vy + (L+W)*wz) / r
//   w_rl = (vx + vy - (L+W)*wz) / r
//   w_rr = (vx - vy + (L+W)*wz) / r
// Each wheel has a sign parameter to correct joint-axis direction
// mismatches found during motion tests (defaults 1.0).

class MecanumController
{
public:
  MecanumController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("wheel_radius", r_, 0.0489);
    pnh.param("wheel_base_x_half", L_, 0.0973);
    pnh.param("wheel_base_y_half", W_, 0.1058);
    pnh.param("sign_fl", s_[0], 1.0);
    pnh.param("sign_fr", s_[1], 1.0);
    pnh.param("sign_rl", s_[2], 1.0);
    pnh.param("sign_rr", s_[3], 1.0);

    pubs_[0] = nh.advertise<std_msgs::Float64>("/wheel_lf_velocity_controller/command", 10);
    pubs_[1] = nh.advertise<std_msgs::Float64>("/wheel_rf_velocity_controller/command", 10);
    pubs_[2] = nh.advertise<std_msgs::Float64>("/wheel_lb_velocity_controller/command", 10);
    pubs_[3] = nh.advertise<std_msgs::Float64>("/wheel_rb_velocity_controller/command", 10);

    sub_ = nh.subscribe("/cmd_vel", 10, &MecanumController::cmdVelCallback, this);

    ROS_INFO("mecanum_controller: r=%.4f L=%.4f W=%.4f signs=[%.0f %.0f %.0f %.0f]",
             r_, L_, W_, s_[0], s_[1], s_[2], s_[3]);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    // X-type mecanum: FL, FR, RL, RR
    double w[4];
    w[0] = s_[0] * (vx - vy - (L_ + W_) * wz) / r_;
    w[1] = s_[1] * (vx + vy + (L_ + W_) * wz) / r_;
    w[2] = s_[2] * (vx + vy - (L_ + W_) * wz) / r_;
    w[3] = s_[3] * (vx - vy + (L_ + W_) * wz) / r_;

    for (int i = 0; i < 4; ++i)
    {
      std_msgs::Float64 cmd;
      cmd.data = w[i];
      pubs_[i].publish(cmd);
    }
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pubs_[4];
  double r_, L_, W_;
  double s_[4];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mecanum_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  MecanumController controller(nh, pnh);
  ros::spin();
  return 0;
}
```

### 5.11 car3_control/src/ground_truth_odom_node.cpp

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

// Phase 3: publish /odom + odom->base_footprint TF from Gazebo ground truth
// (task requires ground-truth odometry, not wheel odometry).

class GroundTruthOdom
{
public:
  GroundTruthOdom()
  {
    sub_ = nh_.subscribe("/gazebo/model_states", 10, &GroundTruthOdom::cb, this);
    pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &GroundTruthOdom::publish, this);
  }

  void cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    int i = -1;
    for (size_t k = 0; k < msg->name.size(); ++k)
      if (msg->name[k] == "car3") { i = static_cast<int>(k); break; }
    if (i < 0) return;
    latest_stamp_ = ros::Time::now();
    latest_pose_ = msg->pose[i];
    latest_twist_ = msg->twist[i];
  }

  void publish(const ros::TimerEvent&)
  {
    if (latest_stamp_.isZero()) return;

    nav_msgs::Odometry odom;
    odom.header.stamp = latest_stamp_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = latest_pose_.position.x;
    odom.pose.pose.position.y = latest_pose_.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = latest_pose_.orientation;
    odom.twist.twist.linear.x = latest_twist_.linear.x;
    odom.twist.twist.linear.y = latest_twist_.linear.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = latest_twist_.angular.z;
    const double cv = 1e-4;
    for (int k = 0; k < 36; ++k) odom.pose.covariance[k] = 0.0;
    for (int k = 0; k < 36; ++k) odom.twist.covariance[k] = 0.0;
    for (int k = 0; k < 6; ++k) odom.pose.covariance[k * 7] = cv;
    for (int k = 0; k < 6; ++k) odom.twist.covariance[k * 7] = cv;
    pub_.publish(odom);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = latest_pose_.position.x;
    tf.transform.translation.y = latest_pose_.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = latest_pose_.orientation;
    tf_br_.sendTransform(tf);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_br_;
  ros::Time latest_stamp_;
  geometry_msgs::Pose latest_pose_;
  geometry_msgs::Twist latest_twist_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_truth_odom");
  GroundTruthOdom node;
  ros::spin();
  return 0;
}
```

### 5.12 car3_control/src/car3_mecanum_grip_plugin.cpp

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <string>

// car3 X-type mecanum roller-grip model plugin.
//
// ODE surface friction cannot model mecanum rollers: fdir1 is body-fixed
// to the spinning wheel collision, so the grip direction rotates with the
// wheel and its anisotropy time-averages out. This plugin implements the
// physically correct roller grip instead: the grip direction of the roller
// currently in contact depends only on the BASE pose (constant in the wheel
// frame), not on the wheel spin angle.
//
// Per wheel: contact slip  s = v_c + w_wheel x r_contact
//   grip direction u in base frame: LF/RB (x-y)/sqrt2, RF/LB (x+y)/sqrt2
//   grip force F = -clamp(c * (s.u), mu*N) * u   applied at the contact point
// Wheel collision surfaces must have mu=mu2=0 so ODE adds no other in-plane
// force; only the normal support remains.

namespace gazebo {

class Car3MecanumGrip : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    r_ = 0.0489;
    muN_ = 52.0;
    c_ = 2000.0;

    if (sdf->HasElement("wheel_radius")) r_ = sdf->Get<double>("wheel_radius");
    if (sdf->HasElement("grip_force_max")) muN_ = sdf->Get<double>("grip_force_max");
    if (sdf->HasElement("grip_gain")) c_ = sdf->Get<double>("grip_gain");

    // grip direction in base frame (X-type mecanum)
    struct Wheel { std::string link; std::string joint; ignition::math::Vector3d grip; };
    std::vector<Wheel> cfg = {
      {"wheel_lf_link", "wheel_lf_joint", { 0.7071, -0.7071, 0.0}},
      {"wheel_rf_link", "wheel_rf_joint", { 0.7071,  0.7071, 0.0}},
      {"wheel_lb_link", "wheel_lb_joint", { 0.7071,  0.7071, 0.0}},
      {"wheel_rb_link", "wheel_rb_joint", { 0.7071, -0.7071, 0.0}},
    };

    for (const auto& w : cfg)
    {
      physics::LinkPtr link = model->GetLink(w.link);
      physics::JointPtr joint = model->GetJoint(w.joint);
      if (!link || !joint)
      {
        gzerr << "car3_mecanum_grip: missing link/joint " << w.link << " " << w.joint << "\n";
        return;
      }
      links_.push_back(link);
      joints_.push_back(joint);
      grip_.push_back(w.grip);
    }

    // the wheel joints' parent is the (possibly fixed-joint-lumped) base link
    baseLink_ = joints_[0]->GetParent();
    if (!baseLink_)
    {
      gzerr << "car3_mecanum_grip: wheel joint has no parent link\n";
      return;
    }

    updateConn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Car3MecanumGrip::OnUpdate, this));
    gzmsg << "car3_mecanum_grip: loaded on [" << model->GetName() << "], r=" << r_
          << " muN=" << muN_ << " gain=" << c_ << "\n";
  }

  void OnUpdate()
  {
    const auto pB = baseLink_->WorldPose().Pos();
    const auto R = baseLink_->WorldPose().Rot();
    const ignition::math::Vector3d vB = baseLink_->WorldLinearVel();
    const ignition::math::Vector3d wB = baseLink_->WorldAngularVel();
    const ignition::math::Vector3d xAxis = R.RotateVector({1.0, 0.0, 0.0});

    for (size_t i = 0; i < links_.size(); ++i)
    {
      const double omega = joints_[i]->GetVelocity(0);

      // wheel center (wheel link origin) and bottom contact point
      const ignition::math::Vector3d pW = links_[i]->WorldPose().Pos();
      const ignition::math::Vector3d cW = pW + R.RotateVector({0.0, 0.0, -r_});
      if (cW.Z() > 0.05)
        continue;  // wheel off the ground

      // velocity of the body at the wheel center + spin surface velocity
      const ignition::math::Vector3d vC = vB + wB.Cross(pW - pB);
      const ignition::math::Vector3d slip = vC - omega * r_ * xAxis;

      const ignition::math::Vector3d u = R.RotateVector(grip_[i]);
      const double su = slip.Dot(u);
      const double fmag = ignition::math::clamp(c_ * su, -muN_, muN_);
      const ignition::math::Vector3d F = -fmag * u;

      // applied to the base at the wheel center: the grip force is
      // transmitted through the axle; the moment (0,0,-r) x F is absorbed
      // by the wheel's own rotation, and the yaw torque is unchanged
      baseLink_->AddForceAtWorldPosition(F, pW);
    }
  }

private:
  physics::ModelPtr model_;
  physics::LinkPtr baseLink_;
  std::vector<physics::LinkPtr> links_;
  std::vector<physics::JointPtr> joints_;
  std::vector<ignition::math::Vector3d> grip_;
  event::ConnectionPtr updateConn_;
  double r_, muN_, c_;
};

GZ_REGISTER_MODEL_PLUGIN(Car3MecanumGrip)
}  // namespace gazebo
```

### 5.13 car3.urdf（改造要点，位于 XTDrone 包内）

完整文件 386 行，关键内容（其余为各 link 的惯量/visual/collision 与 IMU 传感器）：

```xml
<robot name="car3">
  <!-- base_footprint (0,0,0) --fixed(0,0,0.0195)--> base_link (20 kg) -->

  <!-- 4 个麦轮：continuous 关节 + SimpleTransmission（原为 fixed，无 transmission） -->
  <link name="wheel_lf_link"> ... cylinder r=0.0489 len=0.052 ... </link>
  <joint name="wheel_lf_joint" type="continuous">
    <origin xyz="0.0973 0.105835 0.0195" rpy="0 0 0"/>
    <parent link="base_link"/><child link="wheel_lf_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="20"/>
  </joint>
  <transmission name="wheel_lf_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_lf_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="wheel_lf_motor">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <!-- wheel_rf (0.0973,-0.1058)、wheel_lb (-0.0973,0.1058)、wheel_rb (-0.0973,-0.1058) 同理 -->

  <!-- 激光：laser_link 位于 base_link (0.14621, 0, 0.0874) -->
  <gazebo reference="laser_link">
    <sensor type="ray" name="head_hokuyo_sensor">
      <update_rate>15</update_rate>
      <ray><scan><horizontal>
        <samples>720</samples><min_angle>-1.57</min_angle><max_angle>1.57</max_angle>
      </horizontal></scan>
      <range><min>0.02</min><max>10.0</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><stddev>0.01</stddev></noise></ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName><frameName>laser_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- 轮子碰撞面摩擦归零：法向支撑保留，切向力全部由 grip 插件提供 -->
  <gazebo reference="wheel_lf_link">
    <mu1>0.0</mu1><mu2>0.0</mu2><kp>1e7</kp><kd>1e4</kd>
    <minDepth>0.000001</minDepth><selfCollide>true</selfCollide>
  </gazebo>   <!-- 其余三轮同理 -->

  <!-- 原 planar_move 插件已移除，替换为： -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin name="car3_mecanum_grip" filename="libcar3_mecanum_grip_plugin.so">
      <wheel_radius>0.0489</wheel_radius>
      <grip_force_max>52.0</grip_force_max>
      <grip_gain>1500.0</grip_gain>
    </plugin>
  </gazebo>
</robot>
```

### 5.14 地图 basic_room.yaml

```yaml
image: /workspace/swarm_defense_ws/src/car3_control/maps/basic_room.pgm
resolution: 0.050000
origin: [-15.200000, -17.800000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

地图：704×704 像素 @ 0.05 m/px（覆盖 35.2 m × 35.2 m），自由空间 91916 格，占用 1983 格。

### 5.15 basic_room_sim（世界）

**worlds/basic_room.world**：directional sun + 60×60 地面（mu=1）+ `<include>model://basic_room</include>`。
**models/basic_room/model.sdf**（152 行，static 模型，5 墙 + 1 柱，模型整体位姿 (0.82,-0.013) 被 world 中 include 位姿 (0,0,0) 覆盖）：

| 构件 | 尺寸 (m) | 模型内位姿 (x, y, yaw) |
|---|---|---|
| Wall_1（柱） | 0.15³ × 2.5 高 | (10.7987, 1.7924, 0) |
| Wall_6（北墙） | 20 × 0.15 × 2.5 | (-0.8737, 6.3, 0) |
| Wall_10（西墙） | 12.75 × 0.15 × 2.5 | (-10.7987, 0, -π/2) |
| Wall_12（南墙） | 20 × 0.15 × 2.5 | (-0.8737, -6.3, 0) |
| Wall_13（东墙） | 12.75 × 0.15 × 2.5 | (9.0513, 0, +π/2) |

**launch/basic_room.launch**：设置 `GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH/GAZEBO_MODEL_DATABASE_URI=""`，包含 gazebo_ros empty_world.launch（world_name 指向 basic_room.world）。

---

## 6. Topic 表

| Topic | 类型 | 方向/说明 |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | move_base(DWA) → mecanum_controller_node（建图阶段由脚本/键盘发布） |
| `/wheel_{lf,rf,lb,rb}_velocity_controller/command` | std_msgs/Float64 | mecanum IK 输出 → ros_control（rad/s） |
| `/wheel_{lf,rf,lb,rb}_velocity_controller/state` | control_msgs/JointControllerState | ros_control 状态回读 |
| `/joint_states` | sensor_msgs/JointState | joint_state_controller @50 Hz → robot_state_publisher |
| `/odom` | nav_msgs/Odometry | ground_truth_odom @50 Hz（Gazebo ground truth） |
| `/scan` | sensor_msgs/LaserScan | 720 束 ±90° @15 Hz，frame laser_link，最大 10 m |
| `/imu` | sensor_msgs/Imu | IMU @100 Hz（本链路未使用） |
| `/gazebo/model_states` | gazebo_msgs/ModelStates | Gazebo → ground_truth_odom |
| `/map` | nav_msgs/OccupancyGrid | map_server → AMCL/global_costmap |
| `/map_metadata` | nav_msgs/MapMetaData | map_server |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL 定位结果 |
| `/particlecloud` | geometry_msgs/PoseArray | AMCL 粒子（rviz 可视化） |
| `/move_base/goal` | move_base_msgs/MoveBaseActionGoal | 目标（action） |
| `/move_base/status` / `/result` / `/feedback` | actionlib | move_base 状态 |
| `/move_base/global_plan` | nav_msgs/Path | GlobalPlanner @1 Hz |
| `/move_base/DWAPlannerROS/global_plan` | nav_msgs/Path | DWA 使用的裁剪后全局路径 |
| `/move_base/DWAPlannerROS/local_plan` | nav_msgs/Path | DWA 最优局部轨迹 |
| `/move_base/DWAPlannerROS/trajectory_cloud` | sensor_msgs/PointCloud2 | 全部合法采样轨迹（x,y,theta,cost，odom 系，调试） |
| `/move_base/DWAPlannerROS/cost_cloud` | sensor_msgs/PointCloud2 | 代价场（path/goal/occ/total，调试） |
| `/move_base/global_costmap/costmap` 及 updates | nav_msgs/OccupancyGrid | 全局代价地图 |
| `/move_base/local_costmap/costmap` 及 updates | nav_msgs/OccupancyGrid | 局部代价地图（4×4 m 滚动窗口） |

---

## 7. TF 树

```
map ──(AMCL 估计 map→odom，omni 模型)──► odom ──(ground_truth_odom @50 Hz)──► base_footprint
                                                                                  │ (fixed, z=0.0195)
                                                                                  ▼
                                                                               base_link ──┬──(continuous y 轴)──► wheel_lf/rf/lb/rb_link
                                                                                            ├──(fixed)──► laser_link
                                                                                            └──(fixed)──► imu_link
```

- `map→odom`：AMCL 持续修正（初始 ≈ (2.544, -0.010, 0) 纯平移 —— 地图原点位于世界 (2.544,-0.010)）。
- `odom→base_footprint`：地面真值，与 Gazebo 世界位姿一致。
- 验证：TF 树完整、无断链，AMCL 定位误差 ≈ 0.03 m；地图与真实墙对齐误差 < 0.02 m。

---

## 8. 启动命令

```bash
# 0) 进入容器环境（每次 exec 都需要）
DOCKER_CONTEXT=default docker exec -it xtdrone-dev bash
source /home/dev/car3_env.sh

# 1) 编译
cd /workspace/swarm_defense_ws && catkin_make
source devel/setup.bash

# 2) 建图（Phase 4：bringup + gmapping）
roslaunch car3_control car3_slam.launch
#    另一终端控制小车漫游（发布 /cmd_vel，或用键盘遥控）
#    满意后保存地图：
rosrun map_server map_saver -f /workspace/swarm_defense_ws/src/car3_control/maps/basic_room

# 3) 导航（Phase 5：map_server + AMCL + move_base）
roslaunch car3_control car3_nav.launch
#    （AMCL 初始位姿默认 (2.544, -0.010, 0) = 地图系生成点；
#      若生成位置不同，传 init_x/init_y/init_a 参数覆盖）

# 4) 发送目标
python3 nav_goal.py '0.0,-1.5,0' '-1.0,-0.5,0' '-2.5,-1.0,0' '-3.0,-2.5,0' '0.0,-0.5,0'

# 5) 验证全向（观察 linear.y 使用情况）
python3 cmdvel_probe.py _duration:=300
```

---

## 9. 阶段验证记录（任务章节 49 状态清单）

| # | 检查项 | 结果 |
|---|---|---|
| [1/14] | Docker 环境检查（容器/ROS/Gazebo/显示） | **PASS** |
| [2/14] | basic_room 世界加载 | **PASS** |
| [3/14] | ros_control 五控制器（joint_state + 4×JointVelocity） | **PASS** |
| [4/14] | mecanum 插件与六项运动测试（前/后/左/右/左转/右转） | **PASS** |
| [5/14] | ground_truth_odom（/odom + odom→base_footprint TF） | **PASS** |
| [6/14] | lidar /scan（无自碰撞，720 束数据有效） | **PASS** |
| [7/14] | SLAM 建图 + 地图保存（basic_room.pgm/.yaml，对齐 <0.02 m） | **PASS** |
| [8/14] | AMCL + move_base 启动（TF 树完整、定位 ≈0.03 m） | **PASS** |
| [9/14] | 全局路径规划（global_plan 60–70 点，终点正确） | **PASS** |
| [10/14] | 局部规划 DWA（全向 linear.y，轨迹合法） | **PASS** |
| [11/14] | 单目标导航 | **PASS** |
| [12/14] | 多目标连续导航（5/5 到达） | **PASS** |
| [13/14] | 全向验证（cmd_vel 大量非零 linear.y，max=0.30 m/s） | **PASS** |
| [14/14] | 全链路开发报告（本文件） | **PASS** |

---

## 10. 导航测试结果

**多目标测试（map 系目标，90 s/目标超时）**：

```
GOAL 0 ((0.0, -1.5, 0.0)):  state=3 SUCCESS
GOAL 1 ((-1.0, -0.5, 0.0)): state=3 SUCCESS
GOAL 2 ((-2.5, -1.0, 0.0)): state=3 SUCCESS
GOAL 3 ((-3.0, -2.5, 0.0)): state=3 SUCCESS
GOAL 4 ((0.0, -0.5, 0.0)):  state=3 SUCCESS
RESULT: 5/5 goals reached
```

**全向（linear.y）证据**（cmd_vel 探针，导航期间统计）：

```
samples=1613  vy_nonzero=908 (56%)  max|vy|=0.300 m/s
典型样本：(0.300, 0.043, 0.420)  (0.288, 0.191, -0.060)  (0.300, -0.043, 0.000)
```

DWA 全程使用全速横移/斜向运动 —— 麦轮全向底盘能力在导航中实际生效。

---

## 11. 关键问题与解决（DWA 退化根因分析）

**症状**：导航 0/5 目标到达 —— 机器人初始以 0.2 m/s 移动，约 1–2 分钟内退化为
±0.01 m/s 爬行、速度符号每 ~2 s 翻转、周期“DWA failed to produce path” → 旋转恢复 →
振荡超时中止（state=4）。

**定位过程**（轨迹云 + /rosout 调试日志 + DWA 源码阅读）：
1. 代价地图清洁（0 致命格）、目标代价场正确、全局路径正确 —— 排除感知/规划层；
2. 轨迹云显示所有被丢弃轨迹的丢弃原因 100% 来自 **振荡代价函数**（critic 0，
   “discarded by cost function 0 with cost: -5.0”，8 种速度符号组合全部大量被拒）；
3. 源码确认（noetic navigation 1.17.3）：
   - `use_dwa=true` 时采样窗口 = 当前速度 ± acc×sim_period = **±0.05 m/s/周期**，
     轨迹终点差异仅 ±0.17 m → 目标代价差 ≈ 8 个代价单位 —— 与噪声同量级；
   - 胜出速度符号被噪声翻转 → 振荡标志置位（`|vx|≤min_vel_trans` 时横移/旋转标志
     全部参与跟踪）→ 方向被禁 → 再翻转 → 全部方向被禁 → “failed to produce path”；
   - `vth_samples=20`（偶数）**采样集不含 vth=0** → “直行”根本不在候选集中，
     旋转方向纯靠噪声决胜 → 旋转标志持续翻转，速度斜坡永远无法建立；
   - 速度上不去 → 终点差异小 → 代价面平坦 → 陷阱自维持。

**修复**（dwa_local_planner_params.yaml，3 处）：

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `use_dwa` | true（默认） | **false** | 每个周期在全速度空间采样（终点差异 ±0.5 m，目标吸引决定性） |
| `vth_samples` | 20 | **21** | 奇数采样数包含 vth=0，直行进入候选集，消除旋转方向噪声决胜 |
| `oscillation_reset_dist` | 0.05 | **0.2** | 振荡标志在行驶 0.2 m 后复位，快速解除方向锁定 |

修复后：5/5 目标到达（平均 ~29 s/目标），全向横移正常使用（max|vy|=0.30 m/s）。

**其他修复记录**：
- Noetic 中 `max_rot_vel/min_rot_vel` 已废弃（“will not load properly”）→ 改用 `max_vel_theta/min_vel_theta`；
- `publish_traj_pc/publish_cost_grid_pc` 为构造时参数，dynamic_reconfigure 无法设置 → 必须写入 yaml 后重启；
- 轮子摩擦 mu=0 与 grip_gain 1500（5200 时离散不稳定）；
- gazebo_pid.yaml 必须先于 spawn 加载（gazebo_ros_control 初始化时读取）。

---

## 12. 运行注意事项

- 容器内 PID 1 不回收僵尸进程（defunct 无害）；`pkill` 注意避免匹配自身命令行；
- 纯 gzserver 启动会卡死 —— 必须经 gazebo_ros wrapper（roslaunch）启动；
- 每次 `docker exec` 需 `source /home/dev/car3_env.sh`；
- 调试日志：`rosservice call /move_base/set_logger_level 'ros.base_local_planner' 'debug'`，
  丢弃轨迹逐条打印（“discarded by cost function N”）。

---

*报告完。单车范围，未扩展任何多车/协同/无人机功能。*
