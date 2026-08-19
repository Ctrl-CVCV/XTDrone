# car3_swarm 开发文档：三车入侵与围捕仿真（第一版）

## 1. 目标

在 `nesting_room`（内房间 5.95m 见方 + 四扇 1.2m 门 + 14m 方形外房间）中部署 **三辆完全同构的 car3**，
模拟"入侵车 + 两辆围捕车"场景。多车仿真独立于单 car3 仿真，放在本包 `car3_swarm` 下，
**不改动单 car3 仿真（car3_control）的既有行为**（只对节点做向后兼容的 ns 化改造，见 §4）。

需求映射：

| 需求 | 方案 | 落位 |
|---|---|---|
| 1.1 新工程包 | 新包 `car3_swarm`（launch/src/scripts/params） | 本包 |
| 1.2 三车同构 | 同一 URDF（xacro 参数化前缀）→ 三份 robot_description；同一套控制器/nav 参数 | §4 |
| 1.3 固定出生位姿 | launch 参数默认值固定（每次运行一致，除非用户改参数） | §5 |
| 1.4 话题/服务严格区分 | 每车 ROS 命名空间 `car0/car1/car2` + TF 帧前缀；接口全表见 §6 | §6 |
| 1.5 两版六脚本 | V1 键盘 ×3、V2 多点导航 ×3（actionlib 等结果，防目标覆盖） | §7 |
| 1.6 位置共享 | `car_state_broadcaster` 读 gazebo 真值 → 每车 `/carN/shared_pose`（Odometry） | §8 |
| 1.7 虚拟障碍物 | 每车一个 `virtual_obstacle_node`：把其他车的虚拟长方体注入本车雷达扫描（选型理由见 §9） | §9 |

## 2. 系统架构

```
gazebo (nesting_room.world, 只启动一次)
 ├─ car0 (围捕车, NE 内墙角)  ├─ car1 (围捕车, SW 内墙角)  ├─ car2 (入侵车, NW 外墙角)
 └─ 公共: map_server (/map, 唯一一份)

每车命名空间 /carN 内（N=0,1,2）:
  robot_description          (xacro: 帧名 carN/xxx + 插件 ns carN)
  spawn_model                (模型名 car0/car1/car2)
  gazebo_ros_control         (robotNamespace=carN, PID 参数 /carN/gazebo_ros_control/pid_gains)
  controller_spawner         (4 轮速度控制器 + joint_state, /carN/ 下)
  mecanum_controller_node    (sub carN/cmd_vel → pub carN/wheel_*_velocity_controller/command)
  ground_truth_odom_node     (pub carN/odom + TF carN/odom→carN/base_footprint, 真值)
  robot_state_publisher      (静态 TF: base_footprint→base_link→laser_link/imu_link)
  amcl                       (定位, 用真扫描 carN/scan, 初始位姿=出生点, 发布 map→carN/odom)
  nav_to_pose_node           (三阶段导航, 用注入后扫描 carN/scan_filtered)
  virtual_obstacle_node      (1.7: 真扫描 + 他车 shared_pose → 注入后扫描)

全局:
  map_server                 (/map, /map_metadata, /static_map — 一份, 所有车共享)
  car_state_broadcaster      (1.6: /gazebo/model_states → /carN/shared_pose)
```

TF 树（三车互不干扰，共用根帧 map）：

```
map ─┬─ car0/odom ─ car0/base_footprint ─ car0/laser_link (静态)
     ├─ car1/odom ─ car1/base_footprint ─ car1/laser_link
     └─ car2/odom ─ car2/base_footprint ─ car2/laser_link
```

启动文件：

- `launch/multi_car3_bringup.launch` — 世界 + 三车（无导航），供 V1 键盘脚本使用
- `launch/multi_car3_nav.launch` — bringup + map_server + 3×AMCL + 3×nav_to_pose + 位置共享 + 虚拟障碍物，供 V2 导航脚本使用

## 3. 关键技术前提（已核查）

- **map 帧 == 世界帧**：nesting_room 房间中心即世界原点，地图无帧平移（与 6_6_room 相同结论），
  AMCL 初始位姿 = gazebo 出生坐标，直接可用。
- **lidar 高度 ~0.29m**（base_footprint→base_link +0.0195 → laser_link +0.27），
  高于车体（~0.15m）——所以真雷达扫不到其他车，这正是 1.7 要弥补的缺陷。
- **lidar 无横向偏置**：laser_link 在车体中心正上方，nav_to_pose 直接把 scan 角度当车体系角度使用，
  nav 不依赖 TF，多车改造时只需换话题名。
- **odom 帧 == 世界帧**：ground_truth_odom 直接发布 gazebo 世界位姿（标为 odom 帧），
  对 AMCL（用 TF）与 nav（只用增量）均无影响。

## 4. 单车节点 ns 化改造（向后兼容）

三个 car3_control 节点全部改为**相对话题名**（ns 下自动隔离，单车主 launch 无 ns 时行为与原来完全相同）：

| 节点 | 改动 |
|---|---|
| `mecanum_controller_node` | 话题改相对名：`/cmd_vel`→`cmd_vel`，`/wheel_*_velocity_controller/command`→相对 |
| `ground_truth_odom_node` | 话题 `odom` 相对；新增参数 `model_name`(默认 car3)、`odom_frame`(默认 odom)、`base_frame`(默认 base_footprint) |
| `nav_to_pose_node` | 话题改相对名：`cmd_vel`/`amcl_pose`/`odom`/`move_base_simple/goal`；`scan_topic` 默认改 `scan`（params yaml 同步） |

URDF：新建 `car3_control/urdf/car3.xacro`（内容与 `car3.urdf` 完全一致，仅两处参数化）：

- `prefix` 参数 → 所有 link 名加前缀（帧隔离）；joint 名保持原名（每车模型内部，无需前缀）
- `ns` 参数 → 各插件 `<robotNamespace>`、lidar `<topicName>scan</topicName>`（相对）、imu `<topicName>imu</topicName>`
- 原 `car3.urdf` 不动，单 car3 仿真零影响

## 5. 出生位姿（需求 1.3，固定默认值）

| 车 | 角色 | 位置 (x,y) | 朝向 yaw | 含义 |
|---|---|---|---|---|
| car0 | 围捕车 | (2.3, 2.4) | -2.334 rad | 内房间 NE 墙角，车头对准房间中心 |
| car1 | 围捕车 | (-2.3, -2.4) | 0.808 rad | 内房间 SW 墙角（与 car0 同一对角线），车头对准房间中心 |
| car2 | 入侵车 | (-6.5, 6.6) | 0.0 rad | 外房间 NW 墙角，车头平行于北外墙（朝 +x） |

- 位置均避开墙体（离墙 ≥0.38m）与地图占用格，z=0.01 与单车一致。
- 以 launch 参数 `carN_x/carN_y/carN_yaw` 形式给出默认值 → **每次运行一致**，需要改形态时只改参数。

## 6. 接口全表（需求 1.4）

每车命名空间 `/carN`（N=0,1,2），公共资源不带前缀：

| 接口 | 类型 | 归属 | 说明 |
|---|---|---|---|
| `/carN/cmd_vel` | Twist | 订阅 | 运动指令（键盘/导航共用） |
| `/carN/odom` | Odometry | 发布 | gazebo 真值里程计（ground_truth_odom） |
| `/carN/scan` | LaserScan | 发布 | 真雷达（1440 点 360°，20m，帧 `carN/laser_link`） |
| `/carN/scan_filtered` | LaserScan | 发布 | 注入虚拟障碍物后的雷达（供 nav_to_pose） |
| `/carN/amcl_pose` | PoseWithCovarianceStamped | 发布 | AMCL 定位（map 帧） |
| `/carN/initialpose` | PoseWithCovarianceStamped | 订阅 | AMCL 初始位姿 |
| `/carN/move_base` | MoveBaseAction | 服务端 | nav_to_pose 动作接口（V2 脚本用） |
| `/carN/move_base_simple/goal` | PoseStamped | 订阅 | RViz 2D Nav Goal 兼容入口 |
| `/carN/wheel_{lf,rf,lb,rb}_velocity_controller/command` | Float64 | 订阅 | 轮速指令（ros_control，robotNamespace=carN） |
| `/carN/joint_states` | sensor_msgs/JointState | 发布 | 关节状态 |
| `/carN/shared_pose` | Odometry | 发布 | **1.6**：位置+车头朝向+线速度+角速度（map 帧） |
| `/carN/gazebo_ros_control/pid_gains` | 参数 | — | 每车独立 PID |
| `/map` `/map_metadata` `/static_map` | — | 全局 | 唯一 map_server，全车共享 |
| `/gazebo/spawn_urdf_model` 等 | 服务 | 全局 | gazebo 公共设施 |

TF：`map → carN/odom → carN/base_footprint → carN/laser_link`（+imu_link），帧名严格前缀，无冲突。

## 7. 六脚本（需求 1.5）

公共逻辑抽到 `scripts/swarm_teleop.py` 与 `scripts/swarm_waypoints.py`，
六个入口脚本各 ≤40 行（满足"三个脚本/三个脚本"的形式要求）。

### V1 键盘控制（三个终端各跑一个）

- `keyboard_car0.py` / `keyboard_car1.py` / `keyboard_car2.py`
- 复用单车键盘逻辑（w/s 前后、a/d 横移、q/e 旋转、± 线速档、]/[ 角速档、空格停、t 帮助）
- 发布 `/carN/cmd_vel`（脚本不经 ns，直接绝对话题）

### V2 多点导航（防目标覆盖策略）

- `waypoints_car0.py` / `waypoints_car1.py` / `waypoints_car2.py`
- 每个脚本内置该车**依次**的导航点列表 (x, y, yaw)
- **防覆盖策略**：用 actionlib `SimpleActionClient(/carN/move_base)`，
  每点 `send_goal → wait_for_result(timeout)` 拿到终态（SUCCEEDED/ABORTED）后才发下一点，
  绝不同时存在两个目标；ABORTED 记日志后继续下一点；支持 Ctrl+C 安全取消
- 示例路径（围捕场景）：
  - car2 入侵：NW 外墙角 → 沿北走廊东行 → 北门南下 → 内房间中心
  - car0/car1 围捕：各自墙角 → 向房间中心收缩 → 最终面向北门堵截

## 8. 位置共享（需求 1.6）

节点 `car_state_broadcaster`（全局，一个）：

- 订阅 `/gazebo/model_states`（gazebo 真值，map 帧==世界帧）
- 对 car0/car1/car2 各发布 `/carN/shared_pose`（nav_msgs/Odometry，20Hz）：
  - `pose.pose` = 位置 + 朝向四元数（车头朝向）
  - `twist.twist` = 线速度 + 角速度（世界系）
  - header.frame_id = `map`，child_frame_id = `carN/base_footprint`
- 任何节点订阅 `/car0/shared_pose`+`/car1/shared_pose`+`/car2/shared_pose` 即可知全车状态

## 9. 虚拟障碍物（需求 1.7）——方案选型

**需求**：以每车几何中心为中心，按实际车大小模拟一个不可见虚拟长方体障碍物，
让三车雷达能"扫到"其他车并触发导航避障（弥补 lidar 高 0.29m 扫不到低车体的缺陷）。

**方案 A（gazebo 不可见碰撞盒，弃用）**：spawn 只有 collision 没有 visual 的静态盒子
（collide_bitmask 0xFFFF0000 可做到"雷达可见+无物理碰撞"），20Hz set_model_state 跟随车。
**致命缺陷：自己车的 lidar 位于自己盒子的几何中心内部**——每条激光束在 ~0.2m 处就打在本车盒子上，
本车扫描变成一个 0.2m 的障碍环，真实环境完全被遮挡（所有墙、所有其他车都看不见），本车导航报废。
这是物理本质（lidar 在盒子内部），bitmask 无法区分"自己的盒子"。

**方案 B（雷达扫描注入，采用）**：每车一个 `virtual_obstacle_node`：

- 订阅本车真扫描 `/carN/scan` + **其他两车**的 `/carM/shared_pose`
- 对每条激光束做"射线 vs 他车虚拟长方体（0.45×0.38m，即车体 0.40×0.34 加小裕量，2D 矩形）"相交计算：
  若相交且入口距离 < 原量程 → 把该束量程覆写为入口距离
- 发布 `/carN/scan_filtered`；`nav_to_pose` 的 `scan_topic` 指向它（**AMCL 仍用真扫描**，定位不受虚拟物影响）
- 效果与物理盒完全一致：他车在雷达上呈现为障碍 → 触发 nav_to_pose 的斥力/横向逃逸避障；
  且天然不存在"扫到自己盒子"问题（自己的盒子根本不注入自己的扫描）
- 本车不动时该节点透明（无他车信息时原样透传），单点失效不影响真雷达链路

**说明**：方案 B 下"足够高"天然满足（注入直接作用在 lidar 平面，等效无限高）。
如果你希望保留"gazebo 里真实存在一个物理盒"的观感（例如在 RViz/Gazebo 里调试可见），
可在 B 之上**附加**方案 A 的盒子仅作可视化/他车可见用途，但本车避障数据一律以注入扫描为准。
这是本方案唯一需要你拍板的技术取舍点，其余按上述设计实现。

## 10. 分阶段实施与验证（全部完成，2026-08-17）

1. ✅ 包骨架 + 单车节点 ns 化 + car3.xacro → `catkin build`，单 car3 原 launch 回归通过（倒车 1.06m 实测，行为不变）
2. ✅ `multi_car3_bringup.launch`：三车按固定位姿出生；`/carN/*` 话题齐全、TF 树三车隔离；
   键盘只动车 0 验证通过（car1/cmd_vel=0.5 时其余两车 1e-7 级静止）
3. ✅ `multi_car3_nav.launch`：3×AMCL 收敛到各自出生点（误差 <5cm）、3×nav_to_pose 独立导航
4. ✅ V1 三键盘脚本（终端实测由用户操作，headless 已验证话题与按键映射）
5. ✅ `car_state_broadcaster`：`/carN/shared_pose` 数值与 gazebo 真值一致、20Hz 实测
6. ✅ `virtual_obstacle_node`：注入实测——car0 扫向 car2 方向 raw 3.4m（墙）→ filtered 0.8m（虚拟盒）；
   **对驶避障实测**：car0 直行路径穿过 car2 虚拟盒 → 0.3m 处触发右向横移逃逸（航向锁定、x 向停进）
   → 绕过盒子（净距 ~0.5m）→ 恢复直行到达目标；car2 全程零位移（无物理碰撞）
7. ✅ V2 三点导航脚本：全流程两次跑通（9 个目标全部 SUCCEEDED，无覆盖）；
   入侵车穿北门入内房间，围捕车同步收缩至门内侧两翼

### 实测发现的问题与修复

| 问题 | 现象 | 修复 |
|---|---|---|
| broadcaster 段错误 | 启动 7s 后 SIGSEGV（-11）：`found_` 等向量在首个 model_states 回调前为空，定时器回调越界访问 | 构造时按车数预初始化三个向量（已修，20Hz 稳定发布） |
| 激光插件偶发静默 | 4 次启动中 1 次三车 scan 全无数据（话题已 advertise、订阅者已连接），其余 3 次正常；缺失的 "Starting Laser Plugin" 日志行是 rosout 丢行，非真因 | nav 九个节点加 launch-prefix：等本车 `/carN/scan` 出首帧再启动（90s 上限，超时回退旧行为），消除 spawn 窗口内的订阅竞争 |
| 目标落在别车排斥场内 | 目标距他车虚拟盒 <0.6m（repulse_range）时，吸引-排斥平衡形成绕目标极限环，永不 settle（20s 停转检测可中止，但不保证） | 属预期行为（保护域生效）；**规划导航点时需与他车保持 >0.6m 距离** |

## 11. 已知风险与备注

- 三份 gazebo_ros_control 的 PID 参数路径需按 `/carN/gazebo_ros_control/pid_gains` 加载（实测确认）
- AMCL 每实例需 remap `map`/`static_map` 到全局（否则会找 `/carN/map`）
- 三车无物理碰撞（虚拟盒方案 B 下），键盘手动驾驶可能视觉上"穿车"——nav 避障依赖虚拟障碍物，属预期行为
- 首次三车同时 spawn，gzserver 负载上升，建图不涉及（地图已建好，静态加载）
- 导航点与他车当前/目标位置保持 >0.6m（repulse_range），否则可能出现绕目标极限环
- "Starting Laser Plugin" 日志行可能只有 2 条（rosout 丢行），以 scan 实际有数据为准，勿据日志行数判断插件失败
- 工具脚本：`scripts/record_trajectory.py` 可按 5-20Hz 录制 `/carN/shared_pose` 轨迹（行缓冲实时落盘）
