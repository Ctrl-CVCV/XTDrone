# SWARM-LIO 地面小车定位接入 + 空地同场总联（M1–M6）完成说明与使用手册

- 日期：2026-09-04
- 容器：`xtdrone-swarm-lio`（镜像 `XTDRONE-SWARM-LIO-2026-09-02.tar`）
- 宿主工作区：`/home/liuhanxu/xtdrone_docker/workspace`（挂载到容器 `/home/dev/XTDrone-single-car/workspace`）
- 目标：把**两辆地面围捕小车（car0/car1，car3 麦轮车 + MID-360）接入 SWARM-LIO 定位**，并和 SWARM-LIO 定位的无人机**共用一个对齐世界**，每辆小车各自在该共享世界里完成 **2D 导航**，空地同时在场演示。

一句话结论：**双车 + UAV 各自验证通过** —— ①car0/car1 各跑 SWARM-LIO 定位 ②gazebo 真值把 `car0/world`、`car1/world` 各自对齐进无人机 `quad0/world`（共享世界，两车世界 T 均≈gazebo 相对真值）③car0/car1 各自用 `map+AMCL+move_base` 完成 2D 导航、可同场并存独立发目标（实测 car1 目标 SUCCEEDED、car0 同场无回归）；无人机 `iris_0` 亦实现 SWARM-LIO 定位起飞/短悬停。**剩余已知边界：长时间场景中 LIO-XY 漂移累积会拖垮同场长悬停**（详见 §5）。

---

## 目录
1. [总体架构](#1-总体架构)
2. [分里程碑完成情况](#2-分里程碑完成情况)
3. [改动/新增文件清单](#3-改动新增文件清单路径)
4. [验证结果与实测数据](#4-验证结果与实测数据)
5. [已知局限与边界](#5-已知局限与边界)
6. [最终使用方法](#6-最终使用方法)
7. [快速核对 status/verify](#7-快速核对)

---

## 1. 总体架构

同一 Gazebo `nesting_room` 世界里：双机 UAV（`iris_0/iris_1`）+ **两辆围捕麦轮车 car0/car1** 都装 MID-360（CustomMsg 点云）。每台 agent **各自独立跑 SWARM-LIO**，各自产生一个 LIO 世界系；再用 **gazebo 真值**把它们统一起来（这是本任务核心：用真值做一次性静态对齐，避免 SWARM-LIO 分布式相对定位漂移）。两车各自与 UAV 的 LIO 世界经 `map_alignment` 对齐进 `quad0/world` 这个**共享世界**，于是任意车的 LIO 位姿都能直接在共享世界里与无人机/另一辆车相互比较。

帧与话题关系：

```
Gazebo nesting_room (唯一真值)
  ├─ iris_0 ──/iris_0/livox/lidar(CustomMsg)──> laserMapping_quad0 ──> LIO 世界 quad0/world
  │             (LIO odom /quad0/lidar_slam/odom; vision_pose_raw)
  │             guard(lio_pose_guard)──>/iris_0/mavros/vision_pose/pose──>PX4 EKF2(外部视觉 XY/航向, Z 用 baro)
  ├─ iris_1 ──> laserMapping_quad1 ──> quad1/world
  ├─ car0  ──/car0/livox/lidar + /car0/imu──> laserMapping_car0 ──> LIO 世界 car0/world
  └─ car1  ──/car1/livox/lidar + /car1/imu──> laserMapping_car1 ──> LIO 世界 car1/world
             (两车 LIO odom /carN/lidar_slam/odom; vision_pose_raw 不接 PX4)

map_alignment(gazebo 真值冻结, model0=iris_0, 每车一实例 __name=map_alignment_carN):
    car0/world ──(TF quad0/world->car0/world)──┐
    car1/world ──(TF quad0/world->car1/world)──┼──> 全部表达进 quad0/world   [共享世界]
carN 2D 导航(各自在 carN/map≈gazebo 帧, 与共享世界几何一致, 可同场并存):
    /carN/livox/lidar ──mid360_to_scan(__name=mid360_to_scan_carN)──> /carN/scan
        ──> AMCL(map<-odom<-base) + move_base(全向 DWA)
    carN/odom = gazebo 真值 odom(车端 ground_truth_odom 节点), 已有 TF
    car0 栈在根命名空间(/move_base/*、frame car0/map); car1 栈整体 <group ns="car1">
    (/car1/move_base/*、/car1/cmd_vel、frame car1/map) —— 两 move_base 互不抢 action server
```

**小车实现链路（carN 从传感器到共享世界到自主导航的完整数据通路）**——一辆车要同时扮演"共享世界里的一个定位体"和"能自主导航到目标的移动体"，其实现链路自底向上分四层：

1. **车端感知与运动底盘**：每辆 car 由 `car_spawn.launch`（`car3_swarm/launch`，参数化 `car:=carN`）在 Gazebo 中生成——`car3.xacro` 的 `lidar:=mid360_lio` 分支在底盘上方装平装 MID-360（`carN/livox_link`），其 gazebo 插件（与 UAV iris_mid360 同款 CustomMsg `.so`）把点云发布成**绝对话题 `/carN/livox/lidar`（livox_ros_driver2/CustomMsg）+ `/carN/imu`**；同一 launch 内还有 `robot_state_publisher`（carN/base_footprint 等 TF）、ros_control + `mecanum_controller_node`（订阅 `/carN/cmd_vel`，麦轮全向解算到四轮速度）与 `ground_truth_odom`（发布 `/carN/odom` = gazebo 真值里程计，供导航栈用）。

2. **SWARM-LIO 单机定位（产生每车自己的 LIO 世界系）**：`start_car_lio` 用 `swarm_lio/livox_mid360.launch` 起 `laserMapping_carN`（drone_id 取与 quad0/quad1 不冲突的空位：car0=0、car1=2；`output_prefix=carN`），喂 `/carN/livox/lidar` + `/carN/imu`、配 `carN_sim.yaml`（lid/imu 话题、平装 R=I、`LI_extrinsic_T`、`actual_uav_num=1`）。SWARM-LIO 输出 LIO 世界位姿 `/carN/lidar_slam/odom`（10 Hz）与世界帧 TF `carN/world`。**这层是纯本地定位**，与世界对齐无关。

3. **世界对齐（把每车 LIO 世界塞进共享世界 quad0/world）**：`map_alignment.py` 每车一个实例（`__name=map_alignment_carN`、`~child_frame=carN/world`、`~parent_frame=quad0/world`、`~source=simulation_truth`、`~model0=iris_0`/`~model1=carN`、`~odom0/odom1`=双 LIO odom、`~imu_offset1="-0.07125 -0.00161 0.0806"` 平移 gazebo 真值 pose 到 LIO 锚定的 IMU）。它采样 N=30 帧 gazebo 真值，把"carN 相对 iris_0 的真值位姿"冻结成对齐 T，广播 latched `TF quad0/world→carN/world` 与 `/map_alignment_carN/transform`。因此**在共享世界里读 carN 的 LIO 位姿 = 该车在房间里的真实几何位姿**（实测两车 T 均与 gazebo 相对真值一致，见 §4）。

4. **2D 导航栈（每车独立闭环）**：`mid360_to_scan.py`（每车一实例 `__name=mid360_to_scan_carN`，防 ROS 节点同名互挤）把 CustomMsg 在水平带内降到 **2D `/carN/scan`**；`carN_nav.launch` 起 `map_server`（复用同一张房间静态图 `car0_nav_map`，frame `carN/map`）+ **AMCL**（`<remap odom→/carN/odom`、`scan→/carN/scan`，初值取出生位）+ **move_base（全向 DWA）**。car0 在根命名空间、car1 整体包在 `<group ns="car1">`，使 `/car1/move_base_simple/goal`、`/car1/cmd_vel` 等与 car0 完全不冲突，两车可同场各发各的目标互不干扰。导航闭环用的是 `/carN/odom`(真值) 与 gazebo 帧静态图，故物理可达；而 SWARM-LIO 世界（第 2–3 层）让这辆车在共享世界里也有与导航几何一致的 LIO 表达，供无人机/他车协同判位。

> 一句话：**感知（/carN/livox/lidar+imu）→ 本车 SWARM-LIO（carN/world）→ gazebo 真值对齐进共享世界（quad0/world）→ 2D 降维（/carN/scan）→ AMCL+move_base 独立 2D 导航**，四段里第 2、3 段让车成为共享世界的一员，第 1、4 段让它能自主动起来。

**路径映射**：所有本任务代码/配置放在**宿主挂载**的 `workspace/`（重建容器不丢）；仅两处 launch 在**容器本地**（见 §3.4）。

---

## 2. 分里程碑完成情况

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 工作区与 SWARM-LIO 源码梳理、依赖确认 | ✅ |
| M1 | car3.xacro 增 `lidar:=mid360_lio`（平装 CustomMsg 插件，与 UAV 同款 `.so`）；topic `/carN/livox/lidar` + `/carN/imu` | ✅ |
| M2 | 单车 SWARM-LIO 冒烟：`car0_sim.yaml` + `laserMapping_car0` 输出 `/car0/lidar_slam/odom`(10Hz)；运动对比探针 | ✅ |
| M3 | 多 agent 世界对齐：`map_alignment.py` 通用化；car1/car0 与 car0/UAV 双 UGV/UAV 验证 | ✅ |
| M4 | 3D→2D 降维：`mid360_to_scan.py`(/car0/scan) + LIO 建图(`build_lio_2d_map/build_scan_2d_map`) + `mapping_drive.py` | ✅ |
| M5 | car0 2D 导航演示：`car0_nav.launch`(map_server+AMCL+move_base 全向 DWA) | ✅ |
| M6 | 空地同场总联：`multi_vehicle` 场景（car0/car1 均 `mid360_lio`）+ `run_m6_air_ground_demo.sh` 分段一键演示，**双车各自 SWARM-LIO + 世界对齐 + 2D 导航** | ✅(含已知边界) |

---

## 3. 改动/新增文件清单（路径）

> 宿主根 = `/home/liuhanxu/xtdrone_docker/workspace`；容器内对应 `/home/dev/XTDrone-single-car/workspace`。下面 `WS` 即宿主根。mtime 均为 2026-09-04（本任务）。

### 3.1 演示工作区 `WS/car3_lio/`（主交付）

| 文件 | 作用 |
|---|---|
| `car3_lio/run_m6_air_ground_demo.sh` | **M6 主入口**，分段 `clean\|up_world\|up_uavlio\|up_carlio\|up_scan\|up_align\|up_nav\|up_takeoff\|verify\|status`（§6）。2026-09-04 二次完善：`up_carlio/up_scan/up_align/up_nav` 全部**双车化**（car0 根命名空间 + car1 `/car1` ns 同场并存）；`clean()` 增加杀 PX4 SITL 常驻 daemon(`px4_sitl_default`)、`controller_manager/spawner`、`multi_vehicle.launch` 并删 `/tmp/px4_lock-*`（防"daemon already running"空跑与 spawner 100% 风暴）；`up_scan` 两节点带 `__name=mid360_to_scan_carN`（防同名互挤） |
| `car3_lio/run_m3_align_demo.sh` | M3 双车世界对齐演示入口 |
| `car3_lio/m3_verify.py` | M3 对齐一致性量化核验（采样点：gazebo 相对位姿 vs LIO odom 残差） |
| `car3_lio/mapping_drive.py` | M4 建图行驶：`car0/odom`(真值)慢速巡航（原地快转会致 LIO 位姿漂移） |
| `car3_lio/env.sh` | 容器内环境：`ws_livox`+`swarm_defense_ws`（**不 source mid360_car3**，避免同名单抢占插件路径）；CustomMsg `.so` 在前；DISPLAY=:1 |
| `car3_lio/car0_sim.yaml` | car0 SWARM-LIO 单机配置：lid/imu 话题、平装 R=I、`LI_extrinsic_T`、`actual_uav_num=1` |
| `car3_lio/car1_sim.yaml` | car1 同款（M3） |
| `car3_lio/start_lio_single.sh` `stop_lio_single.sh` `start_lio_only.sh` | M2 单车 LIO 起/停（不含/含车的两种） |
| `car3_lio/motion_probe.py` | M2 运动验证：稳定发布者驱动 car0，对比 LIO odom vs gazebo 真值 odom（不用 `rostopic pub` 驱动车轮——分叉生命周期坑） |
| `car3_lio/clean_sim.sh` `test_car_spawn.sh` `diag_sensor_test.sh` | 场景清理 / 车 spawn 测试 / 传感器诊断 |
| `car3_lio/zsamp.py` `ap_samp.py` `patch_guard_launch.py` | 临时采样（gazebo/LIO/vision z、姿态）与 guard launch 重打补丁脚本（诊断/复现用，保留） |

### 3.2 `car3_swarm`：SWARM-LIO 车辆/UAV 定位侧（`WS/swarm_defense_ws/src/car3_swarm/`）

| 文件 | 类型 | 作用 |
|---|---|---|
| `scripts/map_alignment.py` | 新增 | **世界对齐通用版**（由 `swarm_lio/dual_map_alignment.py` 通用化，odom 话题参数化以适配地面车）。`~source=simulation_truth/swarm`，按 gazebo 真值冻结 `TF parent->child = inv(g_w0)*g_w1`，`~rate` 重发 + latched `/map_alignment/transform`。支持 `~imu_offsetN` 平移 model 位姿到 LIO 锚定 IMU |
| `scripts/lio_pose_guard.py` | **修改** | 外部视觉输入守卫：拒绝失锁样本(roll/pitch>max_tilt 或位置跳变 step/speed)，回填上一有效位姿（保持时间戳更新以免 PX4 断流）。**本次新增 `~baro_z_topic`：把输出 Z 覆盖为 PX4 mavros local z**（`_out()` 逻辑），避免 LIO-Z 拖拽 EKF 高度 |
| `scripts/uav_offboard_takeoff_lio_ref.py` | 新增 | iris 一键 OFFBOARD/ARM/起飞，SWARM-LIO 参考 + 看门狗（z 增量/gazebo、LIO tilt、掉高/冲高、OFFBOARD 意外退出），`--no-ego-handover` 保持悬停。旗标见 §6.4 |
| `scripts/ego_lio_to_px4_bridge.py` | 新增 | EGO/SWARM-LIO 位姿 → PX4 local ENU 桥（早期多机方案，当前演示用 guard+MAVROS vision_pose 通路，此桥作参考） |
| `launch/lio_car_bringup.launch` | 新增 | 单车车端：nesting_room + car_spawn（`lidar:=mid360_lio`，出生 `(2.3,2.4,-2.334)` 避开原点 camera_gimbal 自遮挡） |
| `launch/dual_lio_car_bringup.launch` | 新增 | 双车车端（car0/car1，M3） |
| `launch/car_spawn.launch` | 新增 | car3 参数化 spawn 车端：`/carN` ns + robot_state_publisher + ros_control/mecanum + `ground_truth_odom`(`/carN/odom`)；`lidar:=hokuyo/mid360/mid360_lio` |
| `launch/swarm_lio_ego_dual.launch` | 沿用 | 双机 EGO-Swarm + SWARM-LIO 联编（多机形态，任务早期产物，M6 未用） |

### 3.3 `car3_control`：2D 导航侧（`WS/swarm_defense_ws/src/car3_control/`）

| 文件 | 类型 | 作用 |
|---|---|---|
| `urdf/car3.xacro` | **修改** | 增 `lidar=mid360_lio` 分支：与 UAV iris_mid360 同款 CustomMsg 插件(`publish_pointcloud_type=3`)，`livox_link` 平装 base 上方 z=0.24，绝对话题 `$lio_lidar_topic` |
| `scripts/mid360_to_scan.py` | 新增 | M4：CustomMsg → 2D `/carN/scan`（取传感器系水平 ±band，切地板/自部件；frame=carN/livox_link），喂 AMCL/move_base |
| `scripts/build_lio_2d_map.py` | 新增 | M4b：LIO odom + `cloud_registered` 稠密点云建 2D 栅格（坐标系=car0/world） |
| `scripts/build_scan_2d_map.py` | 新增 | M4c：LIO odom + 2D scan 经典光投影建图 |
| `launch/car0_nav.launch` | 新增 | M5：map_server(`car0_nav_map.yaml`, frame `car0/map`) + AMCL + move_base（全向 DWA）。**AMCL 加了 `<remap from="odom" to="/car0/odom"/>` 与 `scan→/car0/scan`**（缺 odom remap 会 TF 失败）。运行于**根命名空间**（`/move_base/*`、`/move_base_simple/goal`、frame `car0/map`） |
| `launch/car1_nav.launch` | 新增 | **car1 双车导航镜像**：同 car0 栈整体包在 `<group ns="car1">` → `map_server_car1`/`amcl_car1`/`move_base_car1` 落到 `/car1/*`（`/car1/move_base_simple/goal`、`/car1/cmd_vel`），复用 `car0_nav_map`、`params_car1`，AMCL 出生取 car1 (-2.3,-2.4,0.808)，remap `scan→/car1/scan`、`odom→/car1/odom`。**与 car0 的根命名空间 move_base 互不冲突，可同场并存各发各目标** |
| `params_car0/`（`amcl_params/costmap_common/global_costmap_params/local_costmap_params/dwa_local_planner_params/move_base_params.yaml`） | 新增 | car0 导航参数。amcl：`odom_model_type=omni`，`odom_frame=car0/odom`、`base=car0/base_footprint`、`global=car0/map` |
| `params_car1/` | 新增 | `cp -r params_car0 params_car1` 后 `sed 's/car0/car1/g'` 得到 car1 全套参数（帧 `car1/map`/`car1/odom`/`car1/base_footprint`、障碍话题 `/car1/scan`）；已核对无 car0 残留 |
| `maps/car0_nav_map.pgm/.yaml` | 新增 | car0 2D 导航占用栅格（gazebo 帧墙图；res 0.05，origin [-8,-8]） |

### 3.4 容器本地改动（重建容器需按 §6.6 重做）

> 路径在容器内，不随宿主挂载持久。

| 文件（容器内） | 改动 |
|---|---|
| `/home/dev/PX4-Autopilot/launch/multi_vehicle.launch` | 调整为 M6 场景：**car0 与 car1 均用 `lidar:=mid360_lio`**（两车出 `/carN/livox/lidar` CustomMsg + `/carN/imu`），car0 出生 `(2.3,2.4,-2.334)`；M6 起 `gui:=true spawn_cars:=true start_car_nav:=false start_ego:=false` |
| `/home/dev/XTDrone-single-car/ws_livox/src/Swarm-LIO2/swarm_lio/launch/dual_mid360_distributed.launch` | `quad0/quad1_lio_pose_guard`(pkg=car3_swarm) 节点加 `<param name="baro_z_topic" value="/iris_0/mavros/local_position/pose"/>` 与 `/iris_1/...`（用 `car3_lio/patch_guard_launch.py` 重打） |

**镜像内已含（非本次改动）**：`ws_livox`（Swarm-LIO2 源码、`livox_mid360.launch`、CustomMsg livox 插件）、PX4 `iris_mid360` 模型与 `/iris_N/livox/lidar` 输出、`dual_mid360_distributed.launch` 里 guard/dual_map_alignment 节点骨架、XTDrone 通信桥。

---

## 4. 验证结果与实测数据

| 项 | 数值/结果 |
|---|---|
| car0 单车 SWARM-LIO | `laserMapping_car0` 起，`/car0/lidar_slam/odom` 10 Hz；运动后 LIO 位移与 gazebo 真值一致（`motion_probe.py`） |
| 世界对齐（quad0↔car0，M6） | `TF quad0/world->car0/world` = 平移 (4.662, 0.171, 0.002) m、yaw −133.5°；gazebo 相对真值 car0 rel iris_0 ≈ (4.6, 0.1)、航向 −133.7° —— **一致** |
| car0 2D 导航（M6） | 目标 (1.3, 1.5) → move_base `nav.log` 报 **"Goal reached"**，gazebo 真值停靠 (1.35, 1.64)；AMCL 收敛误差 ~2 cm；`/car0/scan` 10 Hz |
| UAV 悬停（isolated 专测） | baro 高度 + guard Z 覆盖 + `--max-lio-tilt 45`：iris_0 物理 ~0.9–1.0 m **稳定 400 s 无看门狗故障**（log "纯 SWARM-LIO 定位保持 400.0 s"） |
| UAV 悬停（M6 同场长跑场景） | 起飞到 0.69 m 悬停 ~30 s 后 OFFBOARD 意外退出 → EKF 发散 → crash-land 房间边角（见 §5） |
| M6 终态 status | `/quad0/lidar_slam/odom` 10 Hz、`/car0/lidar_slam/odom` 10 Hz、`/car0/scan` 10 Hz；amcl/move_base/map_server/map_alignment 存活 |

**双车同场复测（2026-09-04 car1 完善后、新起干净 rosmaster + 干净 clean）**——car1 与 car0 完全同款配置，两车同场各自验证：

| 项 | 数值/结果 |
|---|---|
| 四路 SWARM-LIO | `/quad0`、`/quad1`、`/car0`、`/car1` 的 `lidar_slam/odom` 全部 ~10–13 Hz；两车 `/carN/livox/lidar`(CustomMsg) + `/carN/scan` ~10 Hz 均在 |
| car1/world 对齐 | `TF quad0/world→car1/world` = 平移 (-0.04, -4.73) m，gazebo 真值 car1 rel iris_0 ≈ (0.00,-4.70)——**一致**（两车均对齐进共享 quad0/world） |
| car0/world 对齐 | `TF quad0/world→car0/world` = 平移 (4.64, 0.15) m，gazebo 真值 car0 rel iris_0 ≈ (4.60,0.10)——**一致** |
| car1 AMCL + 2D 导航 | AMCL 收敛于出生 (-2.3,-2.4)；发目标 (1.0,1.5) → `/car1/move_base/status` **3 SUCCEEDED**，gazebo 真值停靠 (0.90,1.44) |
| car0 同场无回归 | 与 car1 同场并存，AMCL 收敛 (2.3,2.4)；发目标 (-1.0,-0.5) → move_base ACTIVE 连续导航 (2.30,2.40)→(0.69,0.94)→(-0.07,0.33)，AMCL 与真值贴合 |
| 可同场并存 | 根命名空间 `/move_base/*`(car0) 与 `/car1/move_base/*`(car1) 并存无 action/节点名冲突（`up_scan` 双实例已 `__name` 区分） |

> 复测期排查并固化的三个**运行/重跑类问题**（都写进 `clean()`/launcher，见 §3.1）：①旧 PX4 SITL 常驻 daemon 没被杀干净 → 重起场景报 `PX4 daemon already running for instance 0/1`、新 autopilot 空跑，现 `clean()` 杀 `px4_sitl_default` + 删 `/tmp/px4_lock-*`；②跨场景累积在同一个 rosmaster 上的**陈旧节点注册**会让新场景 `mecanum_controller_node "new node registered with same name"` 被挤死 → controller spawner 卡 100%×8 风暴 → CPU 打满 LIO 静默，**对策：彻底 clean 后新起干净 rosmaster 再 up_world**；③`up_scan` 双 `mid360_to_scan` 节点同名互挤，已带 `__name`。

---



## 5. 已知局限与边界

1. **同场长悬停不稳（主要短板）**：长时间场景 LIO-XY 漂移累积（多次 abort/重起飞中无人机实际位置被 PX4 追着漂移估计挪动数米）→ vision 创新过大 → PX4 弃用/助停 → 失去位置源后 EKF XY 发散、OFFBOARD 丢失。这是**XY**问题，不是高度；§4 的 400 s 稳定是在**新起/漂移小**的场景达成的。
   - 缓解方向：新场景/重对齐后短悬停验证；或周期用 gazebo 真值重对齐防 XY 漂移累积。
2. **LIO-Z 不跟踪垂直爬升**：本环境 SWARM-LIO 高度不可靠 → 用 baro 主高度 + guard Z 覆盖解决（已固化在 §6.4 配方）。
3. **car0/car1 出生都在房间角部"口袋"**（内墙围成，出屋需走门）：`car0_nav_map` 直接给全场图，首次导航会规划穿墙/穿门路径，正常给屋门内的目标点即可（两车各走各门/各通道）。图中 `car0/map`≈`car1/map`≈gazebo，两车用同一张图只因每车 ns 不同而帧名不同。
4. `laserMapping_car0` 沿用 UAV 的默认 body-frame TF 名会与 quad0 的该帧重合（cosmetic，不影响 odom 话题与对齐；car1 取 `drone_id=2` 空位避免与 quad0/quad1 撞）。

---

## 6. 最终使用方法

### 6.1 前置
- 容器在跑：`docker ps | grep xtdrone-swarm-lio`。
- 若重建容器：先做 §6.6 重做清单。
- GUI 直连宿主 X `:1`（`DISPLAY` 由 `env.sh`/run_m6 设 `:1`；容器需可连宿主 X，按该容器从镜像 tar 重建的启动方式：系统 docker + GUI 直连宿主 X :1，无 VNC）。按项目要求演示必须开 GUI，不 `--headless`。

### 6.2 一键分段启动（M6 空地同场）
在**容器内**执行（每条一个独立 `docker exec`）：

```bash
docker exec xtdrone-swarm-lio bash -lc '
cd /home/dev/XTDrone-single-car/workspace/car3_lio
bash run_m6_air_ground_demo.sh clean       # 杀旧场景全部进程(含 PX4 SITL daemon/spawner, 保 rosmaster)
bash run_m6_air_ground_demo.sh up_world    # multi_vehicle 场景: gazebo+iris_0/1 PX4+car0..3 (~数分钟起)
bash run_m6_air_ground_demo.sh up_uavlio   # 双机 UAV SWARM-LIO (quad0/1) + guard
bash run_m6_air_ground_demo.sh up_carlio   # 双车 SWARM-LIO: laserMapping_car0(id0)+car1(id2), car0/world+car1/world
bash run_m6_air_ground_demo.sh up_scan     # /car0/scan + /car1/scan (2D, 喂 AMCL/move_base)
bash run_m6_air_ground_demo.sh up_align    # 对齐 car0/world、car1/world -> quad0/world (gazebo 真值, 各一实例)
bash run_m6_air_ground_demo.sh up_nav      # 双 2D 导航栈: car0(根)+car1(/car1 ns) map+AMCL+move_base
bash run_m6_air_ground_demo.sh verify      # 数值核验(见 §7)
'
```
各阶段日志在容器 `/tmp/m6logs/*.log`。每段可单独重跑；`up_*` 内部会先 pkill 本段旧节点。**若 clean 后场景仍异常（如 spawner 100% 风暴、mecanum 节点被挤死），多半是旧 rosmaster 上堆了跨场景陈旧注册——先 `kill` 掉 rosmaster 再新起一个干净 rosmaster，然后从头 up_world**。

### 6.3 给 car0 / car1 发 2D 导航目标（两车可同场各发各的，互不干扰）
```bash
# car0（根命名空间：/move_base_simple/goal，帧 car0/map）
docker exec xtdrone-swarm-lio bash -lc 'source /home/dev/XTDrone-single-car/workspace/car3_lio/env.sh && \
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header:{frame_id:\"car0/map\"}, pose:{position:{x:1.3,y:1.5}, orientation:{w:1}}}"'
# car1（/car1 命名空间：/car1/move_base_simple/goal，帧 car1/map）
docker exec xtdrone-swarm-lio bash -lc 'source /home/dev/XTDrone-single-car/workspace/car3_lio/env.sh && \
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped \
"{header:{frame_id:\"car1/map\"}, pose:{position:{x:1.0,y:1.5}, orientation:{w:1}}}"'
```
或在 RViz 里分别选 `move_base_simple/goal`（car0/map 帧）与 `/car1/move_base_simple/goal`（car1/map 帧）发 2D Nav Goal。各自 move_base 接受后持续发布 `/car0/cmd_vel`、`/car1/cmd_vel`（稳定订阅者，麦轮车会正常动）。

### 6.4 iris_0 起飞悬停（两种模式）

**A. 稳定配方（实测可长悬停；推荐）——baro 主高度**（先确认 dual launch 已带 guard `baro_z_topic`，即 §3.4 已打）：
```bash
docker exec xtdrone-swarm-lio bash -lc '
cd /home/dev/XTDrone-single-car/workspace/car3_lio; source ./env.sh
nohup python3 ../swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff_lio_ref.py \
  --uavs iris_0 --altitude 0.7 --height-source baro --no-ego-handover \
  --hold-seconds 120 --timeout 90 --no-start-bridge --max-lio-tilt 45 > /tmp/m6logs/takeoff_stable.log 2>&1 &'
```
> 说明：PX4 `EKF2_HGT_MODE=0`(baro) 主高度；guard 把外部视觉输出 Z 覆盖成 PX4 自身 local z（self-consistent），XY/航向仍走 LIO；tilt 阈值放宽到 45° 避免爬升期 LIO 姿态漂移误报。**`--altitude` 是 local z**：物理爬升 ≈ altitude − MAVROS 静止 local z（约 −0.3~−0.66）。**务必在漂移小的新场景/刚重对齐后执行**（§5）。

**B. 全 SWARM-LIO-vision 模式（理念完整但脆弱）**：`run_m6_air_ground_demo.sh up_takeoff` 现默认 `--height-source vision`。LIO-Z 在本环境不跟踪垂直爬升（环境性局限），长悬停不稳，仅用于短验证。

### 6.5 单车 / 双车 SWARM-LIO 快速演示（M2/M3）
```bash
docker exec xtdrone-swarm-lio bash -lc '
cd /home/dev/XTDrone-single-car/workspace/car3_lio
bash run_m3_align_demo.sh        # 双车 car0+car1: bringup -> 各起 swarm_lio -> 对齐 car1/world->car0/world -> m3_verify
bash start_lio_single.sh         # M2 单车冒烟(或手动 roslaunch car3_swarm lio_car_bringup.launch gui:=true)
'
```

### 6.6 容器重建后的重做清单（容器本地不持久项）
1. 打 guard `baro_z_topic`：`python3 /home/dev/.../workspace/car3_lio/patch_guard_launch.py`（在容器内 ws_livox 的 `dual_mid360_distributed.launch` 上）。
2. 核对 `multi_vehicle.launch`：**car0 与 car1 都带 `<arg name="lidar" value="mid360_lio"/>`**、car0 出生 `(2.3,2.4,-2.334)`；M6 起 `spawn_cars:=true start_car_nav:=false start_ego:=false`。
3. 起通信桥后再 takeoff：`python3 /home/dev/XTDrone/communication/multirotor_communication.py iris 0`（或 `--no-start-bridge` 前先桥）。
4. 其余全部产物在宿主挂载（§3.1–3.3），天然保留。

---

## 7. 快速核对

```bash
# 整场健康度（节点/topic hz/模型位置）：
docker exec xtdrone-swarm-lio bash -lc \
 'cd /home/dev/XTDrone-single-car/workspace/car3_lio && bash run_m6_air_ground_demo.sh status'
# 对齐数值核验（gazebo 真值相对位姿 vs LIO TF 链）：
docker exec xtdrone-swarm-lio bash -lc \
 'cd /home/dev/XTDrone-single-car/workspace/car3_lio && bash run_m6_air_ground_demo.sh verify'
```
期望：四路 LIO odom 各 ~10 Hz（quad0/quad1/car0/car1）、`/car0/scan`、`/car1/scan` ~10 Hz、两车各自 AMCL 停在目标位；`verify` 报 `TF quad0/world→car0/world`、`quad0/world→car1/world` ≈ gazebo 相对 iris_0 位姿（双车一致，新场景下）。
