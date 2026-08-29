# mid360_car3 — 独立 MID-360 3D 激光雷达麦轮小车

在 Gazebo 中加载一辆 **car3 麦轮底盘 + MID-360 3D 激光雷达** 的小车，用 `livox_laser_simulation` 插件复现 MID-360 的真实非重复扫描模式，实时发布 `sensor_msgs/PointCloud2` 点云到 `/scan`，可在 RViz 中查看。

这是一个**独立演示工作区**，不运行整个围捕仿真（不依赖 `swarm_defense_ws` 的 car3_swarm 场景）。

## 目录结构

```
mid360_car3/
├── run_mid360_car3.sh              # 一键启动脚本
├── README.md
└── src/
    ├── livox_laser_simulation/     # 雷达插件（qiurongcan/Mid360_imu_sim 的 PointCloud2_1 分支）
    │   ├── meshes/mid360.dae       # MID-360 外观 mesh
    │   ├── scan_mode/mid360.csv    # 真实扫描方向采样表（80 万行 Azimuth/Zenith）
    │   └── CMakeLists.txt          # 已删 libprotobuf.so.9 链接行（容器只有 libprotobuf.so.17）
    └── mid360_car3/                # 本包（纯启动配置，无源码）
        ├── launch/mid360_car3.launch
        ├── rviz/mid360_car3.rviz
        ├── worlds/mid360_demo.world  # 8×8 房间 + 天花板 + 障碍物（保证点云有内容）
        ├── package.xml
        └── CMakeLists.txt
```

## 前置条件

- 容器：`xtdrone-dev-gpu`（活动容器）
- 宿主 X 已授权：`xhost +local:`
- 容器内可用 `glxinfo -B` 验证 X 连接（`direct rendering: Yes` + NVIDIA）

## 构建

构建过一次即可，`devel/` 已在工作区。

```bash
docker exec xtdrone-dev-gpu bash -lc '
cd /home/dev/XTDrone-single-car/workspace/mid360_car3
source /home/dev/car3_env.sh
catkin build livox_laser_simulation mid360_car3
'
```

> 注意：插件源码本身不用 protobuf，但上游 CMakeLists 硬链接 `libprotobuf.so.9`，已删掉该行（详见目录结构备注）。

## 运行

```bash
# 围捕小车变体（无红块，默认）
docker exec xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/mid360_car3/run_mid360_car3.sh

# 入侵机变体（带红色物块）
docker exec xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/mid360_car3/run_mid360_car3.sh true

# 显式围捕车变体
docker exec xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/mid360_car3/run_mid360_car3.sh false
```

启动脚本会：清理残留进程 → 注入工作区路径 → `roslaunch mid360_car3 mid360_car3.launch`，拉起 Gazebo(GUI) + 模型 + RViz。按 `Ctrl+C` 一键关闭全部进程。

## 话题与坐标系

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scan` | `sensor_msgs/PointCloud2` | MID-360 点云，10 Hz，24000 点，`frame_id = livox_link` |
| `/cmd_vel` | `geometry_msgs/Twist` | 麦轮底盘速度指令（linear.x 前进，linear.y 横移，angular.z 旋转） |
| `/odom` | `nav_msgs/Odometry` | 里程计（ground truth，来自 gazebo 模型位姿） |
| `/joint_states` | `sensor_msgs/JointState` | 轮子关节角速度等 |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | 模型列表与位姿 |

TF 链：`odom → base_footprint`（由 ground_truth_odom_node 发布，odom 帧=世界原点）→ `livox_link`（由 robot_state_publisher 发布）。RViz 固定坐标系为 `odom`。

## 验证方法

```bash
# 点云频率
rostopic hz /scan
# 点云帧与类型
rostopic type /scan          # sensor_msgs/PointCloud2
rostopic echo -n1 /scan/header
# 模型是否生成
rostopic echo -n1 /gazebo/model_states/name
# TF 链
rosrun tf tf_echo odom livox_link
```

> **重要坑：`rostopic pub` 驱动不了 mecanum 控制器。** `rostopic pub` 的进程分叉（fork）生命周期会导致 C++ 订阅者（mecanum_controller_node）无法连接发布者，表现为轮速话题无输出、小车不动。**测试运动必须用稳定发布者**，例如下面的 Python 脚本，或键盘控制器 / 导航节点：

```bash
# 前进 2 秒后停止
python3 - <<'PY'
import rospy, time
from geometry_msgs.msg import Twist
rospy.init_node("vel_demo", anonymous=True)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
def drive(vx, dur):
    m = Twist(); m.linear.x = vx
    t0 = time.time()
    while time.time()-t0 < dur:
        pub.publish(m); time.sleep(0.05)
drive(0.3, 2.0)   # 前进 0.3 m/s × 2 s
drive(0.0, 0.2)   # 停
PY
```

## 变体说明

- **围捕小车**（`red_marker=false`，默认）：无红色物块。
- **入侵机**（`red_marker=true`）：车顶加 0.08 m 红色立方体（`intruder_marker` 链接，质量 0.0001，纯视觉，不参与物理/导航；顶 0.17 m < 雷达底 0.2356 m，不遮挡雷达）。

## 自定义

- 雷达点数：改 launch 里 `livox_samples`（默认 24000）。
- 出生位置：`x` / `y` / `yaw` 参数。
- 场地：`worlds/mid360_demo.world`（8×8 房间 + 四面墙 + 天花板 + 3 个障碍物）。
- 雷达换用 2D：`run_mid360_car3.sh` 依赖的 `car3.xacro` 支持 `lidar:=hokuyo|mid360`（本工作区固定 mid360）。
