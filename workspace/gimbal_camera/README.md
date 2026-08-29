# 云台相机仿真（gimbal_camera）

CAD 导出的云台相机部件（源材料：宿主 `~/XTDrone_camera/camera_urdf/`，Ctrl-CVCV/XTDrone fork 的 commit `c67f948`），
已集成到 XTDrone 容器 **v1.5**（镜像 `xtdrone-noetic-px4:1.13.2-v1.5`，容器 `xtdrone-dev`）。

目录位于宿主挂载 `/workspace/gimbal_camera`（宿主路径 `~/xtdrone_docker/workspace/gimbal_camera`），
容器重建不丢失，改任何文件即时生效、**无需编译**（不是 catkin 包，全部绝对路径引用）。

2026-08-16 全链路实测通过：Gazebo 仿真 + ros_control 关节控制 + 相机话题输出 + RViz 实时画面（GPU 渲染）。

---

## 快速上手

```bash
# 一键启动（Gazebo 左 / RViz 右 自动摆窗，宿主桌面直接显示）
docker exec -it xtdrone-dev bash /workspace/gimbal_camera/gimbal_demo.sh
```

启动流程（脚本自动完成）：清理残留 → 启动 Gazebo 测试世界（0.5m 底座柱 + 彩色方块环）→ spawn 云台模型
→ 加载 ros_control 控制器 → yaw/roll 正弦摆动 → 相机画面自动验证 → RViz（相机画面 / 模型 / TF）。

**停止**（两种方式，任选）：

```bash
# 方式一：启动终端在前台时，直接 Ctrl+C（脚本 trap 一键关闭全部进程）
# 方式二：后台/无 TTY 启动时，发 TERM 触发同样的清理
docker exec xtdrone-dev pkill -TERM -f "[g]imbal_demo"
```

**手动控制云台**（演示运行中，另开终端）：

```bash
docker exec xtdrone-dev bash -c 'source /home/dev/car3_env.sh && export DISPLAY=:1 && \
  rostopic pub -1 /gimbal/gimbal_yaw_controller/command std_msgs/Float64 "data: 0.8" && \
  rostopic pub -1 /gimbal/gimbal_roll_controller/command std_msgs/Float64 "data: 0.5"'
```

---

## 环境前置条件

| 条件 | 说明 |
|---|---|
| 容器运行中 | `xtdrone-dev`（系统 docker：`DOCKER_HOST=unix:///var/run/docker.sock`） |
| X 授权 | 宿主 X 重启后需重跑一次 `xhost +local:`，否则容器连不上 `:1` |
| ROS 环境 | `docker exec` 不触发 entrypoint，任何 ros 命令前先 `source /home/dev/car3_env.sh` |
| 渲染 | 脚本自动设 `DISPLAY=:1`（宿主桌面 NVIDIA GPU 直连，无 VNC） |
| GAZEBO_MODEL_PATH | 脚本自动补 `/usr/share/gazebo-11/models`（**手动启动时必须自己 export**，见"常见问题"第 1 条） |

---

## 话题与服务

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gimbal_camera/image_raw` | sensor_msgs/Image | 相机画面 640×480，实测 ~19Hz（标称 30Hz，受渲染负载影响） |
| `/gimbal_camera/camera_info` | sensor_msgs/CameraInfo | 相机内参，frame_id = `link_PITCH` |
| `/gimbal/joint_states` | sensor_msgs/JointState | 关节状态 50Hz（yaw, roll） |
| `/gimbal/gimbal_yaw_controller/command` | std_msgs/Float64 | yaw 目标角（rad），±1.57 |
| `/gimbal/gimbal_roll_controller/command` | std_msgs/Float64 | roll 目标角（rad），±1.57 |
| `/gimbal/gimbal_yaw_controller/state` | control_msgs/JointControllerState | yaw 控制器状态（set_point/process_value/error） |

TF：`base_link → link_YAW → link_ROLL → link_PITCH`（robot_state_publisher 发布，固定帧 `base_link`）。

常用命令：

```bash
# 看相机帧率
rostopic hz /gimbal_camera/image_raw
# 看关节实时位置
rostopic echo /gimbal/joint_states
# 保存一帧相机图片
rosrun image_view image_saver image:=/gimbal_camera/image_raw _save_all_image:=false
# 自动验证相机画面（亮度/彩色像素判定，PASS/FAIL）
python3 /workspace/gimbal_camera/scripts/gimbal_cam_check.py
```

---

## 控制云台

- 两个可驱动关节：`link_YAW_joint`（绕 z 轴）、`link_ROLL_joint`（倾斜轴），限位均为 **±1.57 rad（±90°）**；
  `link_PITCH_joint` 是固定关节（CAD 结构如此），相机装在 `link_PITCH` 上。
- 位置控制器收到目标角后由 PID 闭环跟踪；关节限位、力矩上限（10 N·m）、速度上限（5 rad/s）
  定义在 `robot.urdf` 的 `<limit>`，PID 参数在 `config/gimbal_control.yaml`。
- 演示脚本 `scripts/gimbal_move.py` 默认正弦摆动，可传参：

```bash
# 用法: gimbal_move.py [yaw幅值] [yaw周期s] [roll幅值] [roll周期s]
python3 /workspace/gimbal_camera/scripts/gimbal_move.py 1.0 8 0.6 6
```

---

## 目录结构

```
gimbal_camera/
├── robot.urdf                  # 集成版 URDF：绝对 mesh 路径 + transmission + gazebo_ros_control
├── meshes/                     # CAD 导出 STL（10 个部件，URDF 中按 file:///workspace/... 绝对路径引用）
├── worlds/gimbal_test.world    # 测试世界（地面/太阳光内联 + 底座柱 + 8 个彩色方块环）
├── launch/gimbal_controllers.launch   # robot_description 参数 + robot_state_publisher + controller_spawner
├── config/gimbal_control.yaml  # joint_state + yaw/roll 位置控制器（含 PID）
├── config/gimbal.rviz          # RViz 配置（Image 背景 + RobotModel + TF）
├── scripts/gimbal_move.py      # 正弦摆动演示（参数见上）
├── scripts/gimbal_cam_check.py # 相机画面自动验证（抓一帧分析亮度/彩色像素）
├── scripts/gcam_analyze.py     # 调试用：相机帧 vs RViz 窗口截图统计对比
└── gimbal_demo.sh              # 一键启动脚本
```

---

## 分步手动启动（调试用，替代一键脚本）

```bash
docker exec -it xtdrone-dev bash
source /home/dev/car3_env.sh
export DISPLAY=:1
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

# 1) 启动 Gazebo（世界文件绝对路径）
roslaunch gazebo_ros empty_world.launch gui:=true \
    world_name:=/workspace/gimbal_camera/worlds/gimbal_test.world

# 2) 先启动控制器 launch（这一步会把 robot_description 参数加载好——
#    gazebo_ros_control 插件在模型 spawn 时读它，顺序不能反）
roslaunch /workspace/gimbal_camera/launch/gimbal_controllers.launch

# 3) spawn 云台模型（底座柱顶 0.5m 处；无窗口可用 gui:=false）
rosrun gazebo_ros spawn_model -file /workspace/gimbal_camera/robot.urdf -urdf \
    -model gimbal_camera -x 0 -y 0 -z 0.5

# 4) 控制 + 显示
python3 /workspace/gimbal_camera/scripts/gimbal_move.py &
rviz -d /workspace/gimbal_camera/config/gimbal.rviz
```

只跑相机话题不做 GUI 时：`gui:=false` 且跳过 rviz（gzserver 仍会渲染相机传感器）。

---

## 修改模型 / 世界（无编译工作流）

- **改 URDF/世界/yaml/rviz 配置后**：直接重跑 `gimbal_demo.sh` 即生效，无需任何编译。
- **改 URDF 后自检**：`gz sdf -p /workspace/gimbal_camera/robot.urdf`（能转换成功=语法没问题）。
- **mesh 路径**：URDF 中写死为 `file:///workspace/gimbal_camera/meshes/...`——若移动整个目录，
  需同步替换 URDF 里的前缀（`sed -i 's#/workspace/gimbal_camera#/新路径#g' robot.urdf`）。
- **改世界**：`gz sdf -p worlds/gimbal_test.world` 自检；注意世界里的地面/太阳光已内联，
  不要改回 `model://ground_plane` 引用（会触发燃料下载卡死，见下）。
- **rviz 配置**：`gimbal.rviz` 的 Image display 订阅 `/gimbal_camera/image_raw` 作为 3D 视图背景；
  固定帧为 `base_link`；改显示内容直接编辑该文件或 rviz 界面 Save。

---

## 常见问题排查

| 症状 | 原因 / 处理 |
|---|---|
| gazebo 长时间起不来、日志无输出 | `GAZEBO_MODEL_PATH` 缺 `/usr/share/gazebo-11/models`，`model://` 引用触发 fuel 在线下载卡死。手动启动时先 `export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH`（demo 脚本已处理） |
| 等待 gazebo 就绪超时 | noetic 的 spawn 服务名是 `/gazebo/spawn_urdf_model`、`/gazebo/spawn_sdf_model`，**没有** `/gazebo/spawn_entity`；确认 `rosservice list \| grep spawn` |
| `/gimbal_camera/image_raw` 不出现 | 查 `/tmp/gimbal_gazebo.log`（一键脚本时）；确认相机插件随模型 spawn 成功（gazebo 界面模型树里应有 gimbal_camera） |
| 控制器话题不出现（`/gimbal/gimbal_*_controller/command`） | 查 `/tmp/gimbal_ctrl.log`；最常见原因是先 spawn 后加载参数——`robot_description`（全局和 `/gimbal` 命名空间两份）必须在模型 spawn **之前**加载，重跑一键脚本即可 |
| rviz 打开但看不到相机画面 | 先 `rostopic hz /gimbal_camera/image_raw` 确认话题有数据；rviz 里检查 Image display 的 Topic 是否为 `/gimbal_camera/image_raw`；`roslaunch` 环境下确认 `/use_sim_time` 为 true |
| 模型悬空或陷地 | spawn 的 `-z` 参数（默认 0.5，对齐底座柱顶）；底座柱在 `worlds/gimbal_test.world` 的 `gimbal_pedestal` 模型 |
| 关节不动 | 确认有进程在发 command（演示脚本是 `gimbal_move.py`，杀掉后云台会停在最后位置）；看 `/gimbal/gimbal_yaw_controller/state` 的 error 是否收敛 |
| 容器连不上宿主 X | 宿主执行 `xhost +local:`；确认容器内 `DISPLAY=:1` 且 `xdpyinfo` 能通 |
| 残留进程没清干净 | 重启 demo 脚本会自动清理（pkill 列表含 rosmaster/gzserver/gzclient/rviz 等）；或手动 `docker exec xtdrone-dev pkill -TERM -f "[g]imbal_demo"` |

---

## 进阶：挂载到其他载体（后续方向）

- **挂到 car3 麦轮车**：在 car3 的 URDF 里加一个 fixed joint 把云台 `base_link` 接上去，
  并把本目录 URDF 的 link/joint/transmission 段并入 car3 描述（控制器命名空间注意与 car3 的
  gazebo_ros_control 区分，避免 controller_manager 冲突）。
- **走 PX4 mount_control 链路**：容器内已有 `libgazebo_gimbal_controller_plugin.so`（支持
  `<joint_yaw>/<joint_roll>/<joint_pitch>` 自定义关节名）；源材料 `~/XTDrone_camera/sensing/gimbal/`
  和 `~/XTDrone_camera/sitl_config/`（含 `2033_plane_gimbal` 机型、typhoon_h480 带云台模型）在宿主机，
  可参考迁移。
