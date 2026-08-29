# V 型封控围捕（air_vblock_capture）定稿说明

> 两机两车围捕一机的**独立方案包** `air_vblock_capture`（不影响既有 `car3_swarm` Voronoi 方案）。
> 定稿验证：**2026-08-26 三次连续 CAPTURED**（入侵机减速至 0.5 m/s、防御机提速至 1.6 m/s）。
> 任务：入侵无人机 **iris_2** 自外墙角 CORNER_1 出生 → 恒高 1.33m 飞越 3m 内墙 → 触发入侵警报后尝试从其余三门逃逸；防御方两架 UAV（iris_0/1）在逃逸轴前方 V 型封堵，两辆小车（car0/1）对 iris_2 地面投影横向夹击，捕获判定**仅由两架 UAV（纯 2D）完成**。

---

## 1. 状态机（FSM）

```
WAITING → TAKEOFF → STAGING → INTRUDER_APPROACH → AIR_INTRUSION_DETECTED
                                                          │
                                                          ▼
                                              CONTAINMENT ─→ CAPTURED
                                                    │
                                                    ├→ FAILED_ESCAPE（iris_2 经某逃逸门离开内区）
                                                    └→ INVALID_ESCAPE
```

- **INTRUDER_APPROACH**：iris_2 三步走（门洞预对齐 → 墙外待机 → 墙内点），墙外须**真正停稳**才发墙内目标（防 EGO 带横移撞门框）。
- **CONTAINMENT**：
  - **逃逸门评分**（每逃逸门）：`J = w_d·d + w_p·Σ 1/(seg+0.6)`，`d`=iris_2 到该门距离，`seg`=两 UAV 到该门路径距离；带滞回（换门需代价优势 `exit_switch_margin=0.5` 持续 `exit_switch_hold_time=0.8s`）。逃逸轴 `e` = iris_2 二维速度方向（低速回退到目标门方向），指数平滑 `e_smooth=0.6`。
  - **UAV V 型点**：`PL/PR = pE + uav_forward_offset·e ± uav_lateral_offset·n`（逃逸轴前方 1.0m、横向展开 1.0m），目标 clamp 进内区并贴墙收缩 `uav_margin=0.9`（防 EGO 撞墙）。角色进入 CONTAINMENT 分配一次并锁定（role_lock）。
  - **小车夹击点**：`pE ± ugv_lateral_offset·n`，`n` 取**垂直于 iris_2 自身速度**的法向（`_car_flank_normal()`；速度 < `v_eps` 时回退朝内区中心）。**与空中逃逸门/UAV 捕获完全解耦**，不再参与逃逸轴后方封堵。
- **捕获判定**（纯 2D，两架 UAV）：两机 XY 距 iris_2 **均 ≤ R_c=1.5m**，且**双侧封堵** `z_A·z_B < 0`（分居逃逸轴两侧），持续 `T_hold=2.0s` → CAPTURED。小车不参与捕获。

---

## 2. 定稿参数

### 2.1 速度（逐机 EGO，经 `multi_vehicle.launch` 下发）

| 对象 | max_vel (m/s) | max_acc (m/s²) | 生效 arg |
|---|---|---|---|
| 防御 UAV iris_0/1 | **1.6** | **1.6** | `uav_pursuit_max_vel/acc` |
| 入侵 iris_2 | **0.5** | **0.8** | `uav_intruder_max_vel/acc` |

> 链路：`run_vblock_capture_sim.sh --uav-speed 1.6 --uav-acc 1.6 --intruder-speed 0.5 --intruder-acc 0.8` → `launch_air_intruder_sim.sh` 透传 → `multi_vehicle.launch` → 各机 `run_in_xtdrone.launch` 的 EGO `max_vel/max_acc`。
> 若直接跑旧 Voronoi 方案（不经这两个脚本），`multi_vehicle.launch` 默认仍为 0.75/0.8，行为不变。

### 2.2 任务参数（`vblock_mission.yaml`）

| 参数 | 值 | 含义 |
|---|---|---|
| `uav.fixed_altitude` | 1.33 | 三机统一恒高（< 3m 内墙） |
| `intruder.fixed_corner` | CORNER_1 | iris_2 出生 (-5,-5)，yaw -π/2 朝 DOWN 门 |
| `intruder.entry_side` | DOWN | 入侵门；逃逸门自动 = 其余三门 |
| `containment.uav_forward_offset` | 1.0 | UAV 在逃逸轴前方距离 df |
| `containment.uav_lateral_offset` | 1.0 | UAV 横向展开 ds |
| `containment.ugv_lateral_offset` | 1.0 | 小车投影夹击横向展开（左右各 1.0m） |
| `containment.uav_margin` / `ugv_margin` | 0.9 / 0.6 | 贴墙收缩余量（防撞墙） |
| `capture.air_capture_radius` | 1.5 | R_c：两 UAV 距 iris_2 的 XY 距离上限 |
| `capture.hold_time` | 2.0 | 捕获条件持续时长 T_hold |
| `capture.bilateral_blocking` | true | 双侧封堵 z_A·z_B<0 开关 |

---

## 3. 一键运行（推荐）

```bash
docker exec -it xtdrone-dev-gpu bash /home/dev/XTDrone-single-car/workspace/run_vblock_capture_oneclick.sh
```

自动串起：清理（含遗留 vblock 节点/挂死 takeoff）→ launch（iris_2 固定 CORNER_1）→ 等世界 → 起 get_local_pose/ego_swarm_transfer/3 通信桥 → 等 mavros → 三机起飞 1.33m → 起 `vblock_capture` 节点 → **自动等待终态并打印结果**（无需手动查日志）。

**任意阶段按 Ctrl+C 即结束仿真**：脚本安装 SIGINT/SIGTERM 陷阱，收到中断后自动清理全部仿真进程（roslaunch/gzserver/gzclient/px4/mavros/EGO/traj_server/rosmaster/通信桥/vblock 节点，TERM→KILL 两级），退出码 130。正常出结果后仿真保留运行，可再 Ctrl+C 收尾。

可选参数（透传，缺省即定稿值）：
`--corner CORNER_0|CORNER_1` · `--entry UP|DOWN|LEFT|RIGHT` · `--uav-speed <m/s>` · `--uav-acc` · `--intruder-speed` · `--intruder-acc` · `--alt <高度>` · `--headless`

等结果超时默认 360s，可用环境变量 `LOG_POLL_SEC` 覆盖。

### 手动分步

```bash
# 1) 启动仿真（清理+launch，默认 fixed CORNER_1）
bash /home/dev/XTDrone-single-car/workspace/launch_air_intruder_sim.sh \
    --uav-speed 1.6 --uav-acc 1.6 --intruder-speed 0.5 --intruder-acc 0.8

# 2) 支持栈 + 3 通信桥（容器内，三机起飞须先持久化起桥）
python3 /home/dev/XTDrone/sensing/pose_ground_truth/get_local_pose.py iris 3 &
python3 /home/dev/XTDrone/motion_planning/3d/ego_swarm_transfer.py iris 3 &
for id in 0 1 2; do python3 /home/dev/XTDrone/communication/multirotor_communication.py iris $id & done

# 3) 三机起飞
python3 /home/dev/XTDrone-single-car/workspace/swarm_defense_ws/src/car3_swarm/scripts/uav_offboard_takeoff.py \
    --altitude 1.33 --timeout 120 --no-start-bridge

# 4) 起围捕节点
source /home/dev/car3_env.sh
roslaunch air_vblock_capture vblock_capture.launch entry_side:=DOWN
```

### 结果监视

```bash
rostopic echo -n1 /vblock_capture/result        # CAPTURED / FAILED_ESCAPE / INVALID_ESCAPE
grep 'state ->' ~/.ros/log/latest/vblock_capture-*.log | tail -5
```

---

## 4. 本轮关键改动（2026-08-26）

1. **速度拆分**：入侵 iris_2 0.75 → **0.5** m/s，防御 iris_0/1 → **1.6** m/s。
   动机：等速/1.2/1.6 下多现 FAILED_ESCAPE——入侵机高速下果断冲门，捕获依赖"它恰好犹豫"的随机性；减速后入侵机无法在 1.6 两机夹击下从任一逃逸门冲出，捕获稳定。
2. **小车改投影夹击**：新增 `_car_flank_normal()`，两车朝 `pE ± ds·n` 横向夹击 iris_2 **地面投影**（n ⊥ iris_2 自身速度），删除 `ugv_rear_offset`，不再参与逃逸轴后方封堵、与空中逃逸门/UAV 捕获解耦。
3. **脚本健壮性**：
   - `timeout -k 2 N` 修复：`timeout` 不带 `-k` 时，遇到不响应 SIGTERM 的进程（如 `rostopic echo` 订阅到尚未发布的话题）会**无限等待**；加 `-k 2` 保证 2s 后 SIGKILL。
   - 启动前清理遗留 `vblock_capture` 节点 / 挂死 takeoff 进程（`[x]` 括号防 pkill 自匹配），防多代堆叠抢控制。
4. **新增一键脚本** `run_vblock_capture_oneclick.sh`（自动定位新节点日志按 `__log` 参数，不依赖 `latest` 软链，杜绝旧日志误跟踪）。

---

## 5. 验证记录（2026-08-26，三次连续 CAPTURED）

| # | 时间 | 结果 | UAV 最大距离 | 围捕耗时 |
|---|---|---|---|---|
| 1 | 22:56 | CAPTURED @ (-0.810, -1.696) | 1.140 m | 8.7 s |
| 2 | 23:11 | CAPTURED @ (-0.729, -1.349) | 0.785 m | 34.9 s |
| 3 | 23:17 | CAPTURED @ (-0.384, -1.822) | 1.415 m | 16.8 s |

> 三次均 `R_c=1.5` 达标，入侵机始终未从 UP/LEFT/RIGHT 任一逃逸门冲出。

---

## 6. 文件清单

| 文件（容器内路径） | 宿主路径 | 作用 |
|---|---|---|
| `swarm_defense_ws/src/air_vblock_capture/scripts/vblock_capture_node.py` | 同左 | 围捕 FSM 主节点 |
| `swarm_defense_ws/src/air_vblock_capture/config/vblock_mission.yaml` | 同左 | 全部任务参数（读一次，改后重启节点） |
| `swarm_defense_ws/src/air_vblock_capture/launch/vblock_capture.launch` | 同左 | 加载参数并启动节点 |
| `workspace/run_vblock_capture_oneclick.sh` | 同左 | **一键启动 + 自动等终态打印结果** |
| `workspace/run_vblock_capture_sim.sh` | 同左 | 整套仿真启动流程（复用对象） |
| `workspace/launch_air_intruder_sim.sh` | 同左 | 清理 + `multi_vehicle.launch` 启动（速度参数透传） |
| 容器 `/home/dev/PX4-Autopilot/launch/multi_vehicle.launch` | — | 新增 `uav_pursuit/intruder_max_vel/acc` 4 个 arg 并接入各机 EGO |
| 容器 `/home/dev/ego_ws/src/ego_planner/plan_manage/launch/run_in_xtdrone.launch` | — | EGO `max_vel/max_acc` 参数化（供 launch 覆盖） |

---

## 7. 注意事项

- **不改地图**：地图/SDF 未改动，只改飞行高度；内墙/门数值取自 `nesting_room/model.sdf` 实测。
- **全机固定朝向**：桥 `hold_yaw` 锁定各机出生朝向（iris_0/1=0°，iris_2=-90°）。
- **改配置生效方式**：`vblock_mission.yaml` 与速度参数在节点/launch 启动时读取，改完需重启对应进程。
- **容器内文件改动**：`multi_vehicle.launch` 与 EGO launch 在容器镜像内（非宿主挂载），需容器内 sed 或 `docker cp`。
- **gzserver 僵死**：多次 `kill -9 gzserver` 会污染 gazebo_ros 桥（world 加载但无 `/gazebo/*` 话题），整栈僵死时 `docker restart xtdrone-dev-gpu`（workspace 宿主挂载无损）。
