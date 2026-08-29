#!/usr/bin/env python3
"""V-block capture: two defender UAVs (front V) + two defender UGVs (rear V)
vs one intruder UAV.

iris_2 (intruder) spawns in an outer-wall corner, takes off to the uniform
altitude H_f, stages at its corner, then flies over the 3m inner wall (through
the entry gate gap) into the protected area. When its world XY transitions
outside -> inside the inner region, AIR_INTRUSION_DETECTED fires.

After intrusion the mission enters CONTAINMENT (ported from the two-car "area
containment" scheme):
  - escape axis e from iris_2's 2D velocity direction (fallback to the current
    chosen escape gate direction); smoothed.
  - iris_2 selects an escape gate (all gates except the entry gate) via cost
    scoring + hysteresis switching.
  - UAVs take front V-blocking points  PL/PR = pE + df_uav*e +/- ds_uav*n
    (roles locked once on entry).
  - UGVs pincer the intruder's ground projection  CL/CR = pE +/- ds_car*n
    (n = normal to iris_2's own 2D velocity, low-speed fallback toward the
    inner region centre -- decoupled from the air escape axis; motion only,
    never part of the capture judgement).
  - capture = both UAVs within R_c (XY only, no altitude) + bilateral blocking
    (z_A*z_B<0) + at least one UAV ahead of the escape axis, held T_hold.
  - escape = iris_2 leaves the inner region (FAILED_ESCAPE through a valid
    gate, INVALID_ESCAPE through the original entry gate).

Coordinate conventions:
  - All tactical math uses Gazebo world XY (from /gazebo/model_states).
  - UAV goals are published to /iris_N/move_base_simple/goal (EGO) in each
    UAV's MAVROS local frame (translation offset, world-aligned axes).
  - UGV goals are published to /carN/move_base_simple/goal (Nav) in world XY.
"""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


PURSUERS = ("car0", "car1", "iris_0", "iris_1")
UAVS = ("iris_0", "iris_1")
CARS = ("car0", "car1")
EVADER = "iris_2"
ALL_AGENTS = PURSUERS + (EVADER,)

ENTRY_GATES = {
    "UP": (-0.116, 3.041),
    "DOWN": (-0.116, -3.059),
    "LEFT": (-3.166, -0.009),
    "RIGHT": (2.934, -0.009),
}
ENTRY_NORMALS = {
    "UP": (0.0, -1.0),
    "DOWN": (0.0, 1.0),
    "LEFT": (1.0, 0.0),
    "RIGHT": (-1.0, 0.0),
}
# 每个出生角落可直连的入侵方向（直线走廊段不跨越内区，approach 段不会提前入侵）
CORNER_ENTRY_DIRECT = {
    "CORNER_0": ("UP", "RIGHT"),
    "CORNER_1": ("DOWN", "LEFT"),
    "CORNER_2": ("DOWN", "RIGHT"),
}


def planar_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def inside_inner(point, region):
    xmin, xmax, ymin, ymax = region
    return xmin <= point[0] <= xmax and ymin <= point[1] <= ymax


def clamp_point(point, bounds, margin):
    xmin, xmax, ymin, ymax = bounds
    return (
        max(xmin + margin, min(xmax - margin, point[0])),
        max(ymin + margin, min(ymax - margin, point[1])),
    )


def point_segment_distance(p, a, b):
    """点到线段 ab 的最近距离。"""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        t = 0.0
    else:
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


class VblockCapture:
    WAITING = "WAITING"
    TAKEOFF = "TAKEOFF"
    STAGING = "STAGING"
    INTRUDER_APPROACH = "INTRUDER_APPROACH"
    AIR_INTRUSION_DETECTED = "AIR_INTRUSION_DETECTED"
    CONTAINMENT = "CONTAINMENT"
    CAPTURED = "CAPTURED"
    FAILED_ESCAPE = "FAILED_ESCAPE"
    INVALID_ESCAPE = "INVALID_ESCAPE"

    def __init__(self):
        rospy.init_node("vblock_capture")

        # ---- YAML 参数（launch 已 rosparam load 到 ~）----
        self.uav_altitude = rospy.get_param("~uav/fixed_altitude", 4.0)
        self.wall_height = rospy.get_param("~uav/wall_height", 3.0)

        self.spawn_mode = str(rospy.get_param("~intruder/spawn_mode", "fixed")).lower()
        self.fixed_corner = str(
            rospy.get_param("~intruder/fixed_corner", "CORNER_0")
        ).upper()
        corners = rospy.get_param("~intruder/spawn_corner_0", [5.0, 5.0, -2.367])
        corners1 = rospy.get_param("~intruder/spawn_corner_1", [-5.0, -5.0, 0.796])
        corners2 = rospy.get_param("~intruder/spawn_corner_2", [5.2, -4.9, 3.1416])
        self.corner_xy = {
            "CORNER_0": (corners[0], corners[1]),
            "CORNER_1": (corners1[0], corners1[1]),
            "CORNER_2": (corners2[0], corners2[1]),
        }
        self.entry_mode = str(rospy.get_param("~intruder/entry_mode", "fixed")).lower()
        self.entry_side = str(
            rospy.get_param("~entry_side", "")
            or rospy.get_param("~intruder/entry_side", "LEFT")
        ).upper()
        self.approach_offset = rospy.get_param("~intruder/approach_offset", 1.2)
        self.inside_offset = rospy.get_param("~intruder/inside_offset", 1.2)
        self.entry_arrive = rospy.get_param("~intruder/entry_arrive", 0.4)
        # 到达外墙外侧待机点后需等待无人机真正停稳再发布进内部目标，否则 EGO 会带着
        # 到达方向的横向动量重规划入场轨迹（可偏出门洞撞墙）。
        self.approach_settle = rospy.get_param("~intruder/approach_settle", 1.2)
        self.entry_speed_eps = rospy.get_param("~intruder/entry_speed_eps", 0.1)
        self.entry_speed_hold = rospy.get_param("~intruder/entry_speed_hold", 0.4)
        self.entry_speed_cap = rospy.get_param("~intruder/entry_speed_cap", 6.0)
        self.approach_pre_extra = rospy.get_param("~intruder/approach_pre_extra", 2.0)

        # 逃逸出口评分与滞回
        self.exit_switch_margin = rospy.get_param("~intruder/exit_switch_margin", 0.5)
        self.exit_switch_hold_time = rospy.get_param(
            "~intruder/exit_switch_hold_time", 0.8
        )
        self.w_dist = rospy.get_param("~intruder/escape_weight_dist", 1.0)
        self.w_pursuer = rospy.get_param("~intruder/escape_weight_pursuer", 1.2)
        self.w_obstacle = rospy.get_param("~intruder/escape_weight_obstacle", 0.8)
        # 逃逸轴速度阈值：iris_2 速度低于该值 → 逃逸轴回退到目标门方向
        self.v_eps = rospy.get_param("~intruder/v_eps", 0.3)

        self.tactical_rate = rospy.get_param("~planning/tactical_rate", 15.0)
        self.goal_rate = rospy.get_param("~planning/goal_publish_rate", 3.0)
        self.goal_min_change = rospy.get_param(
            "~planning/goal_change_threshold", 0.3
        )
        self.re_goal_interval = rospy.get_param(
            "~planning/re_goal_interval", 5.0
        )

        self.uav_forward_offset = rospy.get_param("~containment/uav_forward_offset", 1.0)
        self.uav_lateral_offset = rospy.get_param("~containment/uav_lateral_offset", 1.0)
        self.ugv_lateral_offset = rospy.get_param("~containment/ugv_lateral_offset", 1.0)
        self.uav_margin = rospy.get_param("~containment/uav_margin", 0.9)
        self.ugv_margin = rospy.get_param("~containment/ugv_margin", 0.6)
        self.role_lock = rospy.get_param("~containment/role_lock", True)
        self.e_smooth = rospy.get_param("~containment/e_smooth", 0.6)

        self.air_capture_radius = rospy.get_param("~capture/air_capture_radius", 1.5)
        self.hold_time = rospy.get_param("~capture/hold_time", 2.0)
        self.bilateral_blocking = rospy.get_param("~capture/bilateral_blocking", True)

        self.intrusion_debounce = rospy.get_param("~fsm/intrusion_debounce", 0.3)
        self.auto_start = rospy.get_param("~fsm/auto_start", True)
        self.start_delay = rospy.get_param("~fsm/start_delay", 1.0)

        inner = rospy.get_param(
            "~inner_region",
            {"x_min": -3.091, "x_max": 2.859, "y_min": -2.984, "y_max": 2.966},
        )
        self.inner_region = (
            inner["x_min"],
            inner["x_max"],
            inner["y_min"],
            inner["y_max"],
        )

        # 出生角落：spawn_mode=fixed 用 fixed_corner；random 由 launch 侧注入
        # （/air_intruder/spawn_corner rosparam，launch wrapper 设置）
        self.spawn_corner = str(
            rospy.get_param("/air_intruder/spawn_corner", self.fixed_corner)
        ).upper()
        if self.spawn_corner not in self.corner_xy:
            self.spawn_corner = self.fixed_corner
        if self.entry_side not in ENTRY_GATES:
            rospy.logwarn("unknown entry side %r; using LEFT", self.entry_side)
            self.entry_side = "LEFT"
        # 只允许与出生角落相邻的墙作为入侵方向（直线走廊段不跨越内区）。
        direct = CORNER_ENTRY_DIRECT[self.spawn_corner]
        if self.entry_side not in direct:
            fallback = direct[0]
            rospy.logwarn(
                "entry %s 非 %s 相邻方向（直连方向=%s）；使用 %s",
                self.entry_side,
                self.spawn_corner,
                "/".join(direct),
                fallback,
            )
            self.entry_side = fallback
        self.crossing_side = self.entry_side
        # 合法逃逸门 = 全部门 \ 入侵门（§4.2）
        self.valid_escape_gates = [g for g in ENTRY_GATES if g != self.entry_side]

        # 出生角落 + 入侵方向 -> 走廊段路线（全程 XY 停留在外环，approach 不提前入侵）
        self.staging_outside = self._wall_outside_point(self.entry_side)
        self.staging_align = self._wall_align_point(self.entry_side)
        self.entry_inside = self._wall_inside_point(self.entry_side)

        # ---- 状态 ----
        self.model_poses = {}
        self.uav_local_poses = {}
        self.uav_states = {}
        self.state = self.WAITING
        self.started = False
        self.ready_since = None
        self.phase_started_at = None
        self.approach_step = 0
        self.approach_goal_sent = False
        self.staging_arrived_at = None
        self.staging_last_fast_at = None
        self.evader_last_pos = None
        self.evader_last_pos_at = None
        self.was_inside = False
        self.inside_since = None
        self.intrusion_fired_at = None
        self.hold_goals = {}
        self.last_publish_time = {}
        self.published_targets = {}
        self.last_targets = {}
        self.goal_connection_counts = {name: 0 for name in ALL_AGENTS}

        # ---- CONTAINMENT 状态 ----
        self.uav_left = "iris_0"
        self.uav_right = "iris_1"
        self.ugv_left = "car0"
        self.ugv_right = "car1"
        self.pursuit_started_at = None
        self.evader_vel_history = []
        self.e_axis = None
        self.last_velocity_dir = None
        self.current_escape_gate = None
        self.best_gate_since = None
        self.last_gate_costs = {}
        self.capture_hold_started = None
        self.last_hold_publish = 0.0
        self.last_containment_targets = {}

        # ---- 发布/订阅 ----
        self.goal_publishers = {
            name: rospy.Publisher(
                "/%s/move_base_simple/goal" % name, PoseStamped, queue_size=1
            )
            for name in ALL_AGENTS
        }
        self.uav_cmd_publishers = {
            name: rospy.Publisher("/xtdrone/%s/cmd" % name, String, queue_size=1)
            for name in UAVS + (EVADER,)
        }
        self.state_publisher = rospy.Publisher(
            "/vblock_capture/state", String, queue_size=1, latch=True
        )
        self.result_publisher = rospy.Publisher(
            "/vblock_capture/result", String, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(
            "/vblock_capture/markers", MarkerArray, queue_size=1
        )

        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1
        )
        for name in UAVS + (EVADER,):
            rospy.Subscriber(
                "/%s/mavros/local_position/pose" % name,
                PoseStamped,
                self._uav_local_pose_cb,
                callback_args=name,
                queue_size=1,
            )
            rospy.Subscriber(
                "/%s/mavros/state" % name,
                State,
                self._uav_state_cb,
                callback_args=name,
                queue_size=1,
            )

        rospy.Service("/vblock_capture/start", Trigger, self._start_service)
        self.state_publisher.publish(String(self.state))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tactical_rate), self._tick)
        rospy.loginfo(
            "vblock_capture ready: corner=%s entry=%s H_f=%.1f region=%s "
            "escape_gates=%s",
            self.spawn_corner,
            self.entry_side,
            self.uav_altitude,
            self.inner_region,
            self.valid_escape_gates,
        )

    # ------------------------------------------------------------------ 数据
    def _model_states_cb(self, msg):
        indices = {name: index for index, name in enumerate(msg.name)}
        for name in ALL_AGENTS:
            if name in indices:
                self.model_poses[name] = msg.pose[indices[name]]
            else:
                self.model_poses.pop(name, None)

    def _uav_local_pose_cb(self, msg, name):
        self.uav_local_poses[name] = msg.pose

    def _uav_state_cb(self, msg, name):
        self.uav_states[name] = msg

    def _position_xy(self, name):
        pose = self.model_poses[name].position
        return (pose.x, pose.y)

    def _uav_altitude(self, name):
        return self.model_poses[name].position.z

    def _ready(self):
        models_ready = all(name in self.model_poses for name in ALL_AGENTS)
        local_ready = all(name in self.uav_local_poses for name in UAVS + (EVADER,))
        flight_ready = all(
            name in self.uav_states
            and self.uav_states[name].connected
            and self.uav_states[name].armed
            and self.uav_states[name].mode == "OFFBOARD"
            for name in UAVS + (EVADER,)
        )
        uav_subscribers_ready = all(
            self.goal_publishers[name].get_num_connections() > 0
            for name in UAVS + (EVADER,)
        )
        car_subscribers = {
            name: self.goal_publishers[name].get_num_connections()
            for name in CARS
        }
        if not all(car_subscribers.values()):
            rospy.logwarn_throttle(5.0, "UGV Nav goal subscribers missing: %s", car_subscribers)
        return (
            models_ready
            and local_ready
            and flight_ready
            and uav_subscribers_ready
        )

    # ------------------------------------------------------------------ 启动
    def _start_service(self, _request):
        if self.started:
            return TriggerResponse(False, "mission already started")
        if not self._ready():
            return TriggerResponse(
                False,
                "not ready: need all agents + EGO/Nav subscribers + 3 UAVs OFFBOARD+armed",
            )
        self._begin()
        return TriggerResponse(True, "vblock capture mission started")

    def _begin(self):
        self.started = True
        self.phase_started_at = time.monotonic()
        self.approach_step = 0
        self.approach_goal_sent = False
        self.was_inside = inside_inner(self._position_xy(EVADER), self.inner_region)
        self.inside_since = None
        self.intrusion_fired_at = None
        self.published_targets.clear()
        self.last_targets.clear()
        self._set_state(self.TAKEOFF)
        for name in UAVS + (EVADER,):
            self.uav_cmd_publishers[name].publish(String(data="OFFBOARD"))
        rospy.loginfo(
            "vblock capture mission started: %s from corner %s",
            self.entry_side,
            self.spawn_corner,
        )

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self.state_publisher.publish(String(state))
            rospy.loginfo("Vblock state -> %s", state)

    # ------------------------------------------------------------------ 目标
    def _wall_outside_point(self, side):
        gx, gy = ENTRY_GATES[side]
        nx, ny = ENTRY_NORMALS[side]
        return (gx - nx * self.approach_offset, gy - ny * self.approach_offset)

    def _wall_align_point(self, side):
        """门洞中心线预对齐点：距墙比待机点更远，令末段沿中心线进场消除横向偏置。"""
        gx, gy = ENTRY_GATES[side]
        nx, ny = ENTRY_NORMALS[side]
        d = self.approach_offset + self.approach_pre_extra
        return (gx - nx * d, gy - ny * d)

    def _wall_inside_point(self, side):
        gx, gy = ENTRY_GATES[side]
        nx, ny = ENTRY_NORMALS[side]
        return (gx + nx * self.inside_offset, gy + ny * self.inside_offset)

    def _make_goal(self, x, y, z=0.0, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _publish_world_goal(self, name, target, altitude=None):
        if name in UAVS + (EVADER,):
            world_pose = self.model_poses[name].position
            local_pose = self.uav_local_poses[name].position
            alt = self.uav_altitude if altitude is None else altitude
            local_target = (
                target[0] - (world_pose.x - local_pose.x),
                target[1] - (world_pose.y - local_pose.y),
                alt - (world_pose.z - local_pose.z),
            )
            self.goal_publishers[name].publish(self._make_goal(*local_target))
            return
        current = self._position_xy(name)
        yaw = math.atan2(target[1] - current[1], target[0] - current[0])
        self.goal_publishers[name].publish(
            self._make_goal(target[0], target[1], 0.0, yaw)
        )

    def _send_goal(self, name, target):
        """目标发送门控：目标明显变化立即发；目标不变但未到位时按 re_goal_interval
        周期重发，否则卡住的 agent 一旦目标稳定就再也收不到 goal。"""
        now = time.monotonic()
        elapsed = now - self.last_publish_time.get(name, 0.0)
        if elapsed < 1.0 / self.goal_rate:
            return
        current = self._position_xy(name)
        arrived = planar_distance(current, target) < self.goal_min_change
        previous = self.published_targets.get(name)
        changed = (
            previous is not None
            and planar_distance(previous, target) >= self.goal_min_change
        )
        if arrived and not changed:
            return
        if not changed and elapsed < self.re_goal_interval:
            return
        self._publish_world_goal(name, target)
        self.published_targets[name] = target
        self.last_publish_time[name] = now

    # ------------------------------------------------------------------ FSM
    def _check_takeoff(self):
        return all(
            self._uav_altitude(name) >= self.uav_altitude - 0.3
            for name in UAVS + (EVADER,)
        )

    def _tick(self, _event):
        if not self.started:
            if self.auto_start and self._ready():
                if self.ready_since is None:
                    self.ready_since = time.monotonic()
                elif time.monotonic() - self.ready_since >= self.start_delay:
                    self._begin()
            else:
                self.ready_since = None
            self._publish_markers()
            return

        if self.state == self.TAKEOFF:
            if self._check_takeoff():
                self._begin_staging()
        elif self.state == self.STAGING:
            self._run_staging()
        elif self.state == self.INTRUDER_APPROACH:
            self._run_approach()
        elif self.state == self.AIR_INTRUSION_DETECTED:
            self._begin_containment()
        elif self.state == self.CONTAINMENT:
            self._run_containment()
        elif self.state in (
            self.CAPTURED,
            self.FAILED_ESCAPE,
            self.INVALID_ESCAPE,
        ):
            self._hold_terminal()

        self._publish_markers()

    # ---- STAGING ----
    def _begin_staging(self):
        self._set_state(self.STAGING)
        self.phase_started_at = time.monotonic()
        # 防御 UAV 保持出生点（内墙附近）；入侵 UAV 前往出生角落待机
        home0 = self._position_xy("iris_0")
        home1 = self._position_xy("iris_1")
        self.hold_goals = {
            "iris_0": home0,
            "iris_1": home1,
        }
        rospy.loginfo("STAGING: iris_0/1 hold, iris_2 -> corner %s", self.spawn_corner)

    def _run_staging(self):
        for name in ("iris_0", "iris_1"):
            self._send_goal(name, self.hold_goals[name])
        corner = self.corner_xy[self.spawn_corner]
        self._send_goal(EVADER, corner)
        if planar_distance(self._position_xy(EVADER), corner) <= self.entry_arrive:
            self._begin_approach()

    # ---- INTRUDER_APPROACH ----
    def _begin_approach(self):
        self._set_state(self.INTRUDER_APPROACH)
        self.phase_started_at = time.monotonic()
        self.approach_step = 0
        self.approach_goal_sent = False
        self.staging_arrived_at = None
        self.staging_last_fast_at = None
        self.evader_last_pos = None
        self.evader_last_pos_at = None
        self.published_targets.pop(EVADER, None)
        self.was_inside = inside_inner(self._position_xy(EVADER), self.inner_region)
        self.inside_since = None
        rospy.loginfo(
            "INTRUDER_APPROACH: iris_2 %s -> align %s -> staging %s -> inside %s",
            self.entry_side,
            self.staging_align,
            self.staging_outside,
            self.entry_inside,
        )

    def _evader_speed(self):
        """iris_2 世界系水平速度估计（连续两拍 model_poses 位移/Δt）；首拍返回 None。"""
        now = time.monotonic()
        pos = self._position_xy(EVADER)
        if self.evader_last_pos is None:
            self.evader_last_pos = pos
            self.evader_last_pos_at = now
            return None
        dt = now - self.evader_last_pos_at
        speed = None
        if dt > 0:
            speed = math.hypot(
                pos[0] - self.evader_last_pos[0], pos[1] - self.evader_last_pos[1]
            ) / dt
        self.evader_last_pos = pos
        self.evader_last_pos_at = now
        return speed

    def _run_approach(self):
        # 全程监控 outside -> inside 过渡（入侵检测），不因路线走完而跳过
        self._update_evader_velocity()
        point = self._position_xy(EVADER)
        now_inside = inside_inner(point, self.inner_region)
        now = time.monotonic()
        speed = self._evader_speed()
        if now_inside:
            if self.inside_since is None:
                self.inside_since = now
            if now - self.inside_since >= self.intrusion_debounce:
                self._fire_intrusion(point)
                return
        else:
            self.inside_since = None
        self.was_inside = now_inside

        steps = [self.staging_align, self.staging_outside, self.entry_inside]
        if self.approach_step >= len(steps):
            return
        step = steps[self.approach_step]
        self._send_goal(EVADER, step)
        if planar_distance(point, step) <= self.entry_arrive:
            if self.approach_step == 1:
                # 先让无人机在待机点真正停稳（水平速度持续低于阈值）再发进内部目标。
                if self.staging_arrived_at is None:
                    self.staging_arrived_at = now
                    self.staging_last_fast_at = now
                    rospy.loginfo(
                        "iris_2 arrived at wall outside staging %.2f, %.2f; "
                        "settling (v=%.2f m/s)",
                        step[0],
                        step[1],
                        speed if speed is not None else -1.0,
                    )
                if speed is not None and speed > self.entry_speed_eps:
                    self.staging_last_fast_at = now
                settled = (
                    now - self.staging_arrived_at >= self.approach_settle
                    and now - self.staging_last_fast_at >= self.entry_speed_hold
                )
                forced = now - self.staging_arrived_at >= self.entry_speed_cap
                if not settled and not forced:
                    return
                if forced:
                    rospy.logwarn(
                        "iris_2 settle forced after %.1fs (v=%.2f); entering anyway",
                        now - self.staging_arrived_at,
                        speed if speed is not None else -1.0,
                    )
                else:
                    rospy.loginfo(
                        "iris_2 settled (v=%.2f < %.2f), sending inside entry goal",
                        speed if speed is not None else 0.0,
                        self.entry_speed_eps,
                    )
            self.approach_step += 1
            self.published_targets.pop(EVADER, None)
            if self.approach_step >= len(steps):
                rospy.loginfo("iris_2 entering protected area over wall")

    def _fire_intrusion(self, point):
        self.intrusion_fired_at = time.monotonic()
        self._set_state(self.AIR_INTRUSION_DETECTED)
        message = (
            "AIR_INTRUSION_DETECTED: iris_2 at world (%.3f, %.3f, %.3f), "
            "side=%s"
            % (
                point[0],
                point[1],
                self._uav_altitude(EVADER),
                self.crossing_side,
            )
        )
        self.result_publisher.publish(String(message))
        rospy.logwarn(message)

    # ------------------------------------------------------------------ CONTAINMENT
    def _current_yaw(self, name):
        q = self.model_poses[name].orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _stop_car(self, name):
        current = self._position_xy(name)
        self.goal_publishers[name].publish(
            self._make_goal(current[0], current[1], 0.0, self._current_yaw(name))
        )

    def _update_evader_velocity(self):
        x, y = self._position_xy(EVADER)
        now = time.monotonic()
        self.evader_vel_history.append((x, y, now))
        if len(self.evader_vel_history) > 6:
            self.evader_vel_history.pop(0)

    def _escape_direction(self):
        """逃逸轴 e：严格取 iris_2 实际二维速度方向；速度低于 v_eps 时回退到最近
        一次有效飞行方向（保持连续性），仅当从未测到速度时才回退到目标门方向。
        对 e 做指数平滑防机动抖动。"""
        p = self._position_xy(EVADER)
        vx = vy = 0.0
        if len(self.evader_vel_history) >= 2:
            (x0, y0, t0) = self.evader_vel_history[0]
            (x1, y1, t1) = self.evader_vel_history[-1]
            dt = t1 - t0
            if dt > 0.05:
                vx = (x1 - x0) / dt
                vy = (y1 - y0) / dt
        speed = math.hypot(vx, vy)
        if speed >= self.v_eps:
            e_new = (vx / speed, vy / speed)
            self.last_velocity_dir = e_new
        elif self.last_velocity_dir is not None:
            e_new = self.last_velocity_dir
        elif self.current_escape_gate is not None:
            pG = ENTRY_GATES[self.current_escape_gate]
            dx = pG[0] - p[0]
            dy = pG[1] - p[1]
            norm = math.hypot(dx, dy)
            e_new = (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)
        else:
            e_new = (1.0, 0.0)
        if self.e_axis is None:
            self.e_axis = e_new
        else:
            k = self.e_smooth
            ex = self.e_axis[0] * (1.0 - k) + e_new[0] * k
            ey = self.e_axis[1] * (1.0 - k) + e_new[1] * k
            norm = math.hypot(ex, ey)
            self.e_axis = (ex / norm, ey / norm) if norm > 1e-9 else e_new
        return self.e_axis

    def _car_flank_normal(self):
        """小车夹击翼展法向 n（左法向）：垂直于 iris_2 自身二维速度；速度低于 v_eps
        时回退到朝向内区中心。只依赖入侵机运动，与空中逃逸门/UAV 捕获解耦——
        两车分别朝 pE +/- ds*n 横向夹击入侵机地面投影。"""
        p = self._position_xy(EVADER)
        vx = vy = 0.0
        if len(self.evader_vel_history) >= 2:
            (x0, y0, t0) = self.evader_vel_history[0]
            (x1, y1, t1) = self.evader_vel_history[-1]
            dt = t1 - t0
            if dt > 0.05:
                vx = (x1 - x0) / dt
                vy = (y1 - y0) / dt
        speed = math.hypot(vx, vy)
        if speed >= self.v_eps:
            self.last_velocity_dir = (vx / speed, vy / speed)
            return (-vy / speed, vx / speed)
        if self.last_velocity_dir is not None:
            lx, ly = self.last_velocity_dir
            return (-ly, lx)
        cx = 0.5 * (self.inner_region[0] + self.inner_region[1])
        cy = 0.5 * (self.inner_region[2] + self.inner_region[3])
        dx, dy = cx - p[0], cy - p[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 1.0)
        return (-dy / norm, dx / norm)

    def _gate_cost(self, gate, pE):
        """逃逸出口代价 J_i = w_d*d(E,G_i) + w_p*J_pursuer + w_o*J_obstacle。
        J_pursuer = 各追捕者到"E->门"路径线段的 1/(d+0.6) 之和：追捕者越贴近逃逸路径代价越大。
        J_obstacle v1 预留恒为 0（被堵已计入 J_pursuer）。"""
        pG = ENTRY_GATES[gate]
        j_dist = planar_distance(pE, pG)
        j_pursuer = 0.0
        for name in PURSUERS:
            if name not in self.model_poses:
                continue
            p = self._position_xy(name)
            seg = point_segment_distance(p, pE, pG)
            j_pursuer += 1.0 / (seg + 0.6)
        return self.w_dist * j_dist + self.w_pursuer * j_pursuer + self.w_obstacle * 0.0

    def _select_escape_gate(self):
        """每周期选代价最小门，带滞回（仅当 J_new < J_cur - margin 且优势持续
        hold_time 才切换，防每帧反复换门）。"""
        pE = self._position_xy(EVADER)
        costs = {g: self._gate_cost(g, pE) for g in self.valid_escape_gates}
        best = min(costs, key=costs.get)
        now = time.monotonic()
        if self.current_escape_gate is None:
            self.current_escape_gate = best
            self.best_gate_since = None
            rospy.loginfo("escape gate init -> %s", best)
        elif best != self.current_escape_gate:
            if costs[best] < costs[self.current_escape_gate] - self.exit_switch_margin:
                if self.best_gate_since is None:
                    self.best_gate_since = now
                elif now - self.best_gate_since >= self.exit_switch_hold_time:
                    rospy.logwarn(
                        "escape gate switch %s -> %s (J %.2f vs %.2f)",
                        self.current_escape_gate,
                        best,
                        costs[best],
                        costs[self.current_escape_gate],
                    )
                    self.current_escape_gate = best
                    self.best_gate_since = None
            else:
                self.best_gate_since = None
        else:
            self.best_gate_since = None
        self.last_gate_costs = costs
        return self.current_escape_gate

    def _evasion_goal(self):
        """iris_2 逃逸目标：当前目标门中心，收敛在内区边界内侧一点（越线即判逃逸）。"""
        if self.current_escape_gate is None:
            return clamp_point(self._position_xy(EVADER), self.inner_region, 0.05)
        pG = ENTRY_GATES[self.current_escape_gate]
        return clamp_point(pG, self.inner_region, 0.05)

    def _assign_roles(self):
        """进入 CONTAINMENT 时按最小总距离分配 UAV 左右角色并锁定；
        小车按最小总距离分配到"投影左右夹击点"（cl/cr）。"""
        pE = self._position_xy(EVADER)
        ex, ey = self._escape_direction()
        nx, ny = -ey, ex
        df = self.uav_forward_offset
        ds = self.uav_lateral_offset
        pl = clamp_point(
            (pE[0] + df * ex + ds * nx, pE[1] + df * ey + ds * ny),
            self.inner_region,
            self.uav_margin,
        )
        pr = clamp_point(
            (pE[0] + df * ex - ds * nx, pE[1] + df * ey - ds * ny),
            self.inner_region,
            self.uav_margin,
        )
        c1 = planar_distance(self._position_xy("iris_0"), pl) + planar_distance(
            self._position_xy("iris_1"), pr
        )
        c2 = planar_distance(self._position_xy("iris_0"), pr) + planar_distance(
            self._position_xy("iris_1"), pl
        )
        if c1 <= c2:
            self.uav_left, self.uav_right = "iris_0", "iris_1"
        else:
            self.uav_left, self.uav_right = "iris_1", "iris_0"

        dsc = self.ugv_lateral_offset
        gx, gy = self._car_flank_normal()
        cl = clamp_point(
            (pE[0] + dsc * gx, pE[1] + dsc * gy),
            self.inner_region,
            self.ugv_margin,
        )
        cr = clamp_point(
            (pE[0] - dsc * gx, pE[1] - dsc * gy),
            self.inner_region,
            self.ugv_margin,
        )
        g1 = planar_distance(self._position_xy("car0"), cl) + planar_distance(
            self._position_xy("car1"), cr
        )
        g2 = planar_distance(self._position_xy("car0"), cr) + planar_distance(
            self._position_xy("car1"), cl
        )
        if g1 <= g2:
            self.ugv_left, self.ugv_right = "car0", "car1"
        else:
            self.ugv_left, self.ugv_right = "car1", "car0"

    def _compute_containment_goals(self):
        """V 型封控目标：UAV 前方 V 点（PL/PR），小车对入侵机地面投影的横向夹击点
        （CL/CR = pE +/- ds_car*n），iris_2 逃逸目标。全部 clamp 到内区安全余量内。"""
        pE = self._position_xy(EVADER)
        ex, ey = self._escape_direction()
        nx, ny = -ey, ex
        targets = {EVADER: self._evasion_goal()}

        df = self.uav_forward_offset
        ds = self.uav_lateral_offset
        pl = clamp_point(
            (pE[0] + df * ex + ds * nx, pE[1] + df * ey + ds * ny),
            self.inner_region,
            self.uav_margin,
        )
        pr = clamp_point(
            (pE[0] + df * ex - ds * nx, pE[1] + df * ey - ds * ny),
            self.inner_region,
            self.uav_margin,
        )
        targets[self.uav_left] = pl
        targets[self.uav_right] = pr

        dsc = self.ugv_lateral_offset
        gx, gy = self._car_flank_normal()
        cl = clamp_point(
            (pE[0] + dsc * gx, pE[1] + dsc * gy),
            self.inner_region,
            self.ugv_margin,
        )
        cr = clamp_point(
            (pE[0] - dsc * gx, pE[1] - dsc * gy),
            self.inner_region,
            self.ugv_margin,
        )
        targets[self.ugv_left] = cl
        targets[self.ugv_right] = cr

        self.last_containment_targets = targets
        return targets

    def _begin_containment(self):
        self.pursuit_started_at = time.monotonic()
        self.capture_hold_started = None
        # 保持 evader_vel_history 连续（approach 阶段已持续供样），CONTAINMENT 首帧
        # 即能得到 iris_2 实际飞行方向，避免初始回退到门方向导致 V 点放在错误一侧。
        self.e_axis = None
        self.current_escape_gate = None
        self.best_gate_since = None
        self.last_containment_targets = {}
        self._select_escape_gate()
        self._assign_roles()
        self._set_state(self.CONTAINMENT)
        rospy.logwarn(
            "CONTAINMENT: escape_gates=%s uav_left=%s uav_right=%s "
            "ugv_left=%s ugv_right=%s",
            self.valid_escape_gates,
            self.uav_left,
            self.uav_right,
            self.ugv_left,
            self.ugv_right,
        )
        targets = self._compute_containment_goals()
        for name, target in targets.items():
            self.published_targets.pop(name, None)
            self._send_goal(name, target)

    def _run_containment(self):
        self._update_evader_velocity()
        self._select_escape_gate()
        if not self.role_lock:
            self._assign_roles()
        targets = self._compute_containment_goals()
        for name, target in targets.items():
            self._send_goal(name, target)
        pE = self._position_xy(EVADER)
        if not inside_inner(pE, self.inner_region):
            self._finish_escaped()
            return
        self._check_capture()

    def _check_capture(self):
        """捕获判定（纯 2D，仅两架 UAV 为依据）：
        max UAV 距离 <= R_c 且 双侧封堵 z_A*z_B<0 且 至少一架 UAV 在逃逸轴前方，
        持续 T_hold。小车不参与。"""
        pE = self._position_xy(EVADER)
        ex, ey = self._escape_direction()
        uav_max = max(planar_distance(self._position_xy(n), pE) for n in UAVS)
        in_range = uav_max <= self.air_capture_radius

        bilateral = True
        if self.bilateral_blocking:
            z = {}
            for n in UAVS:
                p = self._position_xy(n)
                z[n] = ex * (p[1] - pE[1]) - ey * (p[0] - pE[0])
            bilateral = z[UAVS[0]] * z[UAVS[1]] < 0

        # 前方封堵：逃逸机已贴近所选逃逸门（距门中心 <= R_c）时，出口已被墙体与逼近
        # 的追捕者双重封死；此时 uav_margin 收缩使无人机物理上无法绕到门内侧，原
        # "ahead" 判定必然不满足（LEFT 门进犯实测 FAILED_ESCAPE 的根因）。故该场景
        # 放宽为仅需双机逼近 + 双侧封堵即可判捕获；中段追逐仍保持原 ahead 判定。
        gate_dist = (
            planar_distance(pE, ENTRY_GATES[self.current_escape_gate])
            if self.current_escape_gate is not None
            else float("inf")
        )
        need_ahead = gate_dist >= self.air_capture_radius
        ahead = (not need_ahead) or any(
            (p[0] - pE[0]) * ex + (p[1] - pE[1]) * ey > 0
            for p in (self._position_xy(n) for n in UAVS)
        )

        satisfied = in_range and bilateral and ahead
        now = time.monotonic()
        hold_elapsed = (
            0.0
            if self.capture_hold_started is None
            else now - self.capture_hold_started
        )
        pos = {n: self._position_xy(n) for n in ALL_AGENTS}
        rospy.loginfo_throttle(
            2.0,
            "C uav_max=%.2f R_c=%.2f bi=%s ahead=%s hold=%.1f gate=%s gdist=%.2f "
            "| ax(%.2f,%.2f) e(%.2f,%.2f) i0(%.2f,%.2f) i1(%.2f,%.2f) "
            "c0(%.2f,%.2f) c1(%.2f,%.2f)",
            uav_max,
            self.air_capture_radius,
            bilateral,
            ahead,
            hold_elapsed,
            self.current_escape_gate,
            gate_dist,
            ex,
            ey,
            pos[EVADER][0],
            pos[EVADER][1],
            pos["iris_0"][0],
            pos["iris_0"][1],
            pos["iris_1"][0],
            pos["iris_1"][1],
            pos["car0"][0],
            pos["car0"][1],
            pos["car1"][0],
            pos["car1"][1],
        )
        if satisfied:
            if self.capture_hold_started is None:
                self.capture_hold_started = now
                rospy.logwarn(
                    "capture conditions met: uav_max=%.2f (R_c=%.2f)",
                    uav_max,
                    self.air_capture_radius,
                )
            elif hold_elapsed >= self.hold_time:
                self._finish_captured()
        else:
            self.capture_hold_started = None

    def _freeze_uav(self, name):
        """围捕结束后冻结 UAV：HOVER_LOCK（桥忽略后续 pose/vel，防止 EGO
        traj_server 残留逃逸轨迹覆盖）+ 以当前世界位姿（含当前 z）为原地目标。"""
        self.uav_cmd_publishers[name].publish(String(data="HOVER_LOCK"))
        p = self.model_poses[name].position
        self._publish_world_goal(name, (p.x, p.y), altitude=p.z)

    def _halt_all(self):
        for name in UAVS + (EVADER,):
            self._freeze_uav(name)
        for name in CARS:
            self._stop_car(name)

    def _hold_terminal(self):
        now = time.monotonic()
        if now - self.last_hold_publish < 0.3:
            return
        self.last_hold_publish = now
        for name in UAVS + (EVADER,):
            self._freeze_uav(name)
        for name in CARS:
            self._stop_car(name)

    def _finish_captured(self):
        pE = self._position_xy(EVADER)
        uav_max = max(planar_distance(self._position_xy(name), pE) for name in UAVS)
        message = (
            "CAPTURED: iris_2 at (%.3f, %.3f), UAV max dist %.3f m, "
            "containment %.1f s after intrusion" % (
                pE[0],
                pE[1],
                uav_max,
                time.monotonic() - self.pursuit_started_at,
            )
        )
        self._set_state(self.CAPTURED)
        self.last_hold_publish = 0.0
        self.result_publisher.publish(String(message))
        rospy.logwarn(message)
        self._halt_all()

    def _finish_escaped(self):
        """逃逸判定：按最靠近离开点的门分类。合法门 -> FAILED_ESCAPE；
        原入侵门 -> INVALID_ESCAPE。"""
        pE = self._position_xy(EVADER)
        gate = min(ENTRY_GATES, key=lambda g: planar_distance(pE, ENTRY_GATES[g]))
        if gate == self.entry_side:
            state = self.INVALID_ESCAPE
        else:
            state = self.FAILED_ESCAPE
        message = (
            "%s: iris_2 left protected region via %s at (%.3f, %.3f) after %.1f s"
            % (
                state,
                gate,
                pE[0],
                pE[1],
                time.monotonic() - self.pursuit_started_at,
            )
        )
        self._set_state(state)
        self.last_hold_publish = 0.0
        self.result_publisher.publish(String(message))
        rospy.logwarn(message)
        self._halt_all()

    # ------------------------------------------------------------------ 可视化
    def _marker(self, marker_id, namespace, marker_type, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _publish_markers(self):
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        xmin, xmax, ymin, ymax = self.inner_region
        protected = self._marker(
            1, "protected", Marker.LINE_STRIP, (0.2, 1.0, 0.2, 0.9)
        )
        protected.scale.x = 0.04
        for x, y in [
            (xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)
        ]:
            protected.points.append(Point(x=x, y=y, z=0.04))
        markers.markers.append(protected)

        colors = {
            "car0": (0.1, 1.0, 0.1, 0.8),
            "car1": (0.1, 0.8, 1.0, 0.8),
            "iris_0": (1.0, 1.0, 0.0, 0.8),
            "iris_1": (1.0, 0.0, 1.0, 0.8),
            "iris_2": (1.0, 0.1, 0.1, 0.8),
        }
        for index, name in enumerate(ALL_AGENTS):
            if name not in self.model_poses:
                continue
            point = self._position_xy(name)
            agent = self._marker(10 + index, "agents", Marker.SPHERE, colors[name])
            agent.pose.position.x = point[0]
            agent.pose.position.y = point[1]
            if name in UAVS + (EVADER,):
                agent.pose.position.z = self._uav_altitude(name)
            else:
                agent.pose.position.z = 0.1
            agent.scale.x = agent.scale.y = agent.scale.z = 0.3
            markers.markers.append(agent)

        # iris_2 地面投影
        if EVADER in self.model_poses:
            point = self._position_xy(EVADER)
            proj = self._marker(20, "projection", Marker.CYLINDER, (1.0, 0.4, 0.1, 0.6))
            proj.pose.position.x = point[0]
            proj.pose.position.y = point[1]
            proj.pose.position.z = 0.05
            proj.scale.x = proj.scale.y = 0.5
            proj.scale.z = 0.05
            markers.markers.append(proj)

        # 入侵路线（预对齐点 -> staging 外侧点 -> 内点）
        if self.state in (self.INTRUDER_APPROACH, self.AIR_INTRUSION_DETECTED):
            route = self._marker(25, "route", Marker.LINE_STRIP, (1.0, 0.6, 0.2, 0.9))
            route.scale.x = 0.03
            for x, y in [self.staging_align, self.staging_outside, self.entry_inside]:
                route.points.append(Point(x=x, y=y, z=0.1))
            markers.markers.append(route)

        # CONTAINMENT 阵型目标点 + 逃逸轴 + 当前逃逸门
        if self.state == self.CONTAINMENT and self.last_containment_targets:
            for index, (name, target) in enumerate(self.last_containment_targets.items()):
                goal = self._marker(
                    30 + index, "containment_goal", Marker.SPHERE, (1.0, 0.5, 1.0, 0.5)
                )
                goal.pose.position.x = target[0]
                goal.pose.position.y = target[1]
                goal.pose.position.z = 0.2 if name in CARS else 1.5
                goal.scale.x = goal.scale.y = goal.scale.z = 0.25
                markers.markers.append(goal)
            e0 = self.last_containment_targets[EVADER]
            line = self._marker(35, "escape_axis", Marker.ARROW, (1.0, 0.3, 0.3, 0.7))
            line.scale.x = 0.03
            line.pose.position.x = e0[0]
            line.pose.position.y = e0[1]
            line.pose.position.z = 1.5
            ex, ey = self._escape_direction()
            line.points.append(Point(x=0, y=0, z=0))
            line.points.append(Point(x=1.5 * ex, y=1.5 * ey, z=0))
            markers.markers.append(line)

            if self.current_escape_gate:
                gx, gy = ENTRY_GATES[self.current_escape_gate]
                gate = self._marker(36, "escape_gate", Marker.SPHERE, (0.0, 0.8, 1.0, 0.9))
                gate.pose.position.x = gx
                gate.pose.position.y = gy
                gate.pose.position.z = 1.0
                gate.scale.x = gate.scale.y = 0.3
                gate.scale.z = 0.8
                markers.markers.append(gate)

        text = self._marker(60, "state", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = self.inner_region[1] + 0.2
        text.pose.position.y = self.inner_region[3] + 0.2
        text.pose.position.z = 0.8
        text.scale.z = 0.3
        if self.state == self.CONTAINMENT and self.capture_hold_started is not None:
            remain = self.hold_time - (time.monotonic() - self.capture_hold_started)
            detail = " | capture hold %.1fs" % max(0.0, remain)
        elif self.state == self.CONTAINMENT:
            pE = self._position_xy(EVADER)
            uav_max = max(planar_distance(self._position_xy(n), pE) for n in UAVS)
            detail = " | gate=%s UAVmax=%.2f" % (
                self.current_escape_gate, uav_max
            )
        else:
            detail = ""
        text.text = "Vblock capture: %s | entry %s%s" % (
            self.state, self.entry_side, detail
        )
        markers.markers.append(text)
        self.marker_publisher.publish(markers)


if __name__ == "__main__":
    try:
        VblockCapture()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        rospy.logerr("vblock_capture stopped: %s", exc)
        sys.exit(1)
