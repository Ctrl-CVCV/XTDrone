#!/usr/bin/env python3
"""Air-intruder pursuit: two defender UAVs + two defender UGVs vs one intruder UAV.

iris_2 (intruder UAV) spawns in one of two outer-wall corners, takes off to the
uniform altitude H_f, stages at its corner, then flies over the real 3m inner
wall into the protected area.  When its world XY transitions outside->inside the
inner region, AIR_INTRUSION_DETECTED fires.

Phase 3 scope: WAITING -> TAKEOFF -> STAGING -> INTRUDER_APPROACH ->
AIR_INTRUSION_DETECTED.  Later phases (5-9) extend PURSUIT / ENCIRCLED /
CAPTURED / ESCAPED behind this node on the same FSM.

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
}


def planar_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def inside_inner(point, region):
    xmin, xmax, ymin, ymax = region
    return xmin <= point[0] <= xmax and ymin <= point[1] <= ymax


# --------------------------------------------------------------------------
# 几何助手（与 voronoi_air_ground_pursuit.py 同源，Phase 5-8 复用）
# --------------------------------------------------------------------------
def clip_polygon_halfplane(polygon, a, b, c, eps=1e-9):
    """Clip a polygon by a*x + b*y <= c (Sutherland-Hodgman)."""
    if not polygon:
        return []
    result = []
    previous = polygon[-1]
    previous_f = a * previous[0] + b * previous[1] - c
    previous_inside = previous_f <= eps
    for current in polygon:
        current_f = a * current[0] + b * current[1] - c
        current_inside = current_f <= eps
        if current_inside != previous_inside:
            denominator = previous_f - current_f
            if abs(denominator) > 1e-12:
                t = previous_f / denominator
                result.append(
                    (
                        previous[0] + t * (current[0] - previous[0]),
                        previous[1] + t * (current[1] - previous[1]),
                    )
                )
        if current_inside:
            result.append(current)
        previous = current
        previous_f = current_f
        previous_inside = current_inside
    return deduplicate_polygon(result)


def deduplicate_polygon(polygon, tolerance=1e-8):
    result = []
    for point in polygon:
        if not result or math.hypot(
            point[0] - result[-1][0], point[1] - result[-1][1]
        ) > tolerance:
            result.append(point)
    if len(result) > 1 and math.hypot(
        result[0][0] - result[-1][0], result[0][1] - result[-1][1]
    ) <= tolerance:
        result.pop()
    return result


def bounded_voronoi_cell(points, index, bounds):
    """Return one Voronoi cell clipped to rectangular bounds."""
    xmin, xmax, ymin, ymax = bounds
    polygon = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    px, py = points[index]
    for other_index, (qx, qy) in enumerate(points):
        if other_index == index:
            continue
        a = 2.0 * (qx - px)
        b = 2.0 * (qy - py)
        c = qx * qx + qy * qy - px * px - py * py
        if abs(a) + abs(b) < 1e-12:
            continue
        polygon = clip_polygon_halfplane(polygon, a, b, c)
        if not polygon:
            break
    return polygon


def polygon_centroid(polygon):
    if len(polygon) < 3:
        if not polygon:
            return None
        return (
            sum(point[0] for point in polygon) / len(polygon),
            sum(point[1] for point in polygon) / len(polygon),
        )
    twice_area = 0.0
    centroid_x = 0.0
    centroid_y = 0.0
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        cross = start[0] * end[1] - end[0] * start[1]
        twice_area += cross
        centroid_x += (start[0] + end[0]) * cross
        centroid_y += (start[1] + end[1]) * cross
    if abs(twice_area) < 1e-10:
        return None
    return (centroid_x / (3.0 * twice_area), centroid_y / (3.0 * twice_area))


def clamp_point(point, bounds, margin):
    xmin, xmax, ymin, ymax = bounds
    return (
        max(xmin + margin, min(xmax - margin, point[0])),
        max(ymin + margin, min(ymax - margin, point[1])),
    )


def convex_hull(points):
    """Monotone-chain convex hull (CCW), dedup duplicates."""
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def polygon_area(polygon):
    if len(polygon) < 3:
        return 0.0
    return 0.5 * abs(
        sum(
            polygon[i][0] * polygon[(i + 1) % len(polygon)][1]
            - polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
            for i in range(len(polygon))
        )
    )


def point_in_polygon(point, polygon):
    """Ray-casting point-in-polygon test (works for any simple polygon)."""
    x, y = point
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def point_polygon_distance(point, polygon):
    """点到凸多边形边界的最近距离；点在内部时返回 0。"""
    if point_in_polygon(point, polygon):
        return 0.0
    x, y = point
    best = float("inf")
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        cx, cy = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(x - cx, y - cy))
    return best


class AirIntruderPursuit:
    WAITING = "WAITING"
    TAKEOFF = "TAKEOFF"
    STAGING = "STAGING"
    INTRUDER_APPROACH = "INTRUDER_APPROACH"
    AIR_INTRUSION_DETECTED = "AIR_INTRUSION_DETECTED"
    PURSUIT = "PURSUIT"
    ENCIRCLED = "ENCIRCLED"
    CAPTURED = "CAPTURED"
    ESCAPED = "ESCAPED"

    def __init__(self):
        rospy.init_node("air_intruder_pursuit")

        # ---- YAML 参数（launch 已 rosparam load 到 ~）----
        self.uav_altitude = rospy.get_param("~uav/fixed_altitude", 4.0)
        self.wall_height = rospy.get_param("~uav/wall_height", 3.0)

        self.spawn_mode = str(rospy.get_param("~intruder/spawn_mode", "fixed")).lower()
        self.fixed_corner = str(
            rospy.get_param("~intruder/fixed_corner", "CORNER_0")
        ).upper()
        corners = rospy.get_param("~intruder/spawn_corner_0", [5.0, 5.0, -2.367])
        corners1 = rospy.get_param("~intruder/spawn_corner_1", [-5.0, -5.0, 0.796])
        self.corner_xy = {
            "CORNER_0": (corners[0], corners[1]),
            "CORNER_1": (corners1[0], corners1[1]),
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
        # 到达方向的横向动量重规划入场轨迹（CORNER_0 西偏/UP、CORNER_1 东偏/DOWN，实测
        # 可偏出门洞 1m+ 撞墙）。停稳判定：水平速度持续低于 entry_speed_eps 且至少停留
        # approach_settle 秒；entry_speed_cap 为最迟推进上限（防悬停抖动永不达标卡死）。
        self.approach_settle = rospy.get_param("~intruder/approach_settle", 1.2)
        self.entry_speed_eps = rospy.get_param("~intruder/entry_speed_eps", 0.1)
        self.entry_speed_hold = rospy.get_param("~intruder/entry_speed_hold", 0.4)
        self.entry_speed_cap = rospy.get_param("~intruder/entry_speed_cap", 6.0)
        # 门洞中心线预对齐点：比待机点更靠外，使"预对齐点 -> 待机点"这一段沿门中心线进场，
        # 消除 EGO 重规划携带的到达方向横向偏置（CORNER_0 西偏 / CORNER_1 东偏撞门框）。
        self.approach_pre_extra = rospy.get_param("~intruder/approach_pre_extra", 2.0)

        self.tactical_rate = rospy.get_param("~planning/tactical_rate", 15.0)
        self.goal_rate = rospy.get_param("~planning/goal_publish_rate", 3.0)
        self.goal_min_change = rospy.get_param(
            "~planning/goal_change_threshold", 0.3
        )
        self.re_goal_interval = rospy.get_param(
            "~planning/re_goal_interval", 5.0
        )

        self.uav_forward_offset = rospy.get_param("~formation/uav_forward_offset", 1.5)
        self.uav_lateral_offset = rospy.get_param("~formation/uav_lateral_offset", 2.0)
        self.ugv_rear_offset = rospy.get_param("~formation/ugv_rear_offset", 1.2)
        self.ugv_lateral_offset = rospy.get_param("~formation/ugv_lateral_offset", 2.0)
        self.ugv_margin = rospy.get_param("~formation/ugv_margin", 0.4)
        # UAV 贴墙安全余量：H_f=1.33 低于 3m 墙，UAV 无法越墙，目标必须 clamp 在
        # 内区边界内 uav_margin 处，否则封堵/紧阵型目标落在墙面上会被 EGO 规划撞墙。
        # 0.8 较 0.5 更安全：CORNER_0 复测 iris_0 贴墙(0.39m)下降，目标贴 0.5 余量时 EGO 过冲。
        self.uav_margin = rospy.get_param("~formation/uav_margin", 0.8)

        self.air_capture_radius = rospy.get_param("~capture/air_capture_radius", 1.5)
        self.ground_projection_radius = rospy.get_param(
            "~capture/ground_projection_radius", 1.0
        )
        self.hold_time = rospy.get_param("~capture/hold_time", 2.0)
        self.encircle_min_area = rospy.get_param("~capture/encircle_min_area", 0.5)
        # 包围判定边界余量：iris_2 在凸包边界附近振荡时，避免捕获计时因 enc 抖动清零
        self.encircle_margin = rospy.get_param("~capture/encircle_margin", 0.3)

        self.evasion_min_speed = rospy.get_param("~intruder/evasion_min_speed", 0.5)
        self.evasion_cell_margin = rospy.get_param("~intruder/evasion_cell_margin", 0.1)

        self.gate_seal_enabled = rospy.get_param("~gate_seal/enabled", True)
        self.gate_seal_trigger = rospy.get_param("~gate_seal/trigger_dist", 1.8)
        self.gate_seal_inset = rospy.get_param("~gate_seal/inset", 1.0)
        self.gate_seal_dwell = rospy.get_param("~gate_seal/dwell", 1.5)
        self.gate_seal_heading_cos = rospy.get_param("~gate_seal/heading_cos", 0.3)

        self.intrusion_debounce = rospy.get_param("~fsm/intrusion_debounce", 0.3)
        self.auto_start = rospy.get_param("~fsm/auto_start", True)
        self.start_delay = rospy.get_param("~fsm/start_delay", 1.0)
        self.encircle_fallback_delay = rospy.get_param(
            "~fsm/encircle_fallback_delay", 1.5
        )

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
        # 只允许与出生角落相邻的墙作为入侵方向：非相邻方向直线飞越内区会提前触发入侵
        # （EGO 在 H_f 处越过 3m 虚拟墙，XY 投影先进入内区）。跨角落方向留待 Phase 9
        # 走廊路由 + 临时加高墙再支持。
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
        self.staging_last_fast_at = None  # 待机点处上次水平速度仍高于 eps 的时刻（单调时钟）
        self.evader_last_pos = None       # 上一拍 iris_2 世界 XY（水平速度估计）
        self.evader_last_pos_at = None
        self.was_inside = False
        self.inside_since = None
        self.intrusion_fired_at = None
        self.hold_goals = {}  # 需要持续保持的目标（世界 XY）
        self.last_publish_time = {}
        self.published_targets = {}
        self.last_targets = {}
        self.goal_connection_counts = {name: 0 for name in ALL_AGENTS}

        # ---- Phase 5-9 围捕状态 ----
        self.uav_left = "iris_0"
        self.uav_right = "iris_1"
        self.ugv_left = "car0"
        self.ugv_right = "car1"
        self.pursuit_started_at = None
        self.evasion_target = None
        self.evader_vel_history = []
        self.encircle_hull = []
        self.capture_hold_started = None
        self.last_hold_publish = 0.0
        self.last_pursuit_targets = {}
        self.gate_seal_active = False
        self.gate_seal_side = None
        self.gate_seal_blocker = None
        self.last_seal_side = None
        self.last_seal_blocker = None
        self.seal_committed_side = None
        self.seal_commit_until = 0.0
        self.encircle_lost_since = None

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
            "/air_intruder/pursuit/state", String, queue_size=1, latch=True
        )
        self.result_publisher = rospy.Publisher(
            "/air_intruder/pursuit/result", String, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(
            "/air_intruder/pursuit/markers", MarkerArray, queue_size=1
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

        rospy.Service("/air_intruder/pursuit/start", Trigger, self._start_service)
        self.state_publisher.publish(String(self.state))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.tactical_rate), self._tick)
        rospy.loginfo(
            "air_intruder_pursuit ready: corner=%s entry=%s H_f=%.1f region=%s",
            self.spawn_corner,
            self.entry_side,
            self.uav_altitude,
            self.inner_region,
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
        # UAV EGO 必须在线；UGV Nav 缺省时仅警告（Phase 3 车辆只保持位姿）
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
            return TriggerResponse(False, "pursuit already started")
        if not self._ready():
            return TriggerResponse(
                False,
                "not ready: need all agents + EGO/Nav subscribers + 3 UAVs OFFBOARD+armed",
            )
        self._begin()
        return TriggerResponse(True, "air intruder pursuit started")

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
            "air intruder mission started: %s from corner %s",
            self.entry_side,
            self.spawn_corner,
        )

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self.state_publisher.publish(String(state))
            rospy.loginfo("Air intruder state -> %s", state)

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
        """目标发送门控（第 18 节）：目标明显变化立即发；
        目标不变但智能体未到位时按 re_goal_interval 周期重发，
        否则卡住的 agent 一旦目标稳定就再也收不到 goal。"""
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
            self._begin_pursuit()
        elif self.state == self.PURSUIT:
            self._run_pursuit()
        elif self.state == self.ENCIRCLED:
            self._run_encircled()
        elif self.state in (self.CAPTURED, self.ESCAPED):
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
        # 全程监控 outside -> inside 过渡（Phase 4），不因路线走完而跳过
        point = self._position_xy(EVADER)
        now_inside = inside_inner(point, self.inner_region)
        now = time.monotonic()
        speed = self._evader_speed()
        # 注意：was_inside 一旦置 True，后续 inside tick 会走 else 重置 inside_since，
        # 导致 debounce 永不完成。改为：只要持续在内，计时器保持；离开才重置。
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
                # 先让无人机在待机点真正停稳（水平速度持续低于阈值）再发进内部目标：
                # 若在仍带到达方向横向速度时切换目标，EGO 重规划会把该动量带进出场轨迹，
                # 偏出门洞（CORNER_0 西偏 UP / CORNER_1 东偏 DOWN，实测 1m+）撞门框。
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
            "z=%.2f > wall=%.2f, side=%s"
            % (
                point[0],
                point[1],
                self._uav_altitude(EVADER),
                self._uav_altitude(EVADER),
                self.wall_height,
                self.crossing_side,
            )
        )
        self.result_publisher.publish(String(message))
        rospy.logwarn(message)

    # ------------------------------------------------------------------ Phase 5-9
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
        if len(self.evader_vel_history) > 3:
            self.evader_vel_history.pop(0)

    def _escape_direction(self):
        """单位逃逸方向：优先当前 XY 速度，速度过小时用逃逸质心目标方向（第 8 节）。"""
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
        if speed >= self.evasion_min_speed:
            return (vx / speed, vy / speed)
        target = self.evasion_target or p
        dx = target[0] - p[0]
        dy = target[1] - p[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (1.0, 0.0)
        return (dx / norm, dy / norm)

    def _nearest_boundary(self, point):
        """内区四条边界线中离 point 最近的一条，返回 (side, 到边界距离)。"""
        x, y = point
        xmin, xmax, ymin, ymax = self.inner_region
        candidates = (
            ("UP", ymax - y),
            ("DOWN", y - ymin),
            ("RIGHT", xmax - x),
            ("LEFT", x - xmin),
        )
        return min(candidates, key=lambda c: c[1])

    def _seal_point(self, point, side, dist):
        """影子封堵点：point 沿指向边界的方向推进 dist，且停在边界内 uav_margin 处。
        堵头 UAV 仍处在 iris_2 与其逼近的边界之间，挖掉正对侧的元胞；
        但不越过边界线，也不把目标压在墙面上（H_f=1.33 低于 3m 墙，
        目标贴墙会被 EGO 规划撞墙）。"""
        x, y = point
        xmin, xmax, ymin, ymax = self.inner_region
        m = self.uav_margin
        if side == "UP":
            return (x, min(y + dist, ymax - m))
        if side == "DOWN":
            return (x, max(y - dist, ymin + m))
        if side == "LEFT":
            return (max(x - dist, xmin + m), y)
        if side == "RIGHT":
            return (min(x + dist, xmax - m), y)
        return (x, y)

    def _evasion_target(self):
        """iris_2 逃逸目标：五智能体 bounded Voronoi 元胞质心（第 7 节），限内区。"""
        agents = CARS + UAVS + (EVADER,)
        points = [self._position_xy(name) for name in agents]
        index = agents.index(EVADER)
        cell = bounded_voronoi_cell(points, index, self.inner_region)
        centroid = polygon_centroid(cell)
        if centroid is None:
            centroid = points[index]
        return clamp_point(centroid, self.inner_region, self.evasion_cell_margin)

    def _compute_pursuit_goals(self, tight):
        """第 9/10 节阵型点；tight=True 为 ENCIRCLED 收缩阵型逼近捕获条件。"""
        pE = self._position_xy(EVADER)
        ex, ey = self._escape_direction()
        nx, ny = -ey, ex
        evader_target = self._evasion_target()
        self.evasion_target = evader_target
        targets = {EVADER: evader_target}

        if tight:
            # H_f=1.33 低于 3m 虚拟墙，UAV 无法越墙，紧阵型目标必须 clamp 在内区
            # （原 H_f=4.0 可越墙封口的假设已失效）。
            uav_l = clamp_point(
                (pE[0] + 0.4 * ex + 0.6 * nx, pE[1] + 0.4 * ey + 0.6 * ny),
                self.inner_region,
                self.uav_margin,
            )
            uav_r = clamp_point(
                (pE[0] + 0.4 * ex - 0.6 * nx, pE[1] + 0.4 * ey - 0.6 * ny),
                self.inner_region,
                self.uav_margin,
            )
            # 用户要求（2026-08-26）：捕获时小车应直接收在入侵机正下方（地面投影附近）。
            # 两车对称贴在投影两侧 ±0.3m，既在机下又避免两车重叠；R_g=0.5 与之匹配。
            ugv_l = clamp_point(
                (pE[0] + 0.3 * nx, pE[1] + 0.3 * ny),
                self.inner_region,
                self.ugv_margin,
            )
            ugv_r = clamp_point(
                (pE[0] - 0.3 * nx, pE[1] - 0.3 * ny),
                self.inner_region,
                self.ugv_margin,
            )
        else:
            uav_l = clamp_point(
                (
                    pE[0] + self.uav_forward_offset * ex + self.uav_lateral_offset * nx,
                    pE[1] + self.uav_forward_offset * ey + self.uav_lateral_offset * ny,
                ),
                self.inner_region,
                self.uav_margin,
            )
            uav_r = clamp_point(
                (
                    pE[0] + self.uav_forward_offset * ex - self.uav_lateral_offset * nx,
                    pE[1] + self.uav_forward_offset * ey - self.uav_lateral_offset * ny,
                ),
                self.inner_region,
                self.uav_margin,
            )
            ugv_l = clamp_point(
                (
                    pE[0] - self.ugv_rear_offset * ex + self.ugv_lateral_offset * nx,
                    pE[1] - self.ugv_rear_offset * ey + self.ugv_lateral_offset * ny,
                ),
                self.inner_region,
                self.ugv_margin,
            )
            ugv_r = clamp_point(
                (
                    pE[0] - self.ugv_rear_offset * ex - self.ugv_lateral_offset * nx,
                    pE[1] - self.ugv_rear_offset * ey - self.ugv_lateral_offset * ny,
                ),
                self.inner_region,
                self.ugv_margin,
            )

        targets[self.uav_left] = uav_l
        targets[self.uav_right] = uav_r
        targets[self.ugv_left] = ugv_l
        targets[self.ugv_right] = ugv_r

        # ---- 边界封堵（仅 PURSUIT 宽松阵型）：iris_2 逼近边界且朝边界逃逸时，
        # 把一架防御 UAV 调到其与边界之间的"影子"位（pE 向边界推进 inset，
        # 不越过边界线），挖掉它正对侧 Voronoi 元胞，把逃逸质心拉回内区并放慢。
        # ENCIRCLED 紧阵型本身封口，此时封堵只会拆散包围圈，故 tight 时禁用。
        # 堵头选离影子点最近的一架 UAV（谁近谁最快到位，远者继续按阵型合围）；
        # 封堵侧带停留时间，避免 iris_2 沿墙摆动时堵头目标快速大跳变（曾把
        # EGO 重规划搞卡死，本会话 root cause）。 ----
        seal_side, seal_dist = self._nearest_boundary(pE)
        outward = {
            "UP": (0.0, 1.0),
            "DOWN": (0.0, -1.0),
            "LEFT": (-1.0, 0.0),
            "RIGHT": (1.0, 0.0),
        }[seal_side]
        heading = ex * outward[0] + ey * outward[1]
        seal_on = (
            self.gate_seal_enabled
            and not tight
            and seal_dist <= self.gate_seal_trigger
            and heading >= self.gate_seal_heading_cos
        )
        now = time.monotonic()
        if seal_on:
            if self.seal_committed_side is None:
                self.seal_committed_side = seal_side
                self.seal_commit_until = now + self.gate_seal_dwell
            elif seal_side == self.seal_committed_side:
                self.seal_commit_until = now + self.gate_seal_dwell
            elif now < self.seal_commit_until:
                seal_side = self.seal_committed_side  # 停留期内保持已承诺侧
            else:
                self.seal_committed_side = seal_side
                self.seal_commit_until = now + self.gate_seal_dwell
        else:
            self.seal_committed_side = None

        self.gate_seal_active = seal_on
        self.gate_seal_side = seal_side if seal_on else None
        if seal_on:
            seal_target = self._seal_point(pE, seal_side, self.gate_seal_inset)
            s0 = planar_distance(self._position_xy(UAVS[0]), seal_target)
            s1 = planar_distance(self._position_xy(UAVS[1]), seal_target)
            blocker = UAVS[0] if s0 <= s1 else UAVS[1]
            self.gate_seal_blocker = blocker
            targets[blocker] = seal_target
            if seal_side != self.last_seal_side or blocker != self.last_seal_blocker:
                rospy.logwarn(
                    "GATE SEAL: side=%s blocker=%s dist=%.2f goal=(%.2f, %.2f)",
                    seal_side,
                    blocker,
                    seal_dist,
                    seal_target[0],
                    seal_target[1],
                )
        elif self.last_seal_side is not None or self.last_seal_blocker is not None:
            rospy.logwarn("GATE SEAL off")
        self.last_seal_side = seal_side if seal_on else None
        self.last_seal_blocker = blocker if seal_on else None

        self.last_pursuit_targets = targets
        return targets

    def _encircled_check(self):
        """第 13 节：四追捕者凸包包含 iris_2 且面积不小于下限。
        iris_2 落在凸包边界附近（encircle_margin 内）也算包围，避免边界振荡抖动。"""
        hull = convex_hull([self._position_xy(name) for name in PURSUERS])
        if len(hull) < 3 or polygon_area(hull) < self.encircle_min_area:
            return False
        return point_polygon_distance(self._position_xy(EVADER), hull) <= self.encircle_margin

    def _begin_pursuit(self):
        self.pursuit_started_at = time.monotonic()
        self.capture_hold_started = None
        self.evader_vel_history = []
        self.evasion_target = None
        pE = self._position_xy(EVADER)
        self.evasion_target = self._evasion_target()
        ex, ey = self._escape_direction()

        def side_cross(name):
            px, py = self._position_xy(name)
            return ex * (py - pE[1]) - ey * (px - pE[0])

        # 第一次围捕按相对逃逸方向的左右分配 UAV 角色并锁定（第 9 节）
        if side_cross("iris_0") > side_cross("iris_1"):
            self.uav_left, self.uav_right = "iris_0", "iris_1"
        else:
            self.uav_left, self.uav_right = "iris_1", "iris_0"
        self.ugv_left, self.ugv_right = "car0", "car1"
        self.encircle_hull = []
        self._set_state(self.PURSUIT)
        rospy.logwarn(
            "PURSUIT: roles uav_left=%s uav_right=%s ugv_left=%s ugv_right=%s",
            self.uav_left,
            self.uav_right,
            self.ugv_left,
            self.ugv_right,
        )
        targets = self._compute_pursuit_goals(tight=False)
        for name, target in targets.items():
            self.published_targets.pop(name, None)
            self._send_goal(name, target)

    def _run_pursuit(self):
        self._update_evader_velocity()
        targets = self._compute_pursuit_goals(tight=False)
        for name, target in targets.items():
            self._send_goal(name, target)
        if not inside_inner(self._position_xy(EVADER), self.inner_region):
            self._finish_escaped()
            return
        if self._encircled_check():
            self._begin_encircled()

    def _begin_encircled(self):
        self.capture_hold_started = None
        self.encircle_lost_since = None
        self._set_state(self.ENCIRCLED)
        rospy.logwarn("ENCIRCLED: iris_2 inside pursuer convex hull")

    def _run_encircled(self):
        self._update_evader_velocity()
        targets = self._compute_pursuit_goals(tight=True)
        for name, target in targets.items():
            self._send_goal(name, target)
        pE = self._position_xy(EVADER)
        self.encircle_hull = convex_hull(
            [self._position_xy(name) for name in PURSUERS]
        )
        if not inside_inner(pE, self.inner_region):
            self._finish_escaped()
            return
        enc = self._encircled_check()
        if not enc:
            # 丢失包围持续超时则回退 PURSUIT：重新启用边界封堵，给 iris_2
            # 冲出包围圈逼近边界时第二次封堵机会（run#4 曾 ENCIRCLED 下 48s
            # enc 恒假、无封堵，最终 UP 门逃逸）。
            if self.encircle_lost_since is None:
                self.encircle_lost_since = time.monotonic()
            elif time.monotonic() - self.encircle_lost_since >= self.encircle_fallback_delay:
                rospy.logwarn(
                    "ENCIRCLED lost containment for %.1fs -> fallback PURSUIT",
                    self.encircle_fallback_delay,
                )
                self._begin_pursuit()
                return
        else:
            self.encircle_lost_since = None
        uav_min = min(planar_distance(self._position_xy(name), pE) for name in UAVS)
        uav_max = max(planar_distance(self._position_xy(name), pE) for name in UAVS)
        cars_max = max(
            planar_distance(self._position_xy(name), pE) for name in CARS
        )
        hold_elapsed = (
            0.0
            if self.capture_hold_started is None
            else time.monotonic() - self.capture_hold_started
        )
        pos = {n: self._position_xy(n) for n in ALL_AGENTS}
        rospy.loginfo_throttle(
            2.0,
            "E uav_max=%.2f cars_max=%.2f hold=%.1f seal=%s | e(%.2f,%.2f) "
            "i0(%.2f,%.2f) i1(%.2f,%.2f) c0(%.2f,%.2f) c1(%.2f,%.2f)",
            uav_max,
            cars_max,
            hold_elapsed,
            self.gate_seal_side or "-",
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
        if enc and uav_max <= self.air_capture_radius and cars_max <= self.ground_projection_radius:
            if self.capture_hold_started is None:
                self.capture_hold_started = time.monotonic()
                rospy.logwarn(
                    "capture conditions met: uav_max=%.2f cars_max=%.2f (R_c=%.2f R_g=%.2f)",
                    uav_max,
                    cars_max,
                    self.air_capture_radius,
                    self.ground_projection_radius,
                )
            elif time.monotonic() - self.capture_hold_started >= self.hold_time:
                self._finish_captured()
        else:
            self.capture_hold_started = None

    def _freeze_uav(self, name):
        """围捕结束后冻结 UAV：HOVER_LOCK（桥忽略后续 pose/vel，防止 EGO
        traj_server 残留逃逸轨迹经 cmd_pose_enu 覆盖）+ 以当前世界位姿（含当前
        z）为原地目标，保证飞行器完全静止。"""
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
            "pursuit %.1f s after intrusion" % (
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
        pE = self._position_xy(EVADER)
        message = "ESCAPED: iris_2 left protected region at (%.3f, %.3f) after %.1f s" % (
            pE[0],
            pE[1],
            time.monotonic() - self.pursuit_started_at,
        )
        self._set_state(self.ESCAPED)
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
            agent = self._marker(
                10 + index, "agents", Marker.SPHERE, colors[name]
            )
            agent.pose.position.x = point[0]
            agent.pose.position.y = point[1]
            if name in UAVS + (EVADER,):
                agent.pose.position.z = self._uav_altitude(name)
            else:
                agent.pose.position.z = 0.1
            agent.scale.x = agent.scale.y = agent.scale.z = 0.3
            markers.markers.append(agent)

        # iris_2 地面投影（第 21 节）
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

        # 围捕阵型目标点（第 21 节）
        if self.state in (self.PURSUIT, self.ENCIRCLED) and self.last_pursuit_targets:
            for index, (name, target) in enumerate(self.last_pursuit_targets.items()):
                goal = self._marker(
                    30 + index, "pursuit_goal", Marker.SPHERE, (1.0, 0.5, 1.0, 0.5)
                )
                goal.pose.position.x = target[0]
                goal.pose.position.y = target[1]
                goal.pose.position.z = 0.2 if name in CARS else 1.5
                goal.scale.x = goal.scale.y = goal.scale.z = 0.25
                markers.markers.append(goal)
            e0 = self.last_pursuit_targets[EVADER]
            line = self._marker(35, "evasion_vec", Marker.ARROW, (1.0, 0.3, 0.3, 0.7))
            line.scale.x = 0.03
            line.pose.position.x = e0[0]
            line.pose.position.y = e0[1]
            line.pose.position.z = 1.5
            ex, ey = self._escape_direction()
            line.points.append(Point(x=0, y=0, z=0))
            line.points.append(Point(x=1.5 * ex, y=1.5 * ey, z=0))
            markers.markers.append(line)

        # 四追捕者凸包（Encirclement 可视化）
        if self.state == self.ENCIRCLED and self.encircle_hull:
            hull = self._marker(40, "encircle_hull", Marker.LINE_STRIP, (1.0, 0.8, 0.1, 0.9))
            hull.scale.x = 0.05
            for px, py in self.encircle_hull:
                hull.points.append(Point(x=px, y=py, z=0.12))
            if len(self.encircle_hull) > 1:
                first = self.encircle_hull[0]
                hull.points.append(Point(x=first[0], y=first[1], z=0.12))
            markers.markers.append(hull)

        text = self._marker(60, "state", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = self.inner_region[1] + 0.2
        text.pose.position.y = self.inner_region[3] + 0.2
        text.pose.position.z = 0.8
        text.scale.z = 0.3
        if self.state == self.ENCIRCLED and self.capture_hold_started is not None:
            remain = self.hold_time - (time.monotonic() - self.capture_hold_started)
            detail = " | capture hold %.1fs" % max(0.0, remain)
        elif self.state in (self.PURSUIT, self.ENCIRCLED):
            pE = self._position_xy(EVADER)
            uav_min = min(planar_distance(self._position_xy(n), pE) for n in UAVS)
            cars_max = max(planar_distance(self._position_xy(n), pE) for n in CARS)
            detail = " | UAVmin=%.2f CARSmax=%.2f" % (uav_min, cars_max)
        else:
            detail = ""
        text.text = "Air intruder: %s | entry %s%s" % (
            self.state, self.entry_side, detail
        )
        markers.markers.append(text)
        self.marker_publisher.publish(markers)


if __name__ == "__main__":
    try:
        AirIntruderPursuit()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        rospy.logerr("air_intruder_pursuit stopped: %s", exc)
        sys.exit(1)
