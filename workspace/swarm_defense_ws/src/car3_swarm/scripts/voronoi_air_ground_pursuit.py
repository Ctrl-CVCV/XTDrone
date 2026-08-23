#!/usr/bin/env python3
"""Bounded-Voronoi pursuit for two ground intruders.

car0, car1, iris_0 and iris_1 pursue car2 and car3. Each intruder enters by a
separate door, moves toward the centroid of its active bounded Voronoi cell,
and is independently captured and removed. Pursuers return home only after
both intruders have been captured.
"""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


PURSUERS = ("car0", "car1", "iris_0", "iris_1")
UAVS = ("iris_0", "iris_1")
EVADERS = ("car2", "car3")
ALL_AGENTS = PURSUERS + EVADERS


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
        # |x-p|^2 <= |x-q|^2
        a = 2.0 * (qx - px)
        b = 2.0 * (qy - py)
        c = qx * qx + qy * qy - px * px - py * py
        if abs(a) + abs(b) < 1e-12:
            continue
        polygon = clip_polygon_halfplane(polygon, a, b, c)
        if not polygon:
            break
    return polygon


def shared_voronoi_edge_midpoint(cell, point_a, point_b, tolerance=1e-5):
    """Return midpoint of the cell edge lying on the A/B bisector, or None."""
    if len(cell) < 2:
        return None
    ax = 2.0 * (point_b[0] - point_a[0])
    ay = 2.0 * (point_b[1] - point_a[1])
    c = (
        point_b[0] * point_b[0]
        + point_b[1] * point_b[1]
        - point_a[0] * point_a[0]
        - point_a[1] * point_a[1]
    )
    scale = max(1.0, abs(ax), abs(ay), abs(c))
    best = None
    best_length = 0.0
    for index, start in enumerate(cell):
        end = cell[(index + 1) % len(cell)]
        start_error = abs(ax * start[0] + ay * start[1] - c) / scale
        end_error = abs(ax * end[0] + ay * end[1] - c) / scale
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if start_error <= tolerance and end_error <= tolerance and length > best_length:
            best = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5)
            best_length = length
    return best if best_length > 1e-4 else None


def clamp_point(point, bounds, margin):
    xmin, xmax, ymin, ymax = bounds
    return (
        max(xmin + margin, min(xmax - margin, point[0])),
        max(ymin + margin, min(ymax - margin, point[1])),
    )


def planar_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def directional_lookahead(origin, direction_target, bounds, margin, distance):
    """Project a distant Nav goal along the MATLAB unit-control direction."""
    dx = direction_target[0] - origin[0]
    dy = direction_target[1] - origin[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-8:
        return clamp_point(direction_target, bounds, margin)
    projected = (
        origin[0] + distance * dx / norm,
        origin[1] + distance * dy / norm,
    )
    return clamp_point(projected, bounds, margin)


def polygon_centroid(polygon):
    """Return the area centroid of a non-self-intersecting polygon."""
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


class VoronoiAirGroundPursuit:
    WAITING = "WAITING"
    APPROACH = "APPROACH"
    PURSUIT = "PURSUIT"
    CAPTURED = "CAPTURED"
    RETURNING = "RETURNING"
    RETURNED = "RETURNED"

    def __init__(self):
        rospy.init_node("voronoi_air_ground_pursuit")

        center_x = rospy.get_param("~boundary_center_x", -0.11576)
        center_y = rospy.get_param("~boundary_center_y", -0.00882)
        side = rospy.get_param("~boundary_side", 6.0)
        half = side * 0.5
        self.bounds = (
            center_x - half,
            center_x + half,
            center_y - half,
            center_y + half,
        )
        self.capture_distance = rospy.get_param("~capture_distance", 0.3)
        self.uav_altitude = rospy.get_param("~uav_altitude", 1.5)
        self.target_margin = rospy.get_param("~target_margin", 0.20)
        self.goal_period = rospy.get_param("~goal_period", 0.25)
        self.goal_min_change = rospy.get_param("~goal_min_change", 0.05)
        self.uav_goal_period = rospy.get_param("~uav_goal_period", 1.5)
        self.uav_goal_min_change = rospy.get_param("~uav_goal_min_change", 0.6)
        self.evader_goal_min_change = rospy.get_param("~evader_goal_min_change", 0.08)
        self.evader_lookahead = rospy.get_param("~evader_lookahead", 1.2)
        self.route_tolerance = rospy.get_param("~route_tolerance", 0.35)
        self.auto_start = rospy.get_param("~auto_start", True)
        self.start_delay = rospy.get_param("~start_delay", 1.0)
        self.return_delay = rospy.get_param("~return_delay", 0.0)
        self.return_tolerance = rospy.get_param("~return_tolerance", 0.25)
        self.uav_return_tolerance_margin = rospy.get_param(
            "~uav_return_tolerance_margin", 0.15
        )
        self.return_goal_period = rospy.get_param("~return_goal_period", 1.0)
        self.uav_return_retry = rospy.get_param("~uav_return_retry", 10.0)
        self.uav_home_clearance = rospy.get_param("~uav_home_clearance", 0.8)

        route_defaults = {
            "car2": [[-4.066, 3.5], [-4.066, -0.009], [-2.166, -0.009]],
            "car3": [[4.0, -3.5], [4.0, -0.009], [1.85, -0.009]],
        }
        route_config = rospy.get_param("~intruder_routes", route_defaults)
        self.routes = {
            name: [tuple(float(value) for value in waypoint) for waypoint in route_config[name]]
            for name in EVADERS
        }

        self.model_poses = {}
        self.uav_local_poses = {}
        self.uav_states = {}
        self.state = self.WAITING
        self.started = False
        self.ready_since = None
        self.home_positions = {}
        self.active_evaders = set()
        self.entered = {}
        self.route_indices = {}
        self.route_goal_sent = {}
        self.pursuit_started_at = {}
        self.capture_elapsed = {}
        self.captured_results = {}
        self.uav_capture_priority = []
        self.deleted_evaders = set()
        self.mission_started_at = None
        self.all_capture_message = None
        self.capture_time = None
        self.last_goal_time = {name: 0.0 for name in ALL_AGENTS}
        self.last_return_goal_time = 0.0
        self.uav_return_sent = set()
        self.uav_return_best_distance = {}
        self.uav_return_progress_time = {}
        self.uav_return_queue = []
        self.uav_return_active = None
        self.last_targets = {}
        self.last_cells = {}
        self.published_targets = {}
        self.goal_connection_counts = {name: 0 for name in ALL_AGENTS}

        self.goal_publishers = {
            name: rospy.Publisher(
                "/%s/move_base_simple/goal" % name,
                PoseStamped,
                queue_size=1,
            )
            for name in ALL_AGENTS
        }
        self.uav_cmd_publishers = {
            name: rospy.Publisher("/xtdrone/%s/cmd" % name, String, queue_size=1)
            for name in UAVS
        }
        self.state_publisher = rospy.Publisher(
            "/air_ground/pursuit/state", String, queue_size=1, latch=True
        )
        self.result_publisher = rospy.Publisher(
            "/air_ground/pursuit/result", String, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(
            "/air_ground/pursuit/markers", MarkerArray, queue_size=1
        )
        # Deleting a live ros_control vehicle can deadlock gzserver while its
        # plugins are being destroyed. Hide captured intruders asynchronously.
        self.model_state_publisher = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=2
        )

        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1
        )
        for name in UAVS:
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

        rospy.Service("/air_ground/pursuit/start", Trigger, self._start_service)
        self.state_publisher.publish(String(self.state))
        self.timer = rospy.Timer(rospy.Duration(0.2), self._tick)

        rospy.loginfo(
            "Dual-intruder Voronoi pursuit ready: bounds x[%.3f, %.3f] "
            "y[%.3f, %.3f], capture=%.2fm, evaders=%s",
            self.bounds[0],
            self.bounds[1],
            self.bounds[2],
            self.bounds[3],
            self.capture_distance,
            ",".join(EVADERS),
        )

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

    def _ready(self):
        models_ready = all(name in self.model_poses for name in ALL_AGENTS)
        local_ready = all(name in self.uav_local_poses for name in UAVS)
        flight_ready = all(
            name in self.uav_states
            and self.uav_states[name].connected
            and self.uav_states[name].armed
            and self.uav_states[name].mode == "OFFBOARD"
            for name in UAVS
        )
        subscribers_ready = all(
            self.goal_publishers[name].get_num_connections() > 0
            for name in ALL_AGENTS
        )
        return models_ready and local_ready and flight_ready and subscribers_ready

    def _start_service(self, _request):
        if self.started:
            return TriggerResponse(False, "pursuit already started")
        if not self._ready():
            return TriggerResponse(
                False,
                "not ready: need six models, six Nav/EGO goal subscribers, "
                "and both UAVs OFFBOARD+armed",
            )
        self._begin()
        return TriggerResponse(True, "dual-intruder Voronoi pursuit started")

    def _begin(self):
        self.started = True
        self.active_evaders = set(EVADERS)
        self.entered = {name: False for name in EVADERS}
        self.route_indices = {name: 0 for name in EVADERS}
        self.route_goal_sent = {name: False for name in EVADERS}
        self.pursuit_started_at = {name: None for name in EVADERS}
        self.capture_elapsed = {}
        self.captured_results = {}
        self.uav_capture_priority = []
        self.deleted_evaders = set()
        self.mission_started_at = None
        self.all_capture_message = None
        self.capture_time = None
        self.home_positions = {name: self._position_xy(name) for name in PURSUERS}
        self.last_targets.clear()
        self.last_cells.clear()
        self.uav_return_sent.clear()
        self.uav_return_best_distance.clear()
        self.uav_return_progress_time.clear()
        self.uav_return_queue = []
        self.uav_return_active = None
        self.published_targets.clear()
        self.last_goal_time = {name: 0.0 for name in ALL_AGENTS}
        self.goal_connection_counts = {
            name: self.goal_publishers[name].get_num_connections()
            for name in ALL_AGENTS
        }
        self._set_state(self.APPROACH)
        for name in UAVS:
            self.uav_cmd_publishers[name].publish(String(data="OFFBOARD"))
        for name in EVADERS:
            self._publish_current_route_goal(name)
        rospy.loginfo("Two intruders started: car2 uses LEFT door, car3 uses RIGHT door")

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self.state_publisher.publish(String(state))
            rospy.loginfo("Pursuit state -> %s", state)

    def _position_xy(self, name):
        pose = self.model_poses[name].position
        return (pose.x, pose.y)

    def _inside_boundary(self, point):
        xmin, xmax, ymin, ymax = self.bounds
        return xmin <= point[0] <= xmax and ymin <= point[1] <= ymax

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

    def _publish_world_goal(self, name, target):
        if name in UAVS:
            world_pose = self.model_poses[name].position
            local_pose = self.uav_local_poses[name].position
            local_target = (
                target[0] - (world_pose.x - local_pose.x),
                target[1] - (world_pose.y - local_pose.y),
                self.uav_altitude - (world_pose.z - local_pose.z),
            )
            self.goal_publishers[name].publish(self._make_goal(*local_target))
            return

        current = self._position_xy(name)
        yaw = math.atan2(target[1] - current[1], target[0] - current[0])
        self.goal_publishers[name].publish(
            self._make_goal(target[0], target[1], 0.0, yaw)
        )

    def _publish_current_route_goal(self, name):
        index = self.route_indices[name]
        route = self.routes[name]
        if index >= len(route):
            return
        waypoint = route[index]
        self._publish_world_goal(name, waypoint)
        self.route_goal_sent[name] = True
        self.last_targets[name] = waypoint
        rospy.loginfo(
            "%s route waypoint %d/%d -> (%.2f, %.2f)",
            name,
            index + 1,
            len(route),
            waypoint[0],
            waypoint[1],
        )

    def _advance_intruder_route(self, name):
        index = self.route_indices[name]
        route = self.routes[name]
        if index >= len(route):
            return
        if not self.route_goal_sent[name]:
            self._publish_current_route_goal(name)
            return
        if planar_distance(self._position_xy(name), route[index]) <= self.route_tolerance:
            self.route_indices[name] += 1
            self.route_goal_sent[name] = False
            if self.route_indices[name] < len(route):
                self._publish_current_route_goal(name)

    def _active_inside_evaders(self):
        return tuple(
            name for name in EVADERS
            if name in self.active_evaders and self.entered.get(name, False)
        )

    def _compute_targets(self):
        active_inside = self._active_inside_evaders()
        if not active_inside:
            self.last_cells = {}
            return {}

        agents = PURSUERS + active_inside
        points = [self._position_xy(name) for name in agents]
        indices = {name: index for index, name in enumerate(agents)}
        cells = {
            name: bounded_voronoi_cell(points, index, self.bounds)
            for index, name in enumerate(agents)
        }
        targets = {}
        for name in PURSUERS:
            pursuer_point = points[indices[name]]
            shared_candidates = []
            for evader in active_inside:
                midpoint = shared_voronoi_edge_midpoint(
                    cells[name], pursuer_point, points[indices[evader]]
                )
                if midpoint is not None:
                    shared_candidates.append(
                        (planar_distance(pursuer_point, midpoint), midpoint)
                    )
            if shared_candidates:
                raw_target = min(shared_candidates, key=lambda item: item[0])[1]
            else:
                nearest = min(
                    active_inside,
                    key=lambda evader: planar_distance(
                        pursuer_point, points[indices[evader]]
                    ),
                )
                raw_target = points[indices[nearest]]
            targets[name] = clamp_point(raw_target, self.bounds, self.target_margin)

        for name in active_inside:
            evader_point = points[indices[name]]
            centroid = polygon_centroid(cells[name])
            if centroid is None:
                centroid = evader_point
            centroid = clamp_point(centroid, self.bounds, self.target_margin)
            targets[name] = directional_lookahead(
                evader_point,
                centroid,
                self.bounds,
                self.target_margin,
                self.evader_lookahead,
            )

        self.last_cells = cells
        self.last_targets = targets
        return targets

    def _publish_pursuit_goals(self, force=False):
        now = time.monotonic()
        targets = self._compute_targets()
        for name, target in targets.items():
            connections = self.goal_publishers[name].get_num_connections()
            reconnected = self.goal_connection_counts.get(name, 0) == 0 and connections > 0
            self.goal_connection_counts[name] = connections
            if reconnected and name in UAVS:
                # A respawned EGO planner has no previous target state.
                self.published_targets.pop(name, None)

            period = self.uav_goal_period if name in UAVS else self.goal_period
            if not force and now - self.last_goal_time[name] < period:
                continue

            previous = self.published_targets.get(name)
            if name in UAVS:
                min_change = self.uav_goal_min_change
            elif name in EVADERS:
                min_change = self.evader_goal_min_change
            else:
                min_change = self.goal_min_change

            if force or previous is None or planar_distance(previous, target) >= min_change:
                self._publish_world_goal(name, target)
                self.published_targets[name] = target
                self.last_goal_time[name] = now

    def _capture_check(self, evader):
        point = self._position_xy(evader)
        distances = {
            name: planar_distance(self._position_xy(name), point)
            for name in PURSUERS
        }
        winner = min(distances, key=distances.get)
        return winner, distances[winner]

    def _capture_evader(self, evader, winner, distance):
        now_ros = rospy.Time.now()
        started_at = self.pursuit_started_at[evader]
        elapsed = max(0.0, (now_ros - started_at).to_sec())
        self.capture_elapsed[evader] = elapsed
        self.active_evaders.remove(evader)
        message = (
            "CAPTURED %s by %s at planar distance %.3f m; capture time %.3f s"
            % (evader, winner, distance, elapsed)
        )
        self.captured_results[evader] = message
        if winner in UAVS:
            if winner in self.uav_capture_priority:
                self.uav_capture_priority.remove(winner)
            self.uav_capture_priority.insert(0, winner)
        self.result_publisher.publish(String(message))
        rospy.logwarn("%s; remaining intruders=%d", message, len(self.active_evaders))
        self._delete_evader(evader)

        if not self.active_evaders:
            total = max(0.0, (now_ros - self.mission_started_at).to_sec())
            self.all_capture_message = "ALL CAPTURED in %.3f s | %s" % (
                total,
                " | ".join(self.captured_results[name] for name in EVADERS),
            )
            self.capture_time = time.monotonic()
            self._set_state(self.CAPTURED)
            self.result_publisher.publish(String(self.all_capture_message))
            for name in ("car0", "car1"):
                self._publish_world_goal(name, self._position_xy(name))
            rospy.logwarn(self.all_capture_message)
        else:
            self._publish_pursuit_goals(force=True)

    def _delete_evader(self, name):
        if name in self.deleted_evaders:
            return True

        hidden = ModelState()
        hidden.model_name = name
        hidden.reference_frame = "world"
        hidden.pose.position.x = 100.0 + EVADERS.index(name) * 5.0
        hidden.pose.position.y = 100.0
        hidden.pose.position.z = -10.0
        hidden.pose.orientation.w = 1.0
        self.model_state_publisher.publish(hidden)

        self.deleted_evaders.add(name)
        self.last_cells.pop(name, None)
        self.last_targets.pop(name, None)
        self.published_targets.pop(name, None)
        rospy.logwarn(
            "Captured target %s hidden outside the world (non-blocking)", name
        )
        return True

    def _retry_pending_deletions(self):
        for name in self.captured_results:
            if name not in self.deleted_evaders:
                self._delete_evader(name)

    def _begin_return(self):
        self._set_state(self.RETURNING)
        self.last_cells.clear()
        self.last_targets = dict(self.home_positions)
        self.last_return_goal_time = 0.0
        now = time.monotonic()
        self.uav_return_sent.clear()
        self.uav_return_best_distance = {
            name: planar_distance(self._position_xy(name), self.home_positions[name])
            for name in UAVS
        }
        self.uav_return_progress_time = {name: now for name in UAVS}
        capture_rank = {
            name: index for index, name in enumerate(self.uav_capture_priority)
        }

        def return_priority(name):
            blocks_other_home = any(
                other != name
                and planar_distance(
                    self._position_xy(name), self.home_positions[other]
                ) < self.uav_home_clearance
                for other in UAVS
            )
            return (
                0 if blocks_other_home else 1,
                capture_rank.get(name, len(UAVS)),
                planar_distance(self._position_xy(name), self.home_positions[name]),
            )

        self.uav_return_queue = sorted(UAVS, key=return_priority)
        self.uav_return_active = None
        # Cancel stale pursuit trajectories before sequential return. The queued
        # UAV must hold position instead of continuing its last pursuit path.
        for name in UAVS:
            self.uav_cmd_publishers[name].publish(String(data="HOVER"))
        return_order = " -> ".join(self.uav_return_queue)
        rospy.logwarn(
            "Both intruders removed; ground vehicles return in parallel, "
            "UAV return order=%s",
            return_order,
        )
        self._publish_return_goals(force=True)

    def _activate_next_uav_return(self, now):
        if self.uav_return_active is not None:
            name = self.uav_return_active
            distance = planar_distance(
                self._position_xy(name), self.home_positions[name]
            )
            if self._at_home(name):
                rospy.logwarn("Sequential UAV return completed: %s", name)
                self.uav_cmd_publishers[name].publish(String(data="HOVER"))
                self.uav_return_active = None

        while self.uav_return_active is None and self.uav_return_queue:
            name = self.uav_return_queue.pop(0)
            distance = planar_distance(
                self._position_xy(name), self.home_positions[name]
            )
            if self._at_home(name):
                rospy.logwarn("Sequential UAV return skipped (already home): %s", name)
                continue
            self.uav_return_active = name
            # Release this UAV from the waiting HOVER latch so EGO can own it.
            self.uav_cmd_publishers[name].publish(String(data="OFFBOARD"))
            self.uav_return_sent.discard(name)
            self.uav_return_best_distance[name] = distance
            self.uav_return_progress_time[name] = now
            rospy.logwarn(
                "Sequential UAV return active: %s (distance %.2f m)",
                name,
                distance,
            )
            break

    def _publish_return_goals(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_return_goal_time < self.return_goal_period:
            return
        # Ground Nav safely accepts periodic goal refreshes.
        for name in ("car0", "car1"):
            if (
                planar_distance(self._position_xy(name), self.home_positions[name])
                > self.return_tolerance
            ):
                self._publish_world_goal(name, self.home_positions[name])

        # Return UAVs one at a time. Their diagonal home paths can otherwise
        # conflict and EGO-Swarm may keep both vehicles in an avoidance deadlock.
        self._activate_next_uav_return(now)
        name = self.uav_return_active
        if name is not None:
            connections = self.goal_publishers[name].get_num_connections()
            reconnected = (
                self.goal_connection_counts.get(name, 0) == 0 and connections > 0
            )
            self.goal_connection_counts[name] = connections
            if reconnected:
                self.uav_return_sent.discard(name)
            distance = planar_distance(
                self._position_xy(name), self.home_positions[name]
            )
            best = self.uav_return_best_distance[name]
            if distance < best - 0.10:
                self.uav_return_best_distance[name] = distance
                self.uav_return_progress_time[name] = now
            stalled = now - self.uav_return_progress_time[name] >= self.uav_return_retry
            if not self._at_home(name) and (
                name not in self.uav_return_sent or stalled
            ):
                if stalled:
                    rospy.logwarn(
                        "Retrying stalled UAV return: %s (distance %.2f m)",
                        name,
                        distance,
                    )
                self._publish_world_goal(name, self.home_positions[name])
                self.uav_return_sent.add(name)
                self.uav_return_best_distance[name] = distance
                self.uav_return_progress_time[name] = now
        self.last_return_goal_time = now

    def _at_home(self, name):
        tolerance = self.return_tolerance
        if name in UAVS:
            tolerance += self.uav_return_tolerance_margin
        return (
            planar_distance(self._position_xy(name), self.home_positions[name])
            <= tolerance
        )

    def _return_complete(self):
        return all(self._at_home(name) for name in PURSUERS)

    def _finish_return(self):
        self._set_state(self.RETURNED)
        for name in ("car0", "car1"):
            self._publish_world_goal(name, self._position_xy(name))
        for name in UAVS:
            self.uav_cmd_publishers[name].publish(String(data="HOVER"))
        message = "%s; all pursuers returned home" % self.all_capture_message
        self.result_publisher.publish(String(message))
        rospy.logwarn(message)

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

        self._retry_pending_deletions()

        if self.state == self.CAPTURED:
            if (
                all(name in self.deleted_evaders for name in EVADERS)
                and time.monotonic() - self.capture_time >= self.return_delay
            ):
                self._begin_return()
            self._publish_markers()
            return

        if self.state == self.RETURNING:
            self._publish_return_goals()
            if self._return_complete():
                self._finish_return()
            self._publish_markers()
            return

        if self.state == self.RETURNED:
            self._publish_markers()
            return

        for name in EVADERS:
            if name in self.active_evaders and not self.entered[name]:
                self._advance_intruder_route(name)

        newly_entered = []
        for name in EVADERS:
            if (
                name in self.active_evaders
                and not self.entered[name]
                and name in self.model_poses
                and self._inside_boundary(self._position_xy(name))
            ):
                self.entered[name] = True
                self.pursuit_started_at[name] = rospy.Time.now()
                if self.mission_started_at is None:
                    self.mission_started_at = self.pursuit_started_at[name]
                newly_entered.append(name)
                rospy.logwarn(
                    "%s entered the room: independent capture timer started", name
                )

        if newly_entered:
            self._set_state(self.PURSUIT)
            self._publish_pursuit_goals(force=True)

        for name in list(self._active_inside_evaders()):
            winner, distance = self._capture_check(name)
            if distance <= self.capture_distance:
                self._capture_evader(name, winner, distance)

        if self.state == self.CAPTURED:
            self._publish_markers()
            return

        if self._active_inside_evaders():
            self._publish_pursuit_goals()

        self._publish_markers()

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

        xmin, xmax, ymin, ymax = self.bounds
        boundary = self._marker(
            1, "boundary", Marker.LINE_STRIP, (1.0, 1.0, 1.0, 0.9)
        )
        boundary.scale.x = 0.04
        for x, y in [
            (xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)
        ]:
            boundary.points.append(Point(x=x, y=y, z=0.04))
        markers.markers.append(boundary)

        colors = {
            "car0": (0.1, 1.0, 0.1, 0.8),
            "car1": (0.1, 0.8, 1.0, 0.8),
            "iris_0": (1.0, 1.0, 0.0, 0.8),
            "iris_1": (1.0, 0.0, 1.0, 0.8),
            "car2": (1.0, 0.1, 0.1, 0.8),
            "car3": (1.0, 0.5, 0.0, 0.8),
        }
        for index, name in enumerate(ALL_AGENTS):
            cell = self.last_cells.get(name)
            if cell:
                marker = self._marker(
                    10 + index, "voronoi", Marker.LINE_STRIP, colors[name]
                )
                marker.scale.x = 0.025
                for x, y in cell + [cell[0]]:
                    marker.points.append(Point(x=x, y=y, z=0.06))
                markers.markers.append(marker)

        for index, name in enumerate(ALL_AGENTS):
            if name not in self.last_targets:
                continue
            target = self.last_targets[name]
            marker = self._marker(
                30 + index, "targets", Marker.SPHERE, colors[name]
            )
            marker.pose.position.x = target[0]
            marker.pose.position.y = target[1]
            marker.pose.position.z = 0.12
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            markers.markers.append(marker)

        for index, name in enumerate(EVADERS):
            if name not in self.active_evaders or name not in self.model_poses:
                continue
            point = self._position_xy(name)
            circle = self._marker(
                50 + index, "capture", Marker.LINE_STRIP, colors[name]
            )
            circle.scale.x = 0.035
            for step in range(41):
                angle = 2.0 * math.pi * step / 40.0
                circle.points.append(
                    Point(
                        x=point[0] + self.capture_distance * math.cos(angle),
                        y=point[1] + self.capture_distance * math.sin(angle),
                        z=0.08,
                    )
                )
            markers.markers.append(circle)

        text = self._marker(
            60, "state", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0)
        )
        text.pose.position.x = self.bounds[0] + 0.2
        text.pose.position.y = self.bounds[3] - 0.2
        text.pose.position.z = 0.8
        text.scale.z = 0.28
        active_text = ",".join(
            name for name in EVADERS if name in self.active_evaders
        ) or "none"
        text.text = (
            "Dual Voronoi: %s | captured %d/2 | active %s | capture %.2f m"
            % (
                self.state,
                len(self.captured_results),
                active_text,
                self.capture_distance,
            )
        )
        markers.markers.append(text)
        self.marker_publisher.publish(markers)


if __name__ == "__main__":
    try:
        VoronoiAirGroundPursuit()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        rospy.logerr("Voronoi pursuit stopped: %s", exc)
        sys.exit(1)

