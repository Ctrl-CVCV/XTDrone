#!/usr/bin/env python3
"""Bounded-Voronoi pursuit: car0/car1/iris_0/iris_1 pursue car2.

The pursuer law is ported from the supplied MATLAB program:
  * if a pursuer and the evader share a bounded Voronoi edge, move toward the
    midpoint of that edge;
  * otherwise move directly toward the evader;
  * capture occurs when any pursuer is within the planar capture distance.

The MATLAB evader-centroid law is intentionally replaced by a deterministic
door-to-door route for car2 (left-door entry, right-door escape).
"""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


PURSUERS = ("car0", "car1", "iris_0", "iris_1")
UAVS = ("iris_0", "iris_1")
EVADER = "car2"
ALL_AGENTS = PURSUERS + (EVADER,)


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


class VoronoiAirGroundPursuit:
    WAITING = "WAITING"
    APPROACH = "APPROACH"
    PURSUIT = "PURSUIT"
    CAPTURED = "CAPTURED"
    ESCAPED = "ESCAPED"

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
        self.goal_period = rospy.get_param("~goal_period", 1.0)
        self.goal_min_change = rospy.get_param("~goal_min_change", 0.15)
        self.route_tolerance = rospy.get_param("~route_tolerance", 0.35)
        self.auto_start = rospy.get_param("~auto_start", True)
        self.start_delay = rospy.get_param("~start_delay", 1.0)
        self.route = [
            tuple(float(value) for value in waypoint)
            for waypoint in rospy.get_param(
                "~intruder_route",
                [
                    [-4.066, 3.5],
                    [-4.066, -0.009],
                    [-2.166, -0.009],
                    [-0.116, -0.009],
                    [1.934, -0.009],
                    [3.534, -0.009],
                    [5.5, -0.009],
                ],
            )
        ]

        self.model_poses = {}
        self.uav_local_poses = {}
        self.uav_states = {}
        self.state = self.WAITING
        self.started = False
        self.entered = False
        self.route_index = 0
        self.route_goal_sent = False
        self.ready_since = None
        self.last_goal_time = 0.0
        self.last_targets = {}
        self.last_cells = {}
        self.published_targets = {}

        self.goal_publishers = {
            name: rospy.Publisher(
                "/%s/move_base_simple/goal" % name,
                PoseStamped,
                queue_size=1,
            )
            for name in ALL_AGENTS
        }
        self.uav_cmd_publishers = {
            name: rospy.Publisher(
                "/xtdrone/%s/cmd" % name, String, queue_size=1
            )
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
            "Voronoi pursuit ready: bounds x[%.3f, %.3f] y[%.3f, %.3f], "
            "capture=%.2fm, route=LEFT->RIGHT",
            self.bounds[0],
            self.bounds[1],
            self.bounds[2],
            self.bounds[3],
            self.capture_distance,
        )

    def _model_states_cb(self, msg):
        indices = {name: index for index, name in enumerate(msg.name)}
        for name in ALL_AGENTS:
            if name in indices:
                self.model_poses[name] = msg.pose[indices[name]]

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
                "not ready: need all models/Nav/EGO and both UAVs OFFBOARD+armed",
            )
        self._begin()
        return TriggerResponse(True, "Voronoi pursuit started")

    def _begin(self):
        self.started = True
        self.entered = False
        self.route_index = 0
        self.route_goal_sent = False
        self._set_state(self.APPROACH)
        self._publish_current_route_goal()
        rospy.loginfo("Intruder route started: approach LEFT door, escape RIGHT door")

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
            self.goal_publishers[name].publish(
                self._make_goal(*local_target)
            )
            return

        current = self._position_xy(name)
        yaw = math.atan2(target[1] - current[1], target[0] - current[0])
        self.goal_publishers[name].publish(
            self._make_goal(target[0], target[1], 0.0, yaw)
        )

    def _publish_current_route_goal(self):
        if self.route_index >= len(self.route):
            return
        waypoint = self.route[self.route_index]
        self._publish_world_goal(EVADER, waypoint)
        self.route_goal_sent = True
        rospy.loginfo(
            "car2 route waypoint %d/%d -> (%.2f, %.2f)",
            self.route_index + 1,
            len(self.route),
            waypoint[0],
            waypoint[1],
        )

    def _advance_intruder_route(self):
        if self.route_index >= len(self.route):
            return
        if not self.route_goal_sent:
            self._publish_current_route_goal()
            return
        if planar_distance(self._position_xy(EVADER), self.route[self.route_index]) <= self.route_tolerance:
            self.route_index += 1
            self.route_goal_sent = False
            if self.route_index < len(self.route):
                self._publish_current_route_goal()

    def _compute_targets(self):
        points = [self._position_xy(name) for name in ALL_AGENTS]
        evader_index = len(ALL_AGENTS) - 1
        cells = {
            name: bounded_voronoi_cell(points, index, self.bounds)
            for index, name in enumerate(ALL_AGENTS)
        }
        targets = {}
        for index, name in enumerate(PURSUERS):
            midpoint = shared_voronoi_edge_midpoint(
                cells[name], points[index], points[evader_index]
            )
            raw_target = midpoint if midpoint is not None else points[evader_index]
            targets[name] = clamp_point(raw_target, self.bounds, self.target_margin)
        self.last_cells = cells
        self.last_targets = targets
        return targets

    def _publish_pursuit_goals(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_goal_time < self.goal_period:
            return
        targets = self._compute_targets()
        for name, target in targets.items():
            previous = self.published_targets.get(name)
            if force or previous is None or planar_distance(previous, target) >= self.goal_min_change:
                self._publish_world_goal(name, target)
                self.published_targets[name] = target
        self.last_goal_time = now

    def _capture_check(self):
        evader = self._position_xy(EVADER)
        distances = {
            name: planar_distance(self._position_xy(name), evader)
            for name in PURSUERS
        }
        winner = min(distances, key=distances.get)
        return winner, distances[winner]

    def _finish(self, terminal_state, message):
        self._set_state(terminal_state)
        self.result_publisher.publish(String(message))

        # Replace all moving goals with current positions.  HOVER additionally
        # makes the XTDrone bridge hold both UAVs while EGO settles.
        for name in ("car0", "car1", EVADER):
            self._publish_world_goal(name, self._position_xy(name))
        for name in UAVS:
            self._publish_world_goal(name, self._position_xy(name))
            self.uav_cmd_publishers[name].publish(String(data="HOVER"))
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

        if self.state in (self.CAPTURED, self.ESCAPED):
            self._publish_markers()
            return

        self._advance_intruder_route()
        evader = self._position_xy(EVADER)

        if not self.entered and self._inside_boundary(evader):
            self.entered = True
            self._set_state(self.PURSUIT)
            self._publish_pursuit_goals(force=True)
            rospy.logwarn("car2 entered through LEFT door: Voronoi pursuit active")

        if self.entered:
            winner, distance = self._capture_check()
            if distance <= self.capture_distance:
                self._finish(
                    self.CAPTURED,
                    "CAPTURED by %s at planar distance %.3f m" % (winner, distance),
                )
                self._publish_markers()
                return
            if evader[0] > self.bounds[1]:
                self._finish(
                    self.ESCAPED,
                    "ESCAPED: car2 crossed the RIGHT-door boundary",
                )
                self._publish_markers()
                return
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
        boundary = self._marker(1, "boundary", Marker.LINE_STRIP, (1.0, 1.0, 1.0, 0.9))
        boundary.scale.x = 0.04
        for x, y in [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)]:
            boundary.points.append(Point(x=x, y=y, z=0.04))
        markers.markers.append(boundary)

        colors = {
            "car0": (0.1, 1.0, 0.1, 0.8),
            "car1": (0.1, 0.8, 1.0, 0.8),
            "iris_0": (1.0, 1.0, 0.0, 0.8),
            "iris_1": (1.0, 0.0, 1.0, 0.8),
            EVADER: (1.0, 0.1, 0.1, 0.8),
        }
        for index, name in enumerate(ALL_AGENTS):
            cell = self.last_cells.get(name)
            if cell:
                marker = self._marker(10 + index, "voronoi", Marker.LINE_STRIP, colors[name])
                marker.scale.x = 0.025
                for x, y in cell + [cell[0]]:
                    marker.points.append(Point(x=x, y=y, z=0.06))
                markers.markers.append(marker)

        for index, name in enumerate(PURSUERS):
            if name not in self.last_targets:
                continue
            target = self.last_targets[name]
            marker = self._marker(30 + index, "targets", Marker.SPHERE, colors[name])
            marker.pose.position.x = target[0]
            marker.pose.position.y = target[1]
            marker.pose.position.z = 0.12
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            markers.markers.append(marker)

        if EVADER in self.model_poses:
            evader = self._position_xy(EVADER)
            circle = self._marker(50, "capture", Marker.LINE_STRIP, (1.0, 0.2, 0.2, 1.0))
            circle.scale.x = 0.035
            for step in range(41):
                angle = 2.0 * math.pi * step / 40.0
                circle.points.append(
                    Point(
                        x=evader[0] + self.capture_distance * math.cos(angle),
                        y=evader[1] + self.capture_distance * math.sin(angle),
                        z=0.08,
                    )
                )
            markers.markers.append(circle)

        text = self._marker(60, "state", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = self.bounds[0] + 0.2
        text.pose.position.y = self.bounds[3] - 0.2
        text.pose.position.z = 0.8
        text.scale.z = 0.28
        text.text = "Voronoi pursuit: %s | capture %.2f m" % (
            self.state,
            self.capture_distance,
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

