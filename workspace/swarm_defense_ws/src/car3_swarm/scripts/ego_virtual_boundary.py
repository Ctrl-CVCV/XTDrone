#!/usr/bin/env python3
"""Publish the nesting-room boundary as static obstacle clouds for EGO-Swarm."""

import math
import threading

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class EgoVirtualBoundary:
    def __init__(self):
        rospy.init_node("ego_virtual_boundary")

        self.uavs = tuple(rospy.get_param("~uavs", ["iris_0", "iris_1"]))
        center_x = float(rospy.get_param("~center_x", -0.11576))
        center_y = float(rospy.get_param("~center_y", -0.00882))
        side = float(rospy.get_param("~side", 6.0))
        self.z_min = float(rospy.get_param("~z_min", 0.0))
        self.z_max = float(rospy.get_param("~z_max", 3.0))
        self.spacing = float(rospy.get_param("~spacing", 0.2))
        monitor_rate = float(rospy.get_param("~monitor_rate", 2.0))
        self.stable_samples = int(rospy.get_param("~stable_samples", 3))
        self.stable_tolerance = float(rospy.get_param("~stable_tolerance", 0.05))
        self.frame_id = rospy.get_param("~frame_id", "world")

        if side <= 0.0 or self.spacing <= 0.0 or self.z_max <= self.z_min:
            raise RuntimeError("invalid virtual boundary geometry parameters")
        if monitor_rate <= 0.0 or self.stable_samples < 1:
            raise RuntimeError("monitor rate and stable sample count must be positive")
        if self.stable_tolerance < 0.0:
            raise RuntimeError("invalid virtual boundary stability threshold")

        half = side * 0.5
        self.bounds = (
            center_x - half,
            center_x + half,
            center_y - half,
            center_y + half,
        )
        self.world_points = self._make_wall_points()
        self.world_poses = {}
        self.local_poses = {}
        self.last_offset_candidates = {}
        self.offset_stable_counts = {}
        self.published_offsets = {}
        self.shutdown_scheduled = False

        self.publishers = {
            name: rospy.Publisher(
                "/%s/pcl_render_node/points" % name,
                PointCloud2,
                queue_size=1,
                latch=True,
            )
            for name in self.uavs
        }

        self.model_states_subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1
        )
        self.local_pose_subscribers = []
        for name in self.uavs:
            self.local_pose_subscribers.append(
                rospy.Subscriber(
                    "/%s/mavros/local_position/pose" % name,
                    PoseStamped,
                    self._local_pose_cb,
                    callback_args=name,
                    queue_size=1,
                )
            )

        # The timer waits only until both local-origin transforms are stable.
        # It is shut down permanently after the two latched clouds are sent.
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / monitor_rate), self._monitor_origins
        )
        rospy.loginfo(
            "EGO virtual boundary ready: x[%.3f, %.3f], y[%.3f, %.3f], "
            "z[%.2f, %.2f], spacing=%.2f, points=%d",
            self.bounds[0],
            self.bounds[1],
            self.bounds[2],
            self.bounds[3],
            self.z_min,
            self.z_max,
            self.spacing,
            len(self.world_points),
        )

    @staticmethod
    def _samples(start, stop, spacing):
        count = int(math.ceil((stop - start) / spacing))
        return [start + (stop - start) * index / count for index in range(count + 1)]

    def _make_wall_points(self):
        xmin, xmax, ymin, ymax = self.bounds
        xs = self._samples(xmin, xmax, self.spacing)
        ys = self._samples(ymin, ymax, self.spacing)
        zs = self._samples(self.z_min, self.z_max, self.spacing)
        points = []
        for z in zs:
            points.extend((x, ymin, z) for x in xs)
            points.extend((x, ymax, z) for x in xs)
            points.extend((xmin, y, z) for y in ys[1:-1])
            points.extend((xmax, y, z) for y in ys[1:-1])
        return points

    def _model_states_cb(self, msg):
        indices = {name: index for index, name in enumerate(msg.name)}
        for name in self.uavs:
            if name in indices:
                self.world_poses[name] = msg.pose[indices[name]].position

    def _local_pose_cb(self, msg, name):
        self.local_poses[name] = msg.pose.position

    @staticmethod
    def _distance(a, b):
        return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))

    def _candidate_offset(self, name):
        world = self.world_poses[name]
        local = self.local_poses[name]
        return (world.x - local.x, world.y - local.y, world.z - local.z)

    def _publish_cloud(self, name, offset, reason):
        local_points = [
            (x - offset[0], y - offset[1], z - offset[2])
            for x, y, z in self.world_points
        ]
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.publishers[name].publish(
            point_cloud2.create_cloud_xyz32(header, local_points)
        )
        self.published_offsets[name] = offset
        rospy.logwarn(
            "%s virtual boundary published once (%s): offset=(%.3f, %.3f, %.3f), "
            "points=%d",
            name,
            reason,
            offset[0],
            offset[1],
            offset[2],
            len(local_points),
        )

    def _monitor_origins(self, _event):
        for name in self.uavs:
            if name not in self.world_poses or name not in self.local_poses:
                continue
            if self.publishers[name].get_num_connections() == 0:
                continue
            candidate = self._candidate_offset(name)
            previous_candidate = self.last_offset_candidates.get(name)
            if (
                previous_candidate is not None
                and self._distance(candidate, previous_candidate)
                <= self.stable_tolerance
            ):
                self.offset_stable_counts[name] = (
                    self.offset_stable_counts.get(name, 1) + 1
                )
            else:
                self.offset_stable_counts[name] = 1
            self.last_offset_candidates[name] = candidate

            if self.offset_stable_counts[name] < self.stable_samples:
                continue
            published = self.published_offsets.get(name)
            if published is None:
                self._publish_cloud(name, candidate, "initial stable origin")

        if (
            len(self.published_offsets) == len(self.uavs)
            and not self.shutdown_scheduled
        ):
            self.shutdown_scheduled = True
            self.timer.shutdown()
            self.model_states_subscriber.unregister()
            for subscriber in self.local_pose_subscribers:
                subscriber.unregister()
            self.world_poses.clear()
            self.local_poses.clear()
            self.last_offset_candidates.clear()
            rospy.logwarn(
                "All virtual boundaries delivered; node exits after a 1 s latch grace period"
            )
            shutdown_timer = threading.Timer(
                1.0,
                lambda: rospy.signal_shutdown("one-shot virtual boundaries delivered"),
            )
            shutdown_timer.daemon = True
            shutdown_timer.start()


if __name__ == "__main__":
    try:
        EgoVirtualBoundary()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        rospy.logerr("EGO virtual boundary stopped: %s", exc)
