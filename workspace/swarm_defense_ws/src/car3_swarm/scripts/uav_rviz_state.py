#!/usr/bin/env python3
"""Publish UAV Gazebo world poses and bounded trails for RViz."""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class UavRvizState:
    def __init__(self):
        self.names = ("iris_0", "iris_1", "iris_2")
        self.pose_pubs = {
            name: rospy.Publisher(
                "/air_ground/%s/world_pose" % name,
                PoseStamped,
                queue_size=1,
            )
            for name in self.names
        }
        self.path_pubs = {
            name: rospy.Publisher(
                "/air_ground/%s/path" % name,
                Path,
                queue_size=1,
                latch=True,
            )
            for name in self.names
        }
        self.paths = {name: Path() for name in self.names}
        self.last_positions = {name: None for name in self.names}
        for path in self.paths.values():
            path.header.frame_id = "map"
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=1)

    def callback(self, msg):
        stamp = rospy.Time.now()
        indices = {name: index for index, name in enumerate(msg.name)}
        for name in self.names:
            if name not in indices:
                continue
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose = msg.pose[indices[name]]
            self.pose_pubs[name].publish(pose_msg)

            current = pose_msg.pose.position
            previous = self.last_positions[name]
            moved = previous is None or math.sqrt(
                (current.x - previous[0]) ** 2
                + (current.y - previous[1]) ** 2
                + (current.z - previous[2]) ** 2
            ) >= 0.05
            if moved:
                self.paths[name].poses.append(pose_msg)
                self.paths[name].poses = self.paths[name].poses[-1000:]
                self.last_positions[name] = (current.x, current.y, current.z)
            self.paths[name].header.stamp = stamp
            self.path_pubs[name].publish(self.paths[name])


if __name__ == "__main__":
    rospy.init_node("uav_rviz_state")
    UavRvizState()
    rospy.spin()

