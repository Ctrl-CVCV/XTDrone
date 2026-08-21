#!/usr/bin/env python3
"""Send four world-coordinate goals to iris_0, iris_1, car0 and car1."""

import argparse
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


UAV_NAMES = ("iris_0", "iris_1")
CAR_NAMES = ("car0", "car1")


class GoalSender:
    def __init__(self):
        self.world_poses = {}
        self.local_poses = {}
        self.goal_publishers = {
            name: rospy.Publisher(
                "/%s/move_base_simple/goal" % name,
                PoseStamped,
                queue_size=1,
            )
            for name in UAV_NAMES + CAR_NAMES
        }
        rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self._model_states_cb,
            queue_size=1,
        )
        for name in UAV_NAMES:
            rospy.Subscriber(
                "/%s/mavros/local_position/pose" % name,
                PoseStamped,
                self._local_pose_cb,
                callback_args=name,
                queue_size=1,
            )

    def _model_states_cb(self, msg):
        indices = {name: index for index, name in enumerate(msg.name)}
        for name in UAV_NAMES:
            if name in indices:
                self.world_poses[name] = msg.pose[indices[name]]

    def _local_pose_cb(self, msg, name):
        self.local_poses[name] = msg.pose

    def ready_for_world_conversion(self):
        return all(
            name in self.world_poses and name in self.local_poses
            for name in UAV_NAMES
        )

    def all_goal_subscribers_ready(self):
        return all(
            publisher.get_num_connections() > 0
            for publisher in self.goal_publishers.values()
        )

    def world_to_uav_local(self, name, world_goal):
        """Translate a world goal into the current MAVROS local ENU frame."""
        world_pose = self.world_poses[name].position
        local_pose = self.local_poses[name].position
        origin_x = world_pose.x - local_pose.x
        origin_y = world_pose.y - local_pose.y
        origin_z = world_pose.z - local_pose.z
        return (
            world_goal[0] - origin_x,
            world_goal[1] - origin_y,
            world_goal[2] - origin_z,
        )

    @staticmethod
    def make_goal(x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def send(self, world_goals, dry_run=False):
        published_goals = {}
        for name in UAV_NAMES:
            local_goal = self.world_to_uav_local(name, world_goals[name])
            published_goals[name] = local_goal
            rospy.loginfo(
                "%s 世界目标 (%.2f, %.2f, %.2f) -> EGO 本地目标 (%.2f, %.2f, %.2f)",
                name,
                *world_goals[name],
                *local_goal,
            )

        for name in CAR_NAMES:
            x, y = world_goals[name]
            published_goals[name] = (x, y, 0.0)
            rospy.loginfo("%s 全局地图目标 (%.2f, %.2f)", name, x, y)

        if dry_run:
            rospy.logwarn("dry-run：仅完成坐标换算，没有发布目标")
            return

        stamp = rospy.Time.now()
        for name, goal in published_goals.items():
            msg = self.make_goal(*goal)
            msg.header.stamp = stamp
            self.goal_publishers[name].publish(msg)

        # Keep publishers alive briefly so all four one-shot messages leave the queue.
        rospy.sleep(0.5)
        rospy.loginfo("四组目标已发布：iris_0、iris_1、car0、car1")


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        rate.sleep()
    raise RuntimeError("等待超时：%s" % description)


def parse_args():
    parser = argparse.ArgumentParser(
        description="向两架 Iris 和两辆 defender 发布四组 Gazebo 世界坐标目标"
    )
    parser.add_argument(
        "--iris0", nargs=3, required=True, type=float, metavar=("X", "Y", "Z")
    )
    parser.add_argument(
        "--iris1", nargs=3, required=True, type=float, metavar=("X", "Y", "Z")
    )
    parser.add_argument(
        "--car0", nargs=2, required=True, type=float, metavar=("X", "Y")
    )
    parser.add_argument(
        "--car1", nargs=2, required=True, type=float, metavar=("X", "Y")
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--dry-run", action="store_true", help="只换算并打印目标，不向 ROS 发布"
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("send_air_ground_goals", anonymous=True)
    sender = GoalSender()

    wait_until(
        sender.ready_for_world_conversion,
        args.timeout,
        "两架飞机的 Gazebo 世界位姿和 MAVROS local pose",
    )
    if not args.dry_run:
        wait_until(
            sender.all_goal_subscribers_ready,
            args.timeout,
            "两套 EGO 和两套车辆 Nav 目标订阅者",
        )

    world_goals = {
        "iris_0": tuple(args.iris0),
        "iris_1": tuple(args.iris1),
        "car0": tuple(args.car0),
        "car1": tuple(args.car1),
    }
    sender.send(world_goals, dry_run=args.dry_run)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException) as exc:
        rospy.logerr("目标发布失败：%s", exc)
        sys.exit(1)

