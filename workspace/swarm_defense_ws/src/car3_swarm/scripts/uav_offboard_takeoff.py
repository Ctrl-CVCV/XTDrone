#!/usr/bin/env python3
"""Start/verify the XTDrone bridge, switch two UAVs to OFFBOARD, arm and take off."""

import argparse
import os
import subprocess
import sys
import time

import rosnode
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String


UAVS = ("iris_0", "iris_1")
BRIDGE_DIR = "/home/dev/XTDrone/communication"
BRIDGE_SCRIPT = os.path.join(BRIDGE_DIR, "multirotor_communication.py")


class UavStatus:
    def __init__(self, name):
        self.name = name
        self.state = None
        self.pose = None
        self.vision_pose_count = 0
        self.camera_pose_count = 0
        self.setpoint_count = 0
        rospy.Subscriber("/%s/mavros/state" % name, State, self._state_cb, queue_size=1)
        self._pose_sub = rospy.Subscriber(
            "/%s/mavros/local_position/pose" % name,
            PoseStamped,
            self._pose_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/%s/mavros/vision_pose/pose" % name,
            PoseStamped,
            self._vision_pose_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/%s/camera_pose" % name,
            PoseStamped,
            self._camera_pose_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/%s/mavros/setpoint_raw/local" % name,
            PositionTarget,
            self._setpoint_cb,
            queue_size=10,
        )
        self.cmd_pub = rospy.Publisher(
            "/xtdrone/%s/cmd" % name, String, queue_size=5
        )
        self.pose_pub = rospy.Publisher(
            "/xtdrone/%s/cmd_pose_enu" % name, Pose, queue_size=5
        )
        self.set_mode = rospy.ServiceProxy(
            "/%s/mavros/set_mode" % name, SetMode
        )
        self.arm = rospy.ServiceProxy(
            "/%s/mavros/cmd/arming" % name, CommandBool
        )

    def _state_cb(self, msg):
        self.state = msg


    def _pose_cb(self, msg):
        self.pose = msg

    def _vision_pose_cb(self, _msg):
        self.vision_pose_count += 1

    def _camera_pose_cb(self, _msg):
        self.camera_pose_count += 1

    def _setpoint_cb(self, _msg):
        self.setpoint_count += 1


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        rate.sleep()
    raise RuntimeError("等待超时：%s" % description)


def start_missing_bridges():
    started = []
    try:
        node_names = set(rosnode.get_node_names())
    except rosnode.ROSNodeIOException:
        node_names = set()

    for index, name in enumerate(UAVS):
        node_name = "/%s_communication" % name
        if node_name in node_names:
            rospy.loginfo("%s 通信桥已运行", name)
            continue
        rospy.loginfo("启动 %s XTDrone 通信桥", name)
        process = subprocess.Popen(
            [sys.executable, BRIDGE_SCRIPT, "iris", str(index)],
            cwd=BRIDGE_DIR,
        )
        started.append(process)
    return started


def publish_hover(statuses):
    for status in statuses:
        status.cmd_pub.publish(String(data="HOVER"))
    rospy.sleep(1.0)


def request_mode(status, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if status.state and status.state.mode == "OFFBOARD":
            return
        response = status.set_mode(base_mode=0, custom_mode="OFFBOARD")
        if not response.mode_sent:
            rospy.logwarn("%s OFFBOARD 请求被拒绝，准备重试", status.name)
        rospy.sleep(1.0)
    raise RuntimeError("%s 无法进入 OFFBOARD" % status.name)


def request_arm(status, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if status.state and status.state.armed:
            return
        response = status.arm(value=True)
        if not response.success:
            rospy.logwarn("%s ARM 被拒绝，准备重试", status.name)
        rospy.sleep(1.0)
    raise RuntimeError("%s 无法 ARM" % status.name)


def make_takeoff_pose(status, altitude):
    target = Pose()
    target.position.x = status.pose.pose.position.x
    target.position.y = status.pose.pose.position.y
    target.position.z = altitude
    target.orientation = status.pose.pose.orientation
    if (
        target.orientation.x == 0.0
        and target.orientation.y == 0.0
        and target.orientation.z == 0.0
        and target.orientation.w == 0.0
    ):
        target.orientation.w = 1.0
    return target


def main():
    parser = argparse.ArgumentParser(description="两架 Iris 一键 OFFBOARD、ARM、起飞")
    parser.add_argument("--altitude", type=float, default=1.5, help="本地起飞高度，默认 1.5 m")
    parser.add_argument("--timeout", type=float, default=30.0, help="每个阶段超时秒数")
    parser.add_argument(
        "--no-start-bridge",
        action="store_true",
        help="不自动启动缺失的 XTDrone 通信桥",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    if args.altitude < 0.5:
        raise RuntimeError("起飞高度不能低于 0.5 m")

    rospy.init_node("uav_offboard_takeoff", anonymous=False)
    bridge_processes = []
    try:
        statuses = [UavStatus(name) for name in UAVS]
        wait_until(
            lambda: all(s.vision_pose_count >= 3 for s in statuses),
            args.timeout,
            "Gazebo ground-truth vision pose；请先运行 get_local_pose.py iris 2",
        )
        wait_until(
            lambda: all(s.camera_pose_count >= 3 for s in statuses),
            args.timeout,
            "EGO camera_pose；请先运行 ego_swarm_transfer.py iris 2",
        )

        if not args.no_start_bridge:
            bridge_processes = start_missing_bridges()

        wait_until(
            lambda: all(s.state and s.state.connected and s.pose for s in statuses),
            args.timeout,
            "两套 MAVROS connected 且 local pose 有数据",
        )

        # Give publishers/subscribers time to connect, then let the bridge hold position.
        wait_until(
            lambda: all(s.cmd_pub.get_num_connections() > 0 for s in statuses),
            args.timeout,
            "两套 XTDrone 通信桥订阅控制命令",
        )
        publish_hover(statuses)

        for status in statuses:
            status.setpoint_count = 0
        rospy.sleep(1.0)
        if not all(s.setpoint_count >= 3 for s in statuses):
            raise RuntimeError("通信桥没有持续发布 MAVROS setpoint，拒绝切换 OFFBOARD")

        for status in statuses:
            request_mode(status, args.timeout)
        for status in statuses:
            request_arm(status, args.timeout)

        # Keep the XTDrone bridge control latch in sync with the MAVROS mode.
        # This releases the initial HOVER hold before takeoff pose streaming.
        for status in statuses:
            status.cmd_pub.publish(String(data="OFFBOARD"))
        rospy.sleep(0.5)

        targets = [make_takeoff_pose(s, args.altitude) for s in statuses]
        rospy.loginfo("两机已进入 OFFBOARD 并 ARM，开始原地起飞到 %.2f m", args.altitude)
        rate = rospy.Rate(10)
        for _ in range(30):
            for status, target in zip(statuses, targets):
                status.pose_pub.publish(target)
            rate.sleep()

        wait_until(
            lambda: all(
                s.pose and s.pose.pose.position.z >= args.altitude - 0.2
                for s in statuses
            ),
            args.timeout,
            "两架飞机到达起飞高度",
        )
        rospy.loginfo("成功：iris_0、iris_1 均已 OFFBOARD、ARM，并到达起飞高度")
        rospy.loginfo("脚本将保持自动启动的通信桥；现在可以发布 EGO 目标。降落后按 Ctrl+C。")
        rospy.spin()
    finally:
        for process in bridge_processes:
            if process.poll() is None:
                process.terminate()
        for process in bridge_processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("一键起飞失败：%s", exc)
        sys.exit(1)

