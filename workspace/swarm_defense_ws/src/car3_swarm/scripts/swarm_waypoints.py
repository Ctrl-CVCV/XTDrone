#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V2 multi-waypoint navigation shared module.

run_waypoints(car, waypoints, timeout) sends each (x, y, yaw) to
/carN/move_base (MoveBaseAction served by nav_to_pose_node) SEQUENTIALLY:
every goal waits for its terminal state (SUCCEEDED/ABORTED/PREEMPTED) before
the next one is sent -- a later waypoint can never overwrite an earlier one.
ABORTED points are logged and skipped; Ctrl+C cancels safely.
"""
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler


def make_goal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal


def run_waypoints(car, waypoints, timeout=90.0):
    rospy.init_node("waypoints_%s" % car, anonymous=True)
    client = actionlib.SimpleActionClient("/%s/move_base" % car, MoveBaseAction)

    rospy.loginfo("[%s] 等待 move_base 动作服务器..." % car)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("[%s] 30s 内未等到 /%s/move_base 服务器, 退出" % (car, car))
        return

    rospy.loginfo("[%s] 共 %d 个导航点, 依次执行 (每点超时 %.0fs)" %
                  (car, len(waypoints), timeout))
    for i, (x, y, yaw) in enumerate(waypoints):
        if rospy.is_shutdown():
            break
        rospy.loginfo("[%s] 发送目标 %d/%d: (%.2f, %.2f, yaw=%.3f)" %
                      (car, i + 1, len(waypoints), x, y, yaw))
        client.send_goal(make_goal(x, y, yaw))

        # block until terminal state: the next waypoint is never sent while
        # this one is still active
        finished = client.wait_for_result(rospy.Duration(timeout))
        state = client.get_state()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[%s] 目标 %d/%d 到达 (SUCCEEDED)" % (car, i + 1, len(waypoints)))
        elif state == GoalStatus.ABORTED:
            rospy.logwarn("[%s] 目标 %d/%d 中止 (ABORTED), 继续下一个" %
                          (car, i + 1, len(waypoints)))
        elif state == GoalStatus.PREEMPTED:
            rospy.logwarn("[%s] 目标 %d/%d 被抢占 (PREEMPTED), 继续下一个" %
                          (car, i + 1, len(waypoints)))
        elif not finished:
            rospy.logwarn("[%s] 目标 %d/%d 超时 (%.0fs), 取消并继续下一个" %
                          (car, i + 1, len(waypoints), timeout))
            client.cancel_goal()
        else:
            rospy.logwarn("[%s] 目标 %d/%d 终态 %s, 继续下一个" %
                          (car, i + 1, len(waypoints), str(state)))

    rospy.loginfo("[%s] 全部导航点执行完毕" % car)
