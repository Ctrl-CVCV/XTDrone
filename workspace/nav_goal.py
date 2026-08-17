#!/usr/bin/env python3
import sys
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations

# usage: python3 nav_goal.py "x1,y1,yaw1" "x2,y2,yaw2" ...
goals = []
for a in sys.argv[1:]:
    x, y, yaw = [float(v) for v in a.split(',')]
    goals.append((x, y, yaw))

rospy.init_node('nav_goal_test')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
if not client.wait_for_server(rospy.Duration(30)):
    print('MOVE_BASE SERVER NOT FOUND')
    sys.exit(1)
print('move_base server up')

ok = 0
for i, (x, y, yaw) in enumerate(goals):
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.orientation.z, g.target_pose.pose.orientation.w = \
        tf.transformations.quaternion_from_euler(0, 0, yaw)[2:]
    client.send_goal(g)
    done = client.wait_for_result(rospy.Duration(90))
    st = client.get_state()
    print('GOAL %d (%s): state=%d %s' % (i, (x, y, yaw), st,
          'SUCCESS' if st == 3 else 'FAIL'))
    if st == 3:
        ok += 1
    rospy.sleep(1.0)

print('RESULT: %d/%d goals reached' % (ok, len(goals)))
