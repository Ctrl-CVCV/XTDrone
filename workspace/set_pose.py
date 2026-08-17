#!/usr/bin/env python3
"""Teleport car3 back to world (0,0,0) and reset AMCL to map (0,0)."""
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('set_pose', anonymous=True, disable_signals=True)
rospy.wait_for_service('/gazebo/set_model_state')
ms = ModelState()
ms.model_name = 'car3'
ms.pose.position.x = 0.0
ms.pose.position.y = 0.0
ms.pose.position.z = 0.0
ms.pose.orientation.w = 1.0
ms.twist.linear.x = ms.twist.linear.y = ms.twist.angular.z = 0.0
ms.reference_frame = 'world'
print('teleport:', rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(ms).status_message)

rospy.sleep(0.5)
p = PoseWithCovarianceStamped()
p.header.frame_id = 'map'
p.header.stamp = rospy.Time.now()
p.pose.pose.position.x = 0.0
p.pose.pose.position.y = 0.0
p.pose.pose.orientation.w = 1.0
p.pose.covariance[0] = 0.04
p.pose.covariance[7] = 0.04
p.pose.covariance[35] = 0.02
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
rospy.sleep(0.3)
pub.publish(p)
print('AMCL initialpose reset to (0,0,0)')
