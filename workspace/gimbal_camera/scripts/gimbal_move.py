#!/usr/bin/env python3
# 云台相机演示控制：yaw/roll 正弦摆动，云台持续转动以验证相机画面随动
# 也可交互控制，例如：
#   rostopic pub -1 /gimbal/gimbal_yaw_controller/command std_msgs/Float64 "data: 0.8"
#   rostopic pub -1 /gimbal/gimbal_roll_controller/command std_msgs/Float64 "data: 0.5"
import math
import sys

import rospy
from std_msgs.msg import Float64

YAW_AMP = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2
YAW_PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
ROLL_AMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.9
ROLL_PERIOD = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0

rospy.init_node('gimbal_move')
yaw_pub = rospy.Publisher('/gimbal/gimbal_yaw_controller/command', Float64, queue_size=1)
roll_pub = rospy.Publisher('/gimbal/gimbal_roll_controller/command', Float64, queue_size=1)
rate = rospy.Rate(30)
t0 = rospy.Time.now().to_sec()
rospy.loginfo('gimbal_move: yaw amp=%.2f T=%.1fs, roll amp=%.2f T=%.1fs' %
              (YAW_AMP, YAW_PERIOD, ROLL_AMP, ROLL_PERIOD))
while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec() - t0
    yaw_pub.publish(Float64(YAW_AMP * math.sin(2 * math.pi * t / YAW_PERIOD)))
    roll_pub.publish(Float64(ROLL_AMP * math.sin(2 * math.pi * t / ROLL_PERIOD + 0.5)))
    rate.sleep()
