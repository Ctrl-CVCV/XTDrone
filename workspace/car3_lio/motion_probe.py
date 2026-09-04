#!/usr/bin/env python3
# M2 运动验证: 用稳定发布者驱动 car0 前移 ~1.2m, 对比 /car0/lidar_slam/odom(LIO)
# 与 /car0/odom(gazebo 真值 odom node) 的位移。不用 rostopic pub(分叉生命周期坑)。
import rospy, math, sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
DUR   = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

def pose(o):
    return (o.pose.pose.position.x, o.pose.pose.position.y,
            o.pose.pose.orientation.z, o.pose.pose.orientation.w)

rospy.init_node('motion_probe', anonymous=True)
pub = rospy.Publisher('/car0/cmd_vel', Twist, queue_size=1, latch=False)
lio = None; tru = None
def cblio(m): global lio; lio = m
def cbtrue(m): global tru; tru = m
rospy.Subscriber('/car0/lidar_slam/odom', Odometry, cblio)
rospy.Subscriber('/car0/odom', Odometry, cbtrue)
# 等两个 odom 首帧
t0 = rospy.Time.now()
while (lio is None or tru is None) and (rospy.Time.now() - t0).to_sec() < 15:
    rospy.sleep(0.2)
if lio is None or tru is None:
    print("PROBE FAIL: missing odom", "lio" if lio is None else "", "truth" if tru is None else "")
    sys.exit(1)
def quat_yaw(z, w):
    return math.atan2(2*(w*z), 1 - 2*z*z)

lio_a = (lio.pose.pose.position.x, lio.pose.pose.position.y, quat_yaw(lio.pose.pose.orientation.z, lio.pose.pose.orientation.w))
tru_a = (tru.pose.pose.position.x, tru.pose.pose.position.y, quat_yaw(tru.pose.pose.orientation.z, tru.pose.pose.orientation.w))
print("PROBE start  LIO=(%.3f,%.3f,yaw%.3f) truth=(%.3f,%.3f,yaw%.3f)" % (*lio_a[:2], lio_a[2], *tru_a[:2], tru_a[2]))

tw = Twist(); tw.linear.x = SPEED
rate = rospy.Rate(30); end = rospy.Time.now() + rospy.Duration(DUR)
while rospy.Time.now() < end:
    pub.publish(tw); rate.sleep()
pub.publish(Twist())  # stop

rospy.sleep(1.0)
lio_b = (lio.pose.pose.position.x, lio.pose.pose.position.y, quat_yaw(lio.pose.pose.orientation.z, lio.pose.pose.orientation.w))
tru_b = (tru.pose.pose.position.x, tru.pose.pose.position.y, quat_yaw(tru.pose.pose.orientation.z, tru.pose.pose.orientation.w))
d_lio = math.hypot(lio_b[0]-lio_a[0], lio_b[1]-lio_a[1])
d_tru = math.hypot(tru_b[0]-tru_a[0], tru_b[1]-tru_a[1])
print("PROBE end    LIO=(%.3f,%.3f,yaw%.3f) truth=(%.3f,%.3f,yaw%.3f)" % (*lio_b[:2], lio_b[2], *tru_b[:2], tru_b[2]))
print("PROBE RESULT dist LIO=%.3f  truth=%.3f  err=%.3f m   dir_err=%.2f deg" %
      (d_lio, d_tru, d_lio-d_tru, math.degrees(lio_b[2]-tru_b[2])))
