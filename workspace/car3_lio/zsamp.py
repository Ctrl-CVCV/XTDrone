#!/usr/bin/env python3
import rospy, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

gz = {"z": 0.0}
lio = {"z": 0.0}
vp = {"z": 0.0}

def gzcb(m):
    if "iris_0" in m.name:
        gz["z"] = m.pose[m.name.index("iris_0")].position.z

def lcb(m):
    lio["z"] = m.pose.pose.position.z

def vcb(m):
    vp["z"] = m.pose.position.z

rospy.init_node("zsamp")
rospy.Subscriber("/gazebo/model_states", ModelStates, gzcb, queue_size=1)
rospy.Subscriber("/quad0/lidar_slam/odom", Odometry, lcb, queue_size=1)
rospy.Subscriber("/quad0/lidar_slam/vision_pose_raw", PoseStamped, vcb, queue_size=1)
t0 = time.time()
while time.time() - t0 < 80:
    print("%.1f gz=%.3f lio=%.3f vp=%.3f" % (time.time() - t0, gz["z"], lio["z"], vp["z"]), flush=True)
    time.sleep(1.0)
