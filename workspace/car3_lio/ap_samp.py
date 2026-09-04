#!/usr/bin/env python3
import math
import rospy, time
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

gz = {"z": 0.0, "r": 0.0, "p": 0.0, "y": 0.0}
vp = {"r": 0.0, "p": 0.0}

def quat_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    ya = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(r), math.degrees(p), math.degrees(ya)

def gzcb(m):
    if "iris_0" in m.name:
        i = m.name.index("iris_0")
        pos = m.pose[i].position
        r, p, y = quat_rpy(m.pose[i].orientation)
        gz.update(z=pos.z, r=r, p=p, y=y)

def vcb(m):
    r, p, _ = quat_rpy(m.pose.orientation)
    vp.update(r=r, p=p)

rospy.init_node("apsamp")
rospy.Subscriber("/gazebo/model_states", ModelStates, gzcb, queue_size=1)
rospy.Subscriber("/quad0/lidar_slam/vision_pose_raw", PoseStamped, vcb, queue_size=1)
t0 = time.time()
while time.time() - t0 < 90:
    print("%.1f gz=%.3f gRPY=%.1f,%.1f,%.1f  LIO_rp=%.1f,%.1f"
          % (time.time() - t0, gz["z"], gz["r"], gz["p"], gz["y"], vp["r"], vp["p"]), flush=True)
    time.sleep(1.0)
