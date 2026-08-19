#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record /carN/shared_pose trajectories to a file for post-analysis.

Usage:
    python3 record_trajectory.py --cars car0 car2 --duration 60 --file /tmp/traj.log
"""
import argparse
import rospy
import time
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cars", nargs="+", default=["car0", "car1", "car2"])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--file", default="/tmp/trajectory.log")
    ap.add_argument("--hz", type=float, default=5.0)
    args = ap.parse_args()

    rospy.init_node("record_trajectory", anonymous=True)
    dt = 1.0 / args.hz
    t0 = time.time()
    with open(args.file, "w", buffering=1) as f:
        f.write("# t car x y yaw\n")
        while time.time() - t0 < args.duration and not rospy.is_shutdown():
            for c in args.cars:
                try:
                    m = rospy.wait_for_message("/" + c + "/shared_pose",
                                               Odometry, timeout=0.8)
                    q = m.pose.pose.orientation
                    import tf.transformations as tfm
                    _, _, yaw = tfm.euler_from_quaternion(
                        [q.x, q.y, q.z, q.w])
                    f.write("%.2f %s %.3f %.3f %.3f\n" %
                            (time.time() - t0, c,
                             m.pose.pose.position.x, m.pose.pose.position.y, yaw))
                except rospy.ROSException:
                    pass
            time.sleep(dt)
    rospy.loginfo("trajectory saved to %s" % args.file)


if __name__ == "__main__":
    main()
