#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start/stop patrol mode on both defender cars.

Usage:
    rosrun car3_swarm patrol_ctl.py start
    rosrun car3_swarm patrol_ctl.py stop
"""
import sys
import rospy
from std_srvs.srv import Trigger

CARS = ["car1", "car0"]  # Car A (SW) first, then Car B (NE)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("start", "stop"):
        print("usage: patrol_ctl.py start|stop")
        sys.exit(1)
    action = sys.argv[1]
    rospy.init_node("patrol_ctl", anonymous=True)
    for car in CARS:
        srv_name = "/%s/%s_patrol" % (car, action)
        try:
            rospy.wait_for_service(srv_name, timeout=10.0)
        except rospy.ROSException:
            print("[%s] service %s not available" % (car, srv_name))
            continue
        try:
            r = rospy.ServiceProxy(srv_name, Trigger)()
            print("[%s] %s_patrol: success=%s, msg='%s'" %
                  (car, action, r.success, r.message))
        except rospy.ServiceException as e:
            print("[%s] service call failed: %s" % (car, e))


if __name__ == "__main__":
    main()
