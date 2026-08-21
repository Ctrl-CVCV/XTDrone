#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

DRONES = {"iris_0": {"start": (-2.3, 2.3), "goal": (2.0, -2.0, 1.5)}, "iris_1": {"start": (2.3, -2.3), "goal": (-2.0, 2.0, 1.5)}}
CARS = {"car0": (0.0, -1.5, 0.0), "car1": (-1.0, -0.5, 0.0), "car2": (-2.5, -1.0, 0.0)}

def pose(x, y, z):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    return msg

def wait_connected(name):
    state = {"msg": None}
    sub = rospy.Subscriber("/%s/mavros/state" % name, State, lambda msg: state.__setitem__("msg", msg), queue_size=1)
    deadline = rospy.Time.now() + rospy.Duration(30.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if state["msg"] and state["msg"].connected:
            sub.unregister()
            return True
        rospy.sleep(0.1)
    sub.unregister()
    return False

def takeoff(name, x, y, z):
    pub = rospy.Publisher("/%s/mavros/setpoint_position/local" % name, PoseStamped, queue_size=20)
    rospy.sleep(1.0)
    msg = pose(x, y, z)
    rate = rospy.Rate(20)
    for _ in range(100):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    rospy.wait_for_service("/%s/mavros/set_mode" % name, timeout=10)
    rospy.wait_for_service("/%s/mavros/cmd/arming" % name, timeout=10)
    set_mode = rospy.ServiceProxy("/%s/mavros/set_mode" % name, SetMode)
    arm = rospy.ServiceProxy("/%s/mavros/cmd/arming" % name, CommandBool)
    set_mode(custom_mode="OFFBOARD")
    arm(value=True)
    return pub, msg

def publish_goal(topic, x, y, z):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=5)
    msg = pose(x, y, z)
    rospy.sleep(0.2)
    for _ in range(20):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(0.05)

def main():
    rospy.init_node("air_ground_unified_control")
    for name in DRONES:
        if not wait_connected(name):
            rospy.logerr("MAVROS connection timeout: %s", name)
            return
    drone_msgs = []
    for name, cfg in DRONES.items():
        pub, msg = takeoff(name, cfg["start"][0], cfg["start"][1], cfg["goal"][2])
        drone_msgs.append((pub, msg))
    rospy.loginfo("Both aircraft requested OFFBOARD and ARM; publishing air/ground goals")
    for name, cfg in DRONES.items():
        publish_goal("/%s/move_base_simple/goal" % name, *cfg["goal"])
    for name, goal in CARS.items():
        publish_goal("/%s/move_base_simple/goal" % name, goal[0], goal[1], 0.0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for pub, msg in drone_msgs:
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
        pass
