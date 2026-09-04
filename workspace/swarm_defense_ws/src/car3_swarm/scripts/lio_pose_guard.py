#!/usr/bin/env python3
"""Reject implausible SWARM-LIO vision-pose samples before MAVROS.

SWARM-LIO remains the position source.  This node only rejects a sample when
the estimator has clearly lost registration (large roll/pitch or an
impossible position jump).  During a rejection it republishes the last valid
pose with the current input timestamp, which keeps PX4's external-vision
stream alive without feeding a runaway estimate into EKF2.
"""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped


def norm_q(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (q.x/n, q.y/n, q.z/n, q.w/n)


def roll_pitch(q):
    x, y, z, w = norm_q(q)
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sp = max(-1.0, min(1.0, 2.0*(w*y - z*x)))
    return math.degrees(roll), math.degrees(math.asin(sp))


class LioPoseGuard:
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "~input")
        self.output_topic = rospy.get_param("~output", "~output")
        self.max_tilt = float(rospy.get_param("~max_tilt_deg", 35.0))
        self.max_step = float(rospy.get_param("~max_step", 0.80))
        self.max_speed = float(rospy.get_param("~max_speed", 5.0))
        self.last_valid = None
        # Detect a discontinuity against the previous raw sample, not the
        # previous accepted sample.  With the latter, one rejected pose while
        # the UAV kept moving could make every later pose exceed max_step and
        # permanently freeze external vision at an old position.
        self.last_input = None
        self.last_input_wall = None
        self.invalid_count = 0
        self.valid_count = 0
        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.input_topic, PoseStamped, self.callback, queue_size=10)
        # When the LIO world's vertical axis is not observable (e.g. baro is the
        # PX4 height source), feeding the raw LIO-Z into external vision drags the
        # EKF altitude.  Optionally override the output Z with PX4's own baro
        # local Z so external-vision Z is self-consistent while XY/yaw stay LIO.
        self.baro_z_topic = rospy.get_param("~baro_z_topic", "")
        self.baro_z = None
        if self.baro_z_topic:
            rospy.Subscriber(self.baro_z_topic, PoseStamped, self.baro_cb, queue_size=5)
        rospy.loginfo("LIO pose guard: %s -> %s, tilt<=%.1f deg, step<=%.2f m, speed<=%.1f m/s%s",
                      self.input_topic, self.output_topic, self.max_tilt,
                      self.max_step, self.max_speed,
                      "  (Z override: %s)" % self.baro_z_topic if self.baro_z_topic else "")

    def baro_cb(self, m):
        self.baro_z = m.pose.position.z

    def _out(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        if self.baro_z_topic and self.baro_z is not None:
            out.pose.position.z = self.baro_z
        return out

    def callback(self, msg):
        p = msg.pose.position
        values = (p.x, p.y, p.z, msg.pose.orientation.x,
                  msg.pose.orientation.y, msg.pose.orientation.z,
                  msg.pose.orientation.w)
        finite = all(math.isfinite(v) for v in values)
        r, pitch = roll_pitch(msg.pose.orientation) if finite else (999.0, 999.0)
        now = time.monotonic()
        bad = not finite or abs(r) > self.max_tilt or abs(pitch) > self.max_tilt
        step = speed = 0.0
        dt = None
        if self.last_input is not None and self.last_input_wall is not None:
            dt = max(now - self.last_input_wall, 1e-3)
            q = self.last_input.pose.position
            step = math.sqrt((p.x-q.x)**2 + (p.y-q.y)**2 + (p.z-q.z)**2)
            speed = step / dt
            if step > self.max_step or speed > self.max_speed:
                bad = True
        # Advance the raw reference even for a rejected sample so a finite,
        # smooth estimate can recover on the following frames.
        if finite:
            self.last_input = msg
            self.last_input_wall = now

        if bad:
            self.invalid_count += 1
            self.valid_count = 0
            rospy.logwarn_throttle(
                1.0,
                "LIO guard rejected sample: roll=%.1f pitch=%.1f step=%.2f speed=%.2f invalid=%d",
                r, pitch, step, speed, self.invalid_count)
            if self.last_valid is None:
                return
            self.pub.publish(self._out(self.last_valid))
            return

        if self.invalid_count:
            rospy.loginfo("LIO guard recovered after %d rejected samples", self.invalid_count)
        self.invalid_count = 0
        self.valid_count += 1
        self.last_valid = msg
        self.pub.publish(self._out(msg))


if __name__ == "__main__":
    rospy.init_node("lio_pose_guard", anonymous=False)
    LioPoseGuard()
    rospy.spin()
