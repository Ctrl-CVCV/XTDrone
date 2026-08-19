#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-car patrol node (巡检模式).

Flow (state machine):
  IDLE -> MOVE_FORWARD -> COMPUTE_PATROL_ANGLES -> ALIGN_FIRST_DOOR
       -> DWELL -> SWEEP_TO_SECOND_DOOR -> DWELL -> SWEEP_TO_FIRST_DOOR -> (loop)

- MOVE_FORWARD: closed-loop straight drive over ~forward_distance m, measured
  from the odometry position at patrol start (P0); yaw is held to the start
  heading so the car moves along its current forward direction.
- COMPUTE_PATROL_ANGLES: bearing to each assigned door computed from the
  actual dwell position (theta = atan2(dy, dx)); sweep direction is the
  shortest angular interval between the two bearings.
- Sweep states: rotation only (linear strictly zero) at ~max_omega until the
  yaw error (normalized to [-pi, pi]) is within ~angle_tolerance, then DWELL.
- start_patrol/stop_patrol services (std_srvs/Trigger); stop_patrol zeroes
  all velocities from ANY state and returns to IDLE.
- Containment (task level): on /mission/state entering containment the patrol
  FSM is interrupted (any sub-state) and CONTAINMENT_ACTIVE tracks the
  world-frame containment_goal (PL/PR) with pure omnidirectional motion
  (wz = 0, yaw locked at the init heading) plus scan repulsion.

Topics (relative to the car namespace): sub odom, pub cmd_vel, pub patrol_state.
"""
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import tf.transformations as tfm

IDLE = "IDLE"
MOVE_FORWARD = "MOVE_FORWARD"
COMPUTE_PATROL_ANGLES = "COMPUTE_PATROL_ANGLES"
ALIGN_FIRST_DOOR = "ALIGN_FIRST_DOOR"
DWELL = "DWELL"
SWEEP_TO_SECOND_DOOR = "SWEEP_TO_SECOND_DOOR"
SWEEP_TO_FIRST_DOOR = "SWEEP_TO_FIRST_DOOR"
# multi-round pre-phase: after a capture the car rests at an arbitrary pose;
# srv_start re-homes to the launch pose before the (unchanged) patrol loop
RETURN_HOME = "RETURN_HOME"
# containment states (task level, added above the patrol FSM; the patrol
# states themselves are unchanged)
CONTAINMENT_INIT = "CONTAINMENT_INIT"
CONTAINMENT_ACTIVE = "CONTAINMENT_ACTIVE"

PATROL_STATES = (IDLE, MOVE_FORWARD, COMPUTE_PATROL_ANGLES, ALIGN_FIRST_DOOR,
                 DWELL, SWEEP_TO_SECOND_DOOR, SWEEP_TO_FIRST_DOOR,
                 RETURN_HOME)
# mission states that force an immediate switch out of ANY patrol sub-state
MISSION_CONTAINMENT = ("M_CONTAINMENT_INIT", "M_CONTAINMENT_ACTIVE",
                       "M_CAPTURE_CONFIRMED", "M_FINAL_ALIGN", "M_SUCCESS")
MISSION_ENDED = ("M_FAILED_ESCAPE", "M_INVALID_ESCAPE", "M_IDLE")

MODE_BY_STATE = {
    IDLE: "STOP",
    DWELL: "STOP",
    MOVE_FORWARD: "FULL_MOTION",
    COMPUTE_PATROL_ANGLES: "STOP",
    ALIGN_FIRST_DOOR: "ROTATION_ONLY",
    SWEEP_TO_SECOND_DOOR: "ROTATION_ONLY",
    SWEEP_TO_FIRST_DOOR: "ROTATION_ONLY",
    RETURN_HOME: "FULL_MOTION",
    CONTAINMENT_INIT: "STOP",
    CONTAINMENT_ACTIVE: "TRANSLATION_ONLY",
}


def wrap_pi(a):
    """Normalize an angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PatrolNode(object):
    def __init__(self):
        rospy.init_node("patrol_node")

        self.forward_distance = rospy.get_param("~forward_distance", 2.0)
        self.patrol_speed = rospy.get_param("~patrol_speed", 0.35)
        self.max_omega = rospy.get_param("~max_omega", 0.50)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.052)
        self.dwell_time = rospy.get_param("~dwell_time", 0.4)
        self.hold_yaw_gain = rospy.get_param("~hold_yaw_gain", 2.0)
        rate = rospy.get_param("~rate", 20.0)
        doors = rospy.get_param("~doors", [[0.0, 2.975], [2.975, 0.0]])
        self.door1 = (float(doors[0][0]), float(doors[0][1]))
        self.door2 = (float(doors[1][0]), float(doors[1][1]))

        self.state = IDLE
        self.odom_pos = None
        self.odom_yaw = 0.0
        self.p0 = None
        self.yaw0 = 0.0
        self.launch_pos = None
        self.launch_yaw = 0.0
        self.home_blocked = False
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.dir12 = 1.0
        self.next_state = None
        self.state_enter_t = rospy.Time.now()
        self.mission_state = ""
        self.cont_yaw = 0.0
        self.cont_goal = None
        self.scan = None

        def p(key, default):
            return rospy.get_param("/mission/" + key, default)

        self.kp_track = p("motion/kp_track", 1.6)
        self.cont_max_v = p("motion/containment_max_vx", 0.55)
        # defenders use weaker repulsion than the intruder: they must be able
        # to enter the capture circle around E (doc: target keeps only the
        # minimal real-collision clearance, target_safe < friendly_safe)
        self.repulse_range = p("motion/defender_repulse_range",
                               p("motion/repulse_range", 0.55))
        self.repulse_gain = p("motion/defender_repulse_gain",
                              p("motion/repulse_gain", 0.7))
        self.align_max_omega = p("motion/final_align_max_omega", 0.60)
        self.align_tolerance = p("motion/final_align_tolerance", 0.05)

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("patrol_state", String, queue_size=1,
                                         latch=True)
        self.mode_pub = rospy.Publisher("control_mode", String, queue_size=1,
                                        latch=True)
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("/mission/state", String, self.mission_state_cb)
        rospy.Subscriber("containment_goal", Point, self.cont_goal_cb)
        rospy.Subscriber("scan_filtered", LaserScan, self.scan_cb)
        rospy.Service("start_patrol", Trigger, self.srv_start)
        rospy.Service("stop_patrol", Trigger, self.srv_stop)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.on_shutdown(self.publish_zero)
        self.state_pub.publish(String(self.state))
        self.mode_pub.publish(String(MODE_BY_STATE[self.state]))

        rospy.loginfo("patrol_node ready: doors %.3f,%.3f -> %.3f,%.3f | "
                      "fwd %.2f m @ %.2f m/s | sweep %.2f rad/s | "
                      "tol %.3f rad | dwell %.2f s | containment: "
                      "kp=%.2f vmax=%.2f repulse r=%.2f g=%.2f",
                      self.door1[0], self.door1[1],
                      self.door2[0], self.door2[1],
                      self.forward_distance, self.patrol_speed,
                      self.max_omega, self.angle_tolerance, self.dwell_time,
                      self.kp_track, self.cont_max_v,
                      self.repulse_range, self.repulse_gain)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tfm.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_yaw = yaw

    def mission_state_cb(self, msg):
        prev = self.mission_state
        self.mission_state = msg.data
        if msg.data != prev and self.state in (CONTAINMENT_INIT,
                                               CONTAINMENT_ACTIVE):
            mode = "ROTATION_ONLY" if msg.data == "M_FINAL_ALIGN" else "STOP"
            if msg.data in ("M_CAPTURE_CONFIRMED", "M_FINAL_ALIGN",
                            "M_SUCCESS"):
                self.mode_pub.publish(String(mode))

    def cont_goal_cb(self, msg):
        self.cont_goal = (msg.x, msg.y)

    def scan_cb(self, msg):
        self.scan = msg

    def _forward_blocked(self):
        """Return-home hold: stop while an obstacle (sibling car's virtual
        box) is close ahead, resume only after it clears (hysteresis)."""
        if self.scan is None:
            return False
        mn = float("inf")
        for i, r in enumerate(self.scan.ranges):
            a = self.scan.angle_min + i * self.scan.angle_increment
            if abs(a) > 1.05:
                continue
            if math.isfinite(r) and r < mn:
                mn = r
        if mn <= 0.25:
            self.home_blocked = True
        elif mn >= 0.45:
            self.home_blocked = False
        return self.home_blocked

    def set_state(self, s):
        self.state = s
        self.state_enter_t = rospy.Time.now()
        self.state_pub.publish(String(s))
        self.mode_pub.publish(String(MODE_BY_STATE.get(s, "STOP")))
        rospy.loginfo("patrol state -> %s", s)

    def publish_twist(self, vx, vy, wz):
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def publish_zero(self):
        try:
            self.publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass

    def srv_start(self, req):
        if self.mission_state in MISSION_CONTAINMENT:
            return TriggerResponse(False, "mission in containment (state=%s)"
                                   % self.mission_state)
        if self.state != IDLE:
            return TriggerResponse(True, "already patrolling (state=%s)" % self.state)
        if self.odom_pos is None:
            return TriggerResponse(False, "no odom data yet")
        if self.launch_pos is None:
            self.launch_pos = self.odom_pos
            self.launch_yaw = self.odom_yaw
        if math.hypot(self.odom_pos[0] - self.launch_pos[0],
                      self.odom_pos[1] - self.launch_pos[1]) > 0.3:
            # multi-round: re-home to the launch pose first, then run the
            # unchanged patrol loop from there
            self.p0 = self.launch_pos
            self.yaw0 = self.launch_yaw
            self.home_blocked = False
            self.set_state(RETURN_HOME)
            rospy.loginfo("patrol start: re-homing to (%.3f, %.3f) from "
                          "(%.3f, %.3f)", self.launch_pos[0],
                          self.launch_pos[1], self.odom_pos[0],
                          self.odom_pos[1])
            return TriggerResponse(True, "patrol started (re-homing)")
        self.p0 = self.odom_pos
        self.yaw0 = self.odom_yaw
        self.set_state(MOVE_FORWARD)
        rospy.loginfo("patrol start: P0=(%.3f, %.3f) yaw0=%.3f rad",
                      self.p0[0], self.p0[1], self.yaw0)
        return TriggerResponse(True, "patrol started")

    def srv_stop(self, req):
        if self.state == IDLE:
            return TriggerResponse(True, "already idle")
        self.publish_twist(0.0, 0.0, 0.0)
        self.set_state(IDLE)
        return TriggerResponse(True, "patrol stopped, velocities zeroed")

    def tick(self, _ev):
        if self.mission_state in MISSION_CONTAINMENT and \
                self.state in PATROL_STATES:
            self.publish_twist(0.0, 0.0, 0.0)
            rospy.loginfo("mission -> %s: interrupting patrol (state=%s)",
                          self.mission_state, self.state)
            self.set_state(CONTAINMENT_INIT)
            return
        if self.mission_state in MISSION_ENDED and self.state != IDLE:
            self.publish_twist(0.0, 0.0, 0.0)
            self.cont_goal = None
            self.set_state(IDLE)
            return
        if self.state == IDLE:
            self.publish_twist(0.0, 0.0, 0.0)
            return
        if self.odom_pos is None:
            return

        if self.state == RETURN_HOME:
            home_dir = math.atan2(self.launch_pos[1] - self.odom_pos[1],
                                  self.launch_pos[0] - self.odom_pos[0])
            dist = math.hypot(self.launch_pos[0] - self.odom_pos[0],
                              self.launch_pos[1] - self.odom_pos[1])
            if dist <= 0.15:
                err = wrap_pi(self.launch_yaw - self.odom_yaw)
                if abs(err) <= self.angle_tolerance:
                    self.publish_twist(0.0, 0.0, 0.0)
                    rospy.loginfo("re-home done at (%.3f, %.3f), patrol "
                                  "resumes", self.odom_pos[0],
                                  self.odom_pos[1])
                    self.set_state(MOVE_FORWARD)
                else:
                    self.publish_twist(0.0, 0.0,
                                       (1.0 if err >= 0.0 else -1.0) *
                                       self.max_omega)
            else:
                err = wrap_pi(home_dir - self.odom_yaw)
                if abs(err) > 0.10:
                    self.publish_twist(0.0, 0.0,
                                       (1.0 if err >= 0.0 else -1.0) *
                                       self.max_omega)
                elif self._forward_blocked():
                    self.publish_twist(0.0, 0.0, 0.0)
                else:
                    wz = max(-self.max_omega,
                             min(self.max_omega,
                                 self.hold_yaw_gain *
                                 wrap_pi(home_dir - self.odom_yaw)))
                    self.publish_twist(self.patrol_speed, 0.0, wz)

        elif self.state == MOVE_FORWARD:
            dist = math.hypot(self.odom_pos[0] - self.p0[0],
                              self.odom_pos[1] - self.p0[1])
            if dist >= self.forward_distance:
                self.publish_twist(0.0, 0.0, 0.0)
                rospy.loginfo("forward %.2f m reached (odom), dwell point "
                              "(%.3f, %.3f)", dist,
                              self.odom_pos[0], self.odom_pos[1])
                self.set_state(COMPUTE_PATROL_ANGLES)
            else:
                yaw_err = wrap_pi(self.yaw0 - self.odom_yaw)
                wz = max(-self.max_omega,
                         min(self.max_omega, self.hold_yaw_gain * yaw_err))
                self.publish_twist(self.patrol_speed, 0.0, wz)

        elif self.state == COMPUTE_PATROL_ANGLES:
            self.theta1 = math.atan2(self.door1[1] - self.odom_pos[1],
                                     self.door1[0] - self.odom_pos[0])
            self.theta2 = math.atan2(self.door2[1] - self.odom_pos[1],
                                     self.door2[0] - self.odom_pos[0])
            self.dir12 = 1.0 if wrap_pi(self.theta2 - self.theta1) >= 0.0 else -1.0
            rospy.loginfo("patrol angles: door1=%.3f rad door2=%.3f rad "
                          "sweep dir=%.0f", self.theta1, self.theta2, self.dir12)
            self.set_state(ALIGN_FIRST_DOOR)

        elif self.state == ALIGN_FIRST_DOOR:
            err = wrap_pi(self.theta1 - self.odom_yaw)
            if abs(err) <= self.angle_tolerance:
                self.publish_twist(0.0, 0.0, 0.0)
                self.next_state = SWEEP_TO_SECOND_DOOR
                self.set_state(DWELL)
            else:
                self.publish_twist(0.0, 0.0, (1.0 if err >= 0.0 else -1.0) * self.max_omega)

        elif self.state == SWEEP_TO_SECOND_DOOR:
            err = wrap_pi(self.theta2 - self.odom_yaw)
            if abs(err) <= self.angle_tolerance:
                self.publish_twist(0.0, 0.0, 0.0)
                self.next_state = SWEEP_TO_FIRST_DOOR
                self.set_state(DWELL)
            else:
                self.publish_twist(0.0, 0.0, self.dir12 * self.max_omega)

        elif self.state == SWEEP_TO_FIRST_DOOR:
            err = wrap_pi(self.theta1 - self.odom_yaw)
            if abs(err) <= self.angle_tolerance:
                self.publish_twist(0.0, 0.0, 0.0)
                self.next_state = SWEEP_TO_SECOND_DOOR
                self.set_state(DWELL)
            else:
                self.publish_twist(0.0, 0.0, -self.dir12 * self.max_omega)

        elif self.state == DWELL:
            self.publish_twist(0.0, 0.0, 0.0)
            if (rospy.Time.now() - self.state_enter_t).to_sec() >= self.dwell_time:
                self.set_state(self.next_state)

        elif self.state == CONTAINMENT_INIT:
            self.p0 = None
            self.next_state = None
            self.cont_yaw = self.odom_yaw
            rospy.loginfo("containment init: yaw locked at %.3f rad",
                          self.cont_yaw)
            self.set_state(CONTAINMENT_ACTIVE)

        elif self.state == CONTAINMENT_ACTIVE:
            self._containment_tick()

    def _containment_tick(self):
        """Omnidirectional tracking of PL/PR with yaw locked (wz = 0).

        Attraction toward the world-frame containment_goal, scan repulsion
        for collision avoidance, both transformed to the base frame with
        the yaw captured at containment init. Capture (M_CAPTURE_CONFIRMED)
        freezes the car. M_FINAL_ALIGN: in-place rotation (omega only,
        translation strictly zero) to face the align target published by
        the mission manager (the intruder's position).
        """
        if self.mission_state in ("M_CAPTURE_CONFIRMED", "M_SUCCESS"):
            self.publish_twist(0.0, 0.0, 0.0)
            return
        if self.mission_state == "M_FINAL_ALIGN":
            if self.cont_goal is None or self.odom_pos is None:
                self.publish_twist(0.0, 0.0, 0.0)
                return
            target = math.atan2(self.cont_goal[1] - self.odom_pos[1],
                                self.cont_goal[0] - self.odom_pos[0])
            err = wrap_pi(target - self.odom_yaw)
            if abs(err) <= self.align_tolerance:
                self.publish_twist(0.0, 0.0, 0.0)
            else:
                wz = self.align_max_omega if err >= 0.0 else \
                    -self.align_max_omega
                self.publish_twist(0.0, 0.0, wz)
            return
        if self.cont_goal is None or self.odom_pos is None:
            self.publish_twist(0.0, 0.0, 0.0)
            return
        dx = self.cont_goal[0] - self.odom_pos[0]
        dy = self.cont_goal[1] - self.odom_pos[1]
        d = math.hypot(dx, dy)
        if d < 1e-3:
            self.publish_twist(0.0, 0.0, 0.0)
            return
        mag = min(self.kp_track * d, self.cont_max_v)
        ux, uy = dx / d, dy / d
        c, s = math.cos(self.cont_yaw), math.sin(self.cont_yaw)
        vx = c * (mag * ux) + s * (mag * uy)
        vy = -s * (mag * ux) + c * (mag * uy)
        if self.scan is not None:
            rx = 0.0
            ry = 0.0
            for i, r in enumerate(self.scan.ranges):
                if 0.01 < r < self.repulse_range:
                    a = self.scan.angle_min + i * self.scan.angle_increment
                    wgt = (1.0 - r / self.repulse_range)
                    rx -= self.repulse_gain * wgt * math.cos(a)
                    ry -= self.repulse_gain * wgt * math.sin(a)
            vx += rx * self.cont_max_v
            vy += ry * self.cont_max_v
        vx = max(-self.cont_max_v, min(self.cont_max_v, vx))
        vy = max(-self.cont_max_v, min(self.cont_max_v, vy))
        self.publish_twist(vx, vy, 0.0)


if __name__ == "__main__":
    PatrolNode()
    rospy.spin()
