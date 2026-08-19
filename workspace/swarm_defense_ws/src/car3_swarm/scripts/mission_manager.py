#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission Manager (任务层状态管理节点).

Maintains the task-level state machine on top of the per-car FSMs:

  M_IDLE -> (start) -> M_PATROL
         -> (INTRUSION_EVENT) -> M_CONTAINMENT_INIT -> M_CONTAINMENT_ACTIVE
         -> (capture) -> M_CAPTURE_CONFIRMED -> M_FINAL_ALIGN -> M_SUCCESS
         -> (escape) -> M_FAILED_ESCAPE / M_INVALID_ESCAPE

This node never commands chassis motion directly; it publishes mission state,
target points and judgments, and the car nodes execute.

Phase 2: intrusion detection (outside->inside + debounce + one-shot latch),
entry_gate = nearest gate to the crossing point, /mission/start + /mission/reset.

Phase 6: containment planning (never motion):
- roles assigned once at CONTAINMENT_INIT (Car A = car1, Car B = car0):
  C1 = d(A,PL)+d(B,PR), C2 = d(A,PR)+d(B,PL); argmin locks the assignment.
- escape axis e = (p_G - p_E)/|p_G - p_E| toward the intruder's current
  escape gate (from /mission/current_escape_gate), left normal n = (-e_y, e_x)
- PL = p_E + d_f*e + d_s*n, PR = p_E + d_f*e - d_s*n, clamped to the inner
  safe region (wall_safe_margin inset)
- PL/PR re-published every tick to /carN/containment_goal (world frame)

Phase 8: dual termination judgment (also never motion):
- capture: d(A,E) <= R_c and d(B,E) <= R_c and z_A*z_B < 0 (z = e x (p-p_E),
  defenders on opposite sides of the escape axis) held continuously for
  T_hold -> M_CAPTURE_CONFIRMED, /mission/result = CAPTURE
- escape: E inside -> outside the inner region for escape_debounce ->
  nearest gate at the crossing point; gate == entry_gate -> M_INVALID_ESCAPE,
  otherwise M_FAILED_ESCAPE; /mission/result = ESCAPE / INVALID_ESCAPE

Phase 9: final align (task-level judgment, still never motion):
- after M_CAPTURE_CONFIRMED holds final_align_delay with all cars stopped,
  then M_FINAL_ALIGN: E's position is published as the align target on the
  containment_goal topics; the defender nodes rotate in place (omega only)
  to face E at final_align_max_omega.
- SUCCESS when both |yaw_i - atan2(E - p_i)| < final_align_tolerance, or on
  final_align_timeout (logged as a warning).
"""
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point
import tf.transformations as tfm

M_IDLE = "M_IDLE"
M_PATROL = "M_PATROL"
M_CONTAINMENT_INIT = "M_CONTAINMENT_INIT"
M_CONTAINMENT_ACTIVE = "M_CONTAINMENT_ACTIVE"
M_CAPTURE_CONFIRMED = "M_CAPTURE_CONFIRMED"
M_FINAL_ALIGN = "M_FINAL_ALIGN"
M_SUCCESS = "M_SUCCESS"
M_FAILED_ESCAPE = "M_FAILED_ESCAPE"
M_INVALID_ESCAPE = "M_INVALID_ESCAPE"

CONTAINMENT_STATES = (M_CONTAINMENT_INIT, M_CONTAINMENT_ACTIVE)
GATES = ["UP", "DOWN", "LEFT", "RIGHT"]


class MissionManager(object):
    def __init__(self):
        rospy.init_node("mission_manager")

        def p(key, default):
            return rospy.get_param("/mission/" + key, default)

        self.x_min = p("inner_region/x_min", -3.091)
        self.x_max = p("inner_region/x_max", 2.859)
        self.y_min = p("inner_region/y_min", -2.984)
        self.y_max = p("inner_region/y_max", 2.966)
        self.debounce = p("fsm/intrusion_debounce", 0.25)
        self.escape_debounce = p("fsm/escape_debounce", 0.25)
        rate = p("fsm/control_rate", 20.0)
        self.dt = 1.0 / rate

        self.d_f = p("containment/forward_offset", 0.35)
        self.d_s = p("containment/lateral_offset", 0.35)
        self.wall_margin = p("containment/wall_safe_margin", 0.25)
        self.capture_radius = p("containment/capture_radius", 0.55)
        self.capture_hold = p("containment/capture_hold_time", 0.4)
        self.align_tolerance = p("motion/final_align_tolerance", 0.05)
        self.align_timeout = p("fsm/final_align_timeout", 15.0)
        self.align_delay = p("fsm/final_align_delay", 1.0)

        self.gate_pos = {}
        for k, v in p("gate_positions", {}).items():
            self.gate_pos[str(k).upper()] = (float(v[0]), float(v[1]))
        if len(self.gate_pos) != 4:
            rospy.logfatal("mission_manager: gate_positions param incomplete, "
                           "is mission_params.yaml loaded?")
            raise SystemExit(1)

        self.state = M_IDLE
        self.e_pos = None
        self.a_pos = None
        self.b_pos = None
        self.a_yaw = 0.0
        self.b_yaw = 0.0
        self.intrusion_latched = False
        self.inside_streak = 0.0
        self.last_inside = False
        # multi-round: intrusion requires E observed outside first (outside->
        # inside transition). After /mission/reset E may still be physically
        # inside from the previous capture; that must not re-trigger intrusion.
        self.e_seen_outside = False
        # E 复位回归（E_RETURN_OUT）期间的穿内区不计入侵（回归段中转点之外的保险）
        self.e_fsm_state = ""
        self.entry_gate = ""
        self.cur_gate = ""
        self.role_a = ""
        self.pl = None
        self.pr = None
        self.last_cont_log = 0.0
        self.capture_streak = 0.0
        self.outside_streak = 0.0
        self.last_inside_pos = None
        self.confirmed_t0 = None
        self.align_t0 = None

        self.state_pub = rospy.Publisher("/mission/state", String,
                                         queue_size=1, latch=True)
        self.event_pub = rospy.Publisher("/mission/intrusion_event", String,
                                         queue_size=1, latch=True)
        self.entry_pub = rospy.Publisher("/mission/entry_gate", String,
                                         queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/mission/result", String,
                                          queue_size=1, latch=True)
        self.goal_a_pub = rospy.Publisher("/car1/containment_goal", Point,
                                          queue_size=1)
        self.goal_b_pub = rospy.Publisher("/car0/containment_goal", Point,
                                          queue_size=1)

        rospy.Subscriber("/car2/shared_pose", Odometry, self.e_pose_cb)
        rospy.Subscriber("/car1/shared_pose", Odometry, self.a_pose_cb)
        rospy.Subscriber("/car0/shared_pose", Odometry, self.b_pose_cb)
        rospy.Subscriber("/mission/current_escape_gate", String,
                         self.cur_gate_cb)
        rospy.Subscriber("/car2/intruder_state", String, self.e_fsm_cb)
        rospy.Service("/mission/start", Trigger, self.srv_start)
        rospy.Service("/mission/reset", Trigger, self.srv_reset)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)

        self.state_pub.publish(String(self.state))
        rospy.loginfo("mission_manager ready: inner x[%.2f, %.2f] y[%.2f, %.2f] "
                      "debounce %.2fs | V: d_f=%.2f d_s=%.2f wall=%.2f",
                      self.x_min, self.x_max, self.y_min, self.y_max,
                      self.debounce, self.d_f, self.d_s, self.wall_margin)

    def e_pose_cb(self, msg):
        self.e_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def a_pose_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tfm.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.a_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.a_yaw = yaw

    def b_pose_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tfm.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.b_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.b_yaw = yaw

    def cur_gate_cb(self, msg):
        self.cur_gate = msg.data

    def e_fsm_cb(self, msg):
        self.e_fsm_state = msg.data

    def set_state(self, s):
        self.state = s
        self.state_pub.publish(String(s))
        rospy.loginfo("[MISSION] state -> %s", s)

    @staticmethod
    def _wrap_pi(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def inside(self, pos):
        return (self.x_min < pos[0] < self.x_max and
                self.y_min < pos[1] < self.y_max)

    def nearest_gate(self, pos):
        best, best_d = None, 1e9
        for name, (gx, gy) in self.gate_pos.items():
            d = math.hypot(pos[0] - gx, pos[1] - gy)
            if d < best_d:
                best, best_d = name, d
        return best, best_d

    def valid_gates(self):
        entry = self.entry_gate if self.entry_gate else ""
        return [g for g in GATES if g != entry]

    def _axis_gate(self):
        """Current escape gate for the containment axis."""
        if self.cur_gate in self.valid_gates():
            return self.cur_gate
        if self.e_pos is not None:
            best, best_d = None, 1e9
            for g in self.valid_gates():
                gx, gy = self.gate_pos[g]
                d = math.hypot(self.e_pos[0] - gx, self.e_pos[1] - gy)
                if d < best_d:
                    best, best_d = g, d
            if best:
                return best
        return "UP"

    def _clamp_safe(self, pt):
        x = max(self.x_min + self.wall_margin,
                min(self.x_max - self.wall_margin, pt[0]))
        y = max(self.y_min + self.wall_margin,
                min(self.y_max - self.wall_margin, pt[1]))
        return (x, y)

    def _compute_v(self):
        """Compute PL/PR from the current escape axis; returns (pl, pr)."""
        if self.e_pos is None:
            return self.pl, self.pr
        gate = self._axis_gate()
        gx, gy = self.gate_pos[gate]
        ex = gx - self.e_pos[0]
        ey = gy - self.e_pos[1]
        d = math.hypot(ex, ey)
        if d < 1e-6:
            return self.pl, self.pr
        ex, ey = ex / d, ey / d
        nx, ny = -ey, ex
        pl = (self.e_pos[0] + self.d_f * ex + self.d_s * nx,
              self.e_pos[1] + self.d_f * ey + self.d_s * ny)
        pr = (self.e_pos[0] + self.d_f * ex - self.d_s * nx,
              self.e_pos[1] + self.d_f * ey - self.d_s * ny)
        return self._clamp_safe(pl), self._clamp_safe(pr)

    def _assign_roles(self):
        pl, pr = self._compute_v()
        self.pl, self.pr = pl, pr
        if self.a_pos is None or self.b_pos is None:
            self.role_a = "LEFT"
            rospy.logwarn("[CONTAINMENT] defender poses missing, role default "
                          "Car A = LEFT")
            return
        c1 = (math.hypot(self.a_pos[0] - pl[0], self.a_pos[1] - pl[1]) +
              math.hypot(self.b_pos[0] - pr[0], self.b_pos[1] - pr[1]))
        c2 = (math.hypot(self.a_pos[0] - pr[0], self.a_pos[1] - pr[1]) +
              math.hypot(self.b_pos[0] - pl[0], self.b_pos[1] - pl[1]))
        self.role_a = "LEFT" if c1 < c2 else "RIGHT"
        role_b = "RIGHT" if self.role_a == "LEFT" else "LEFT"
        rospy.loginfo("[CONTAINMENT] role: Car A = %s, Car B = %s "
                      "(C1=%.2f C2=%.2f)", self.role_a, role_b, c1, c2)
        self._log_v()

    def _log_v(self):
        if self.pl is None:
            return
        rospy.loginfo("[CONTAINMENT] PL = (%.2f, %.2f)",
                      self.pl[0], self.pl[1])
        rospy.loginfo("[CONTAINMENT] PR = (%.2f, %.2f)",
                      self.pr[0], self.pr[1])

    def _update_containment(self):
        pl, pr = self._compute_v()
        if pl is None:
            return
        gate_changed = False
        old_gate = self.cur_gate
        self.pl, self.pr = pl, pr
        a_goal = pl if self.role_a == "LEFT" else pr
        b_goal = pr if self.role_a == "LEFT" else pl
        self.goal_a_pub.publish(Point(a_goal[0], a_goal[1], 0.0))
        self.goal_b_pub.publish(Point(b_goal[0], b_goal[1], 0.0))
        now = rospy.Time.now().to_sec()
        if gate_changed or now - self.last_cont_log >= 2.0:
            self.last_cont_log = now
            self._log_v()

    def srv_start(self, _req):
        if self.state != M_IDLE:
            return TriggerResponse(True, "already running (state=%s)" % self.state)
        if self.e_pos is None:
            return TriggerResponse(False, "no shared_pose data yet")
        self.set_state(M_PATROL)
        rospy.loginfo("[MISSION] PATROL")
        # fire the defenders' patrol services (best effort: the mission
        # launch may not include them yet in early phases)
        for car in ("car1", "car0"):
            try:
                rospy.wait_for_service("/%s/start_patrol" % car, timeout=3.0)
                r = rospy.ServiceProxy("/%s/start_patrol" % car, Trigger)()
                rospy.loginfo("[MISSION] %s patrol: %s", car, r.message)
            except (rospy.ROSException, rospy.ServiceException) as e:
                rospy.logwarn("[MISSION] %s start_patrol unavailable: %s", car, e)
        return TriggerResponse(True, "mission started (PATROL)")

    def srv_reset(self, _req):
        self.intrusion_latched = False
        self.inside_streak = 0.0
        self.last_inside = False
        self.e_seen_outside = False
        self.entry_gate = ""
        self.cur_gate = ""
        self.role_a = ""
        self.pl = None
        self.pr = None
        self.capture_streak = 0.0
        self.outside_streak = 0.0
        self.last_inside_pos = None
        self.confirmed_t0 = None
        self.align_t0 = None
        self.event_pub.publish(String(""))
        self.entry_pub.publish(String(""))
        self.result_pub.publish(String(""))
        self.set_state(M_IDLE)
        rospy.loginfo("[MISSION] RESET done, back to M_IDLE")
        return TriggerResponse(True, "mission reset")

    def tick(self, _ev):
        if self.e_pos is None:
            return

        inside_now = self.inside(self.e_pos)
        if inside_now and not self.last_inside:
            self.inside_streak = 0.0
        self.last_inside = inside_now

        if self.state in CONTAINMENT_STATES:
            self._update_containment()
            self._check_escape(inside_now)

        if self.state == M_CONTAINMENT_ACTIVE:
            self._check_capture()

        if self.state == M_CAPTURE_CONFIRMED:
            if self.confirmed_t0 is None:
                self.confirmed_t0 = rospy.Time.now().to_sec()
            elif rospy.Time.now().to_sec() - self.confirmed_t0 >= \
                    self.align_delay:
                rospy.loginfo("[MISSION] final align: defenders rotate to "
                              "face intruder")
                self.align_t0 = rospy.Time.now().to_sec()
                self.set_state(M_FINAL_ALIGN)

        if self.state == M_FINAL_ALIGN:
            self._update_final_align()

        if self.state == M_PATROL and not self.intrusion_latched:
            if not inside_now:
                self.e_seen_outside = True
                self.inside_streak = 0.0
            elif self.e_seen_outside and self.e_fsm_state != "E_RETURN_OUT":
                self.inside_streak += self.dt
                if self.inside_streak >= self.debounce:
                    self._on_intrusion()

    def _on_intrusion(self):
        self.intrusion_latched = True
        gate, dist = self.nearest_gate(self.e_pos)
        if dist > 1.2:
            rospy.logwarn("[MISSION] crossing point (%.2f, %.2f) is %.2fm "
                          "from nearest gate %s", self.e_pos[0], self.e_pos[1],
                          dist, gate)
        self.entry_gate = gate
        self.entry_pub.publish(String(gate))
        self.event_pub.publish(String(gate))
        rospy.loginfo("[MISSION] intrusion detected from %s", gate)
        rospy.loginfo("[MISSION] patrol interrupted")
        self.set_state(M_CONTAINMENT_INIT)
        self._assign_roles()
        self.set_state(M_CONTAINMENT_ACTIVE)

    def _check_escape(self, inside_now):
        if inside_now:
            self.last_inside_pos = self.e_pos
            self.outside_streak = 0.0
            return
        self.outside_streak += self.dt
        if self.outside_streak >= self.escape_debounce:
            self._on_escape()

    def _on_escape(self):
        pos = self.last_inside_pos if self.last_inside_pos is not None \
            else self.e_pos
        gate, dist = self.nearest_gate(pos)
        if gate == self.entry_gate:
            self.result_pub.publish(String("INVALID_ESCAPE"))
            rospy.logwarn("[MISSION] intruder left via entry gate %s -> "
                          "INVALID_ESCAPE (experiment anomaly)", gate)
            self.set_state(M_INVALID_ESCAPE)
        else:
            self.result_pub.publish(String("ESCAPE"))
            rospy.logwarn("[MISSION] intruder escaped via %s (crossing %.2fm "
                          "from gate) -> FAILED_ESCAPE", gate, dist)
            rospy.logwarn("[MISSION] defense failed, containment aborted")
            self.set_state(M_FAILED_ESCAPE)

    def _check_capture(self):
        if self.a_pos is None or self.b_pos is None:
            self.capture_streak = 0.0
            return
        da = math.hypot(self.a_pos[0] - self.e_pos[0],
                        self.a_pos[1] - self.e_pos[1])
        db = math.hypot(self.b_pos[0] - self.e_pos[0],
                        self.b_pos[1] - self.e_pos[1])
        if da > self.capture_radius or db > self.capture_radius:
            self.capture_streak = 0.0
            return
        gate = self._axis_gate()
        gx, gy = self.gate_pos[gate]
        ex = gx - self.e_pos[0]
        ey = gy - self.e_pos[1]
        d = math.hypot(ex, ey)
        if d < 1e-6:
            self.capture_streak = 0.0
            return
        ex, ey = ex / d, ey / d
        za = ex * (self.a_pos[1] - self.e_pos[1]) - \
            ey * (self.a_pos[0] - self.e_pos[0])
        zb = ex * (self.b_pos[1] - self.e_pos[1]) - \
            ey * (self.b_pos[0] - self.e_pos[0])
        if za * zb < 0.0:
            self.capture_streak += self.dt
            if self.capture_streak >= self.capture_hold:
                self._on_capture(da, db, za, zb, gate)
        else:
            self.capture_streak = 0.0

    def _on_capture(self, da, db, za, zb, gate):
        self.result_pub.publish(String("CAPTURE"))
        rospy.loginfo("[MISSION] capture confirmed: dA=%.2f dB=%.2f "
                      "zA=%.2f zB=%.2f (axis %s, hold %.2fs)",
                      da, db, za, zb, gate, self.capture_hold)
        rospy.loginfo("[MISSION] all cars stop")
        self.confirmed_t0 = rospy.Time.now().to_sec()
        self.set_state(M_CAPTURE_CONFIRMED)

    def _update_final_align(self):
        """Defenders rotate in place to face E (task-level judgment only).

        Publishes E's position as the align target on the containment_goal
        topics (the car nodes execute the rotation); judges completion when
        both |yaw - atan2(E - A/B)| < tolerance, or on timeout.
        """
        if self.e_pos is None:
            return
        self.goal_a_pub.publish(Point(self.e_pos[0], self.e_pos[1], 0.0))
        self.goal_b_pub.publish(Point(self.e_pos[0], self.e_pos[1], 0.0))
        if self.a_pos is None or self.b_pos is None or self.align_t0 is None:
            return
        err_a = abs(self._wrap_pi(
            math.atan2(self.e_pos[1] - self.a_pos[1],
                       self.e_pos[0] - self.a_pos[0]) - self.a_yaw))
        err_b = abs(self._wrap_pi(
            math.atan2(self.e_pos[1] - self.b_pos[1],
                       self.e_pos[0] - self.b_pos[0]) - self.b_yaw))
        rospy.loginfo_throttle(1.0, "[ALIGN] errA=%.3f errB=%.3f rad "
                               "(tol %.3f)", err_a, err_b,
                               self.align_tolerance)
        elapsed = rospy.Time.now().to_sec() - self.align_t0
        if err_a <= self.align_tolerance and err_b <= self.align_tolerance:
            rospy.loginfo("[MISSION] final align done (%.2fs), SUCCESS",
                          elapsed)
            self.set_state(M_SUCCESS)
        elif elapsed >= self.align_timeout:
            rospy.logwarn("[MISSION] final align timeout (%.2fs), "
                          "SUCCESS anyway", elapsed)
            self.set_state(M_SUCCESS)


if __name__ == "__main__":
    MissionManager()
    rospy.spin()
