#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-car intruder mission node (入侵车, ns /car2).

State machine:
  E_IDLE -> E_SELECT_ENTRY -> E_APPROACH_ENTRY -> E_CROSS_ENTRY -> E_CROSSED
  containment signal -> E_EVASION_ACTIVE -> E_ESCAPED
  (later phases append: E_CAPTURED)

- E_SELECT_ENTRY: pick the entry gate (fixed or random with seed) from the
  /mission/intruder/* params.
- E_APPROACH_ENTRY: drive to a point just outside the gate through the
  existing nav_to_pose action server (move_base); nav_to_pose owns cmd_vel
  during this state. Goal yaw = gate inward normal so the car ends facing
  the doorway.
- E_CROSS_ENTRY: direct cmd_vel, translation through the door to an inside
  point; yaw held (crossing happens before the intrusion event, so a small
  heading-hold omega is still allowed here). Once the mission enters
  containment the heading hold is dropped (wz == 0 hard constraint).
- E_EVASION_ACTIVE: active escape inside the inner room. Every
  decision_period the three valid gates (all gates except the entry gate)
  are scored J = wd*Jd + wp*Jp + wo*Jo:
    Jd: normalized distance E->gate;
    Jp: defender proximity to the gate and to the escape path;
    Jo: fraction of scan rays toward the gate blocked before the gate
        (walls / virtual obstacles / defenders).
  argmin J selects the escape gate; switching requires
  J_new < J_current - exit_switch_margin sustained exit_switch_hold_time.
  Motion is omnidirectional translation only (wz == 0, yaw locked):
  attraction toward a point just beyond the gate plus scan repulsion,
  converted to the locked base frame.
- E_ESCAPED: reached the escape goal, zero velocity.

Topics (relative): sub shared_pose, sub scan_filtered, sub /mission/state,
sub /mission/entry_gate, sub /car0/shared_pose, /car1/shared_pose,
pub cmd_vel, pub intruder_state (latch), pub /mission/current_escape_gate
(latch). Services: start_intrusion (std_srvs/Trigger).
"""
import math
import random
import rospy
import actionlib
import tf.transformations as tfm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

E_IDLE = "E_IDLE"
E_SELECT_ENTRY = "E_SELECT_ENTRY"
E_APPROACH_ENTRY = "E_APPROACH_ENTRY"
E_CROSS_ENTRY = "E_CROSS_ENTRY"
E_CROSSED = "E_CROSSED"
E_EVASION_ACTIVE = "E_EVASION_ACTIVE"
E_ESCAPED = "E_ESCAPED"
E_CAPTURED = "E_CAPTURED"
E_RETURN_OUT = "E_RETURN_OUT"

GATES = ["UP", "DOWN", "LEFT", "RIGHT"]
MISSION_CONTAINMENT_PREFIX = "M_CONTAINMENT"
M_PATROL = "M_PATROL"
M_IDLE = "M_IDLE"

_ACT_SUCCEEDED = actionlib.GoalStatus.SUCCEEDED
_ACT_TERMINAL = (actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED,
                 actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.PREEMPTED)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist_pt_seg(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-9 else ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


class IntruderNode(object):
    def __init__(self):
        rospy.init_node("intruder_node")

        def p(key, default):
            return rospy.get_param("/mission/" + key, default)

        self.entry_mode = p("intruder/entry_mode", "fixed")
        self.entry_gate = p("intruder/entry_gate", "LEFT").upper()
        self.random_seed = p("intruder/random_seed", 42)
        # 持久 RNG：random 模式下每轮取门（seed>0 序列可复现，<=0 时间种子真随机）
        self._rng = (random.Random(self.random_seed) if self.random_seed > 0
                     else random.Random())
        self.prev_entry_gate = None
        self.escape_mode = p("intruder/escape_mode", "auto")
        self.escape_gate = p("intruder/escape_gate", "RIGHT").upper()
        self.auto_intrude = p("fsm/auto_intrude", True)
        self.wait_patrol_ready = p("fsm/wait_patrol_ready", True)
        self.start_delay = p("fsm/intruder_start_delay", 0.0)
        self.approach_offset = p("intruder/approach_offset", 0.9)
        self.inside_offset = p("intruder/inside_offset", 1.0)
        self.cross_speed = p("intruder/cross_speed", 0.35)
        self.max_vx = p("motion/intruder_max_vx", 0.50)
        self.max_vy = p("motion/intruder_max_vy", 0.50)
        self.kp_track = p("motion/kp_track", 1.6)
        self.yaw_hold_gain = p("motion/cross_yaw_hold_gain", 1.5)
        self.cross_arrive = 0.15
        self.decision_period = p("fsm/decision_period", 0.5)
        self.switch_margin = p("intruder/exit_switch_margin", 0.15)
        self.switch_hold = p("intruder/exit_switch_hold_time", 0.5)
        w = p("intruder/escape_weights", {})
        self.wd = float(w.get("distance", 1.0))
        self.wp = float(w.get("pursuer", 1.5))
        self.wo = float(w.get("obstacle", 1.0))
        self.distance_norm = p("intruder/distance_norm", 8.4)
        self.p_gate_decay = p("intruder/pursuer_gate_decay", 1.2)
        self.p_path_decay = p("intruder/pursuer_path_decay", 0.9)
        self.escape_goal_offset = p("intruder/escape_goal_offset", 0.6)
        self.escape_arrive = p("intruder/escape_arrive", 0.2)
        self.gate_block_margin = p("intruder/gate_block_margin", 0.1)
        self.repulse_range = p("motion/repulse_range", 0.55)
        self.repulse_gain = p("motion/repulse_gain", 0.7)
        self.gate_half_width = p("gate_half_width", 0.6)
        rate = p("fsm/control_rate", 20.0)
        # inner region bounds for reset return-out (multi-round)
        self.inner_x = (p("inner_region/x_min", -1e9),
                        p("inner_region/x_max", 1e9))
        self.inner_y = (p("inner_region/y_min", -1e9),
                        p("inner_region/y_max", 1e9))

        self.gate_pos = {}
        for k, v in p("gate_positions", {}).items():
            self.gate_pos[str(k).upper()] = (float(v[0]), float(v[1]))
        self.gate_n = {}
        for k, v in p("gate_normals", {}).items():
            self.gate_n[str(k).upper()] = (float(v[0]), float(v[1]))
        if len(self.gate_pos) != 4 or len(self.gate_n) != 4:
            rospy.logfatal("intruder_node: gate_positions/gate_normals params "
                           "incomplete (%d/%d), is mission_params.yaml loaded?",
                           len(self.gate_pos), len(self.gate_n))
            raise SystemExit(1)

        self.approach_wps = {}
        for k, v in p("intruder/approach_waypoints", {}).items():
            self.approach_wps[str(k).upper()] = [tuple(map(float, w)) for w in v]
        self.return_wps = {}
        for k, v in p("intruder/return_waypoints", {}).items():
            self.return_wps[str(k).upper()] = [tuple(map(float, w)) for w in v]
        self.spawn_pose = tuple(map(float, p("intruder/spawn_pose",
                                            [-6.5, 6.6])))
        self.spawn_return_dist = p("intruder/spawn_return_dist", 0.5)
        self.spawn_arrive_dist = p("intruder/spawn_arrive_dist", 0.35)
        self._goals = []
        self._goal_idx = 0
        self._fell_back = False
        self._ret_out_failed = False

        self.state = E_IDLE
        self.pos = None
        self.yaw = 0.0
        self.cross_target = None
        self.cross_yaw0 = 0.0
        self.mission_state = ""
        self.entry_gate_m = ""
        self.def_pos = {"car0": None, "car1": None}
        self.scan = None
        self.cur_gate = ""
        self.switch_streak = 0.0
        self.last_score_t = -1e9
        self.patrol_ps = {"car0": "", "car1": ""}
        self.auto_triggered = False
        self.auto_ready_t = None

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("intruder_state", String,
                                         queue_size=1, latch=True)
        self.escape_pub = rospy.Publisher("/mission/current_escape_gate",
                                          String, queue_size=1, latch=True)
        rospy.Subscriber("shared_pose", Odometry, self.pose_cb)
        rospy.Subscriber("/mission/state", String, self.mission_state_cb)
        rospy.Subscriber("/mission/entry_gate", String, self.entry_gate_cb)
        rospy.Subscriber("scan_filtered", LaserScan, self.scan_cb)
        for name in ("car0", "car1"):
            rospy.Subscriber("/%s/shared_pose" % name, Odometry,
                             self.def_cb, name)
            rospy.Subscriber("/%s/patrol_state" % name, String,
                             self.ps_cb, name)
        rospy.Service("start_intrusion", Trigger, self.srv_start)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.on_shutdown(self.publish_zero)

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.state_pub.publish(String(self.state))
        rospy.loginfo("intruder_node ready: entry_mode=%s entry_gate=%s "
                      "escape_mode=%s escape_gate=%s auto=%s wait_ready=%s "
                      "delay=%.2f | approach=%.2f inside=%.2f cross=%.2f m/s "
                      "| evasion: wd=%.1f wp=%.1f wo=%.1f dJ=%.2f Tsw=%.2fs",
                      self.entry_mode, self.entry_gate,
                      self.escape_mode, self.escape_gate,
                      self.auto_intrude, self.wait_patrol_ready,
                      self.start_delay, self.approach_offset,
                      self.inside_offset, self.cross_speed,
                      self.wd, self.wp, self.wo,
                      self.switch_margin, self.switch_hold)

    def pose_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tfm.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = yaw

    def mission_state_cb(self, msg):
        self.mission_state = msg.data

    def entry_gate_cb(self, msg):
        self.entry_gate_m = msg.data

    def def_cb(self, msg, name):
        self.def_pos[name] = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y)

    def ps_cb(self, msg, name):
        self.patrol_ps[name] = msg.data

    def _patrols_ready(self):
        ready = ("COMPUTE_PATROL_ANGLES", "ALIGN_FIRST_DOOR", "DWELL",
                 "SWEEP_TO_SECOND_DOOR", "SWEEP_TO_FIRST_DOOR")
        return all(self.patrol_ps[n] in ready for n in ("car0", "car1"))

    def _auto_start_check(self):
        if not self.auto_intrude or self.auto_triggered or \
                self.state != E_IDLE or self.mission_state != M_PATROL or \
                self.pos is None:
            return
        if self.wait_patrol_ready and not self._patrols_ready():
            return
        now = rospy.Time.now().to_sec()
        if self.auto_ready_t is None:
            self.auto_ready_t = now
            return
        if now - self.auto_ready_t < self.start_delay:
            return
        self.auto_triggered = True
        self.auto_ready_t = None
        rospy.loginfo("[INTRUDER] auto intrusion start")
        self.set_state(E_SELECT_ENTRY)

    def scan_cb(self, msg):
        self.scan = msg

    def set_state(self, s):
        self.state = s
        self.state_pub.publish(String(s))
        rospy.loginfo("[INTRUDER] state -> %s", s)

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

    def srv_start(self, _req):
        if self.state != E_IDLE:
            return TriggerResponse(True, "already running (state=%s)" % self.state)
        if self.pos is None:
            return TriggerResponse(False, "no shared_pose data yet")
        self.set_state(E_SELECT_ENTRY)
        return TriggerResponse(True, "intrusion mission started")

    def _select_entry(self):
        if self.entry_mode == "random":
            # 排除上一轮入口门，保证每轮演示换门
            candidates = [g for g in GATES if g != self.prev_entry_gate]
            if not candidates:
                candidates = list(GATES)
            gate = self._rng.choice(candidates)
        else:
            gate = self.entry_gate
        if gate not in self.gate_pos:
            rospy.logerr("[INTRUDER] unknown entry gate %s, fallback LEFT", gate)
            gate = "LEFT"
        self.entry_gate = gate
        self.prev_entry_gate = gate
        rospy.loginfo("[INTRUDER] selected entry gate = %s", gate)

    def _send_approach_goal(self):
        # 目标队列 = 走廊中转点 + 最终接近点（中转点保证全局规划不抄近路穿内区）
        wps = list(self.approach_wps.get(self.entry_gate, []))
        gx, gy = self.gate_pos[self.entry_gate]
        nx, ny = self.gate_n[self.entry_gate]
        wps.append((gx - nx * self.approach_offset,
                    gy - ny * self.approach_offset))
        self._goals = wps
        self._goal_idx = 0
        self._fell_back = False
        self._send_goal(wps[0])
        rospy.loginfo("[INTRUDER] approach via %d move_base goal(s): %s",
                      len(wps),
                      " ".join("(%.2f, %.2f)" % w for w in wps))

    def _send_goal(self, w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = w[0]
        goal.target_pose.pose.position.y = w[1]
        nx, ny = self.gate_n[self.entry_gate]
        goal.target_pose.pose.orientation = Quaternion(
            *tfm.quaternion_from_euler(0.0, 0.0, math.atan2(ny, nx)))
        self.ac.send_goal(goal)
        rospy.loginfo("[INTRUDER] move_base goal %d/%d (%.2f, %.2f)",
                      self._goal_idx + 1, len(self._goals), w[0], w[1])

    def _inside_inner(self, pos):
        return (self.inner_x[0] <= pos[0] <= self.inner_x[1] and
                self.inner_y[0] <= pos[1] <= self.inner_y[1])

    def _start_return_out(self):
        """Multi-round reset: E 回归规范位置。若在内区（上一轮捕获点）先经
        最近门驶出门外（保持既有行为），再沿外圈回出生点——保证下一轮接近
        路径从规范起点出发，防全局规划抄近路穿内区（穿错门/误触发入侵）。"""
        goals = []
        g = None
        if self.pos is not None:
            g = min(self.gate_pos, key=lambda k: math.hypot(
                self.gate_pos[k][0] - self.pos[0],
                self.gate_pos[k][1] - self.pos[1]))
        if g is not None and self._inside_inner(self.pos):
            gx, gy = self.gate_pos[g]
            nx, ny = self.gate_n[g]
            goals.append((gx - nx * self.approach_offset,
                          gy - ny * self.approach_offset))
            rospy.loginfo("[INTRUDER] reset: exiting via %s then to spawn", g)
        else:
            rospy.loginfo("[INTRUDER] reset: returning to spawn")
        # 回归段走廊中转点（按驶出/最近门选取）：防去出生点的全局规划
        # 抄近路穿内区（实测 DOWN/RIGHT 门外出发时直线距离短于外圈绕行）
        if g is not None:
            goals += list(self.return_wps.get(g, []))
        goals.append(self.spawn_pose)
        self._goals = goals
        self._goal_idx = 0
        self._fell_back = False
        self._send_goal(goals[0])
        self.set_state(E_RETURN_OUT)

    def _enter_evasion(self):
        self.switch_streak = 0.0
        self.last_score_t = -1e9
        self.cur_gate = ""
        if self.escape_mode == "fixed" and \
                self.escape_gate in self._valid_gates():
            self.cur_gate = self.escape_gate
            self.escape_pub.publish(String(self.escape_gate))
            rospy.loginfo("[INTRUDER] fixed escape gate = %s", self.escape_gate)
        self.set_state(E_EVASION_ACTIVE)
        rospy.loginfo("[INTRUDER] evasion active (yaw locked at %.3f rad)",
                      self.yaw)

    def _valid_gates(self):
        entry = self.entry_gate_m if self.entry_gate_m else self.entry_gate
        if self.escape_mode == "fixed" and self.escape_gate != entry:
            return [self.escape_gate]
        return [g for g in GATES if g != entry and g in self.gate_pos]

    def _gate_blocked_fraction(self, gate):
        if self.scan is None or self.pos is None:
            return 0.0
        gx, gy = self.gate_pos[gate]
        d = math.hypot(gx - self.pos[0], gy - self.pos[1])
        if d < 0.3:
            return 0.0
        a0 = wrap_pi(math.atan2(gy - self.pos[1], gx - self.pos[0]) - self.yaw)
        half = math.atan2(self.gate_half_width, d) + 0.08
        rng = self.scan.ranges
        n = len(rng)
        if n == 0:
            return 0.0
        cnt = 0
        blk = 0
        for i in range(n):
            a = self.scan.angle_min + i * self.scan.angle_increment
            if abs(wrap_pi(a - a0)) <= half:
                cnt += 1
                r = rng[i]
                if 0.01 < r < d - self.gate_block_margin:
                    blk += 1
        return float(blk) / cnt if cnt else 0.0

    def _score_gates(self):
        if self.pos is None:
            return {}
        scores = {}
        for g in self._valid_gates():
            gx, gy = self.gate_pos[g]
            d = math.hypot(gx - self.pos[0], gy - self.pos[1])
            jd = d / self.distance_norm
            jp = 0.0
            for name in ("car0", "car1"):
                dp = self.def_pos[name]
                if dp is None:
                    continue
                jp += math.exp(-math.hypot(dp[0] - gx, dp[1] - gy)
                               / self.p_gate_decay)
                jp += math.exp(-dist_pt_seg(dp, self.pos, (gx, gy))
                               / self.p_path_decay)
            jo = self._gate_blocked_fraction(g)
            scores[g] = self.wd * jd + self.wp * jp + self.wo * jo
        return scores

    def _score_and_select(self):
        if self.escape_mode == "fixed":
            return
        scores = self._score_gates()
        if not scores:
            return
        best = min(scores, key=scores.get)
        if self.cur_gate == "":
            rospy.loginfo("[INTRUDER] candidate exits:")
            for g in sorted(scores):
                rospy.loginfo("  %s: J=%.3f", g, scores[g])
            self.cur_gate = best
            self.escape_pub.publish(String(best))
            rospy.loginfo("[INTRUDER] selected escape gate = %s", best)
            return
        if best != self.cur_gate and \
                scores[best] < scores[self.cur_gate] - self.switch_margin:
            self.switch_streak += self.decision_period
            if self.switch_streak >= self.switch_hold:
                rospy.loginfo("[INTRUDER] switching escape gate %s -> %s "
                              "(J %.3f -> %.3f)", self.cur_gate, best,
                              scores[self.cur_gate], scores[best])
                self.cur_gate = best
                self.escape_pub.publish(String(best))
                self.switch_streak = 0.0
        else:
            self.switch_streak = 0.0

    def _evasion_tick(self):
        if self.pos is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self.last_score_t >= self.decision_period:
            self.last_score_t = now
            self._score_and_select()
        if self.cur_gate == "":
            self.publish_twist(0.0, 0.0, 0.0)
            return
        gx, gy = self.gate_pos[self.cur_gate]
        nx, ny = self.gate_n[self.cur_gate]
        # gate normal points inward; escape goal is just OUTSIDE the gate
        tx = gx - nx * self.escape_goal_offset
        ty = gy - ny * self.escape_goal_offset
        dx = tx - self.pos[0]
        dy = ty - self.pos[1]
        d = math.hypot(dx, dy)
        if d < self.escape_arrive:
            self.publish_twist(0.0, 0.0, 0.0)
            rospy.loginfo("[INTRUDER] escaped via %s (%.2f, %.2f)",
                          self.cur_gate, self.pos[0], self.pos[1])
            self.set_state(E_ESCAPED)
            return
        mag = min(self.kp_track * d, self.max_vx)
        ux, uy = dx / d, dy / d
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx = c * (mag * ux) + s * (mag * uy)
        vy = -s * (mag * ux) + c * (mag * uy)
        if self.scan is not None:
            rng = self.scan.ranges
            rx = 0.0
            ry = 0.0
            for i, r in enumerate(rng):
                if 0.01 < r < self.repulse_range:
                    a = self.scan.angle_min + i * self.scan.angle_increment
                    wgt = (1.0 - r / self.repulse_range)
                    rx -= self.repulse_gain * wgt * math.cos(a)
                    ry -= self.repulse_gain * wgt * math.sin(a)
            vx += rx * self.max_vx
            vy += ry * self.max_vy
        vx = max(-self.max_vx, min(self.max_vx, vx))
        vy = max(-self.max_vy, min(self.max_vy, vy))
        self.publish_twist(vx, vy, 0.0)

    def tick(self, _ev):
        if self.mission_state.startswith(MISSION_CONTAINMENT_PREFIX) and \
                self.state in (E_APPROACH_ENTRY, E_CROSS_ENTRY, E_CROSSED):
            if self.state == E_APPROACH_ENTRY:
                self.ac.cancel_goal()
            if self.entry_gate_m or self.entry_gate:
                self._enter_evasion()
        if self.mission_state == "M_IDLE" and \
                self.state in (E_APPROACH_ENTRY, E_CROSS_ENTRY,
                               E_EVASION_ACTIVE, E_ESCAPED, E_CROSSED,
                               E_CAPTURED):
            self.ac.cancel_goal()
            self.publish_twist(0.0, 0.0, 0.0)
            self.escape_pub.publish(String(""))
            self.set_state(E_IDLE)
        if self.mission_state == "M_IDLE":
            self.auto_triggered = False
            self.auto_ready_t = None
            if self.state == E_IDLE and self.pos is not None and \
                    not self._ret_out_failed and \
                    (self._inside_inner(self.pos) or
                     math.hypot(self.pos[0] - self.spawn_pose[0],
                                self.pos[1] - self.spawn_pose[1]) >
                     self.spawn_return_dist):
                self._start_return_out()
        else:
            self._ret_out_failed = False
        self._auto_start_check()
        if self.mission_state in ("M_CAPTURE_CONFIRMED", "M_FINAL_ALIGN",
                                  "M_SUCCESS") and \
                self.state not in (E_IDLE, E_RETURN_OUT, E_CAPTURED):
            self.publish_twist(0.0, 0.0, 0.0)
            rospy.loginfo("[INTRUDER] captured, holding stop")
            self.set_state(E_CAPTURED)
            return
        if self.mission_state in ("M_FAILED_ESCAPE", "M_INVALID_ESCAPE") and \
                self.state == E_EVASION_ACTIVE:
            self.publish_twist(0.0, 0.0, 0.0)
            rospy.loginfo("[INTRUDER] mission %s, stopped outside",
                          self.mission_state)
            self.set_state(E_ESCAPED)
            return
        if self.state == E_IDLE:
            return
        if self.state == E_RETURN_OUT:
            if self.pos is not None and \
                    math.hypot(self.pos[0] - self.spawn_pose[0],
                               self.pos[1] - self.spawn_pose[1]) <= \
                    self.spawn_arrive_dist:
                self.ac.cancel_goal()
                rospy.loginfo("[INTRUDER] returned to spawn, ready for "
                              "next round")
                self.set_state(E_IDLE)
                return
            st = self.ac.get_state()
            if st == _ACT_SUCCEEDED:
                self._goal_idx += 1
                if self._goal_idx < len(self._goals):
                    self._send_goal(self._goals[self._goal_idx])
                else:
                    rospy.loginfo("[INTRUDER] returned to spawn, ready for "
                                  "next round")
                    self.set_state(E_IDLE)
            elif st in _ACT_TERMINAL:
                if self._goal_idx < len(self._goals) - 1 and not self._fell_back:
                    self._fell_back = True
                    self._goal_idx = len(self._goals) - 1
                    rospy.logwarn("[INTRUDER] exit goal failed -> direct "
                                  "spawn return")
                    self._send_goal(self._goals[-1])
                else:
                    self._ret_out_failed = True
                    rospy.logwarn("[INTRUDER] return-out failed (action state "
                                  "%d) -> E_IDLE", st)
                    self.set_state(E_IDLE)
            return
        if self.pos is None:
            return

        if self.state == E_SELECT_ENTRY:
            self._select_entry()
            self._send_approach_goal()
            self.set_state(E_APPROACH_ENTRY)

        elif self.state == E_APPROACH_ENTRY:
            st = self.ac.get_state()
            if st == _ACT_SUCCEEDED:
                self._goal_idx += 1
                if self._goal_idx < len(self._goals):
                    self._send_goal(self._goals[self._goal_idx])
                else:
                    rospy.loginfo("[INTRUDER] approach done at (%.2f, %.2f)",
                                  self.pos[0], self.pos[1])
                    self.cross_yaw0 = self.yaw
                    gx, gy = self.gate_pos[self.entry_gate]
                    nx, ny = self.gate_n[self.entry_gate]
                    self.cross_target = (gx + nx * self.inside_offset,
                                         gy + ny * self.inside_offset)
                    self.set_state(E_CROSS_ENTRY)
            elif st in _ACT_TERMINAL:
                # 中转点失败则直接尝试最终接近点（旧行为，最坏退回抄近路）
                if self._goal_idx < len(self._goals) - 1 and not self._fell_back:
                    self._fell_back = True
                    self._goal_idx = len(self._goals) - 1
                    rospy.logwarn("[INTRUDER] waypoint %d failed -> direct "
                                  "approach", self._goal_idx)
                    self._send_goal(self._goals[-1])
                else:
                    rospy.logwarn("[INTRUDER] approach failed (action state "
                                  "%d) -> E_IDLE", st)
                    self.set_state(E_IDLE)

        elif self.state == E_CROSS_ENTRY:
            dx = self.cross_target[0] - self.pos[0]
            dy = self.cross_target[1] - self.pos[1]
            dist = math.hypot(dx, dy)
            if dist < self.cross_arrive:
                self.publish_twist(0.0, 0.0, 0.0)
                rospy.loginfo("[INTRUDER] crossed into inner room (%.2f, %.2f)",
                              self.pos[0], self.pos[1])
                self.set_state(E_CROSSED)
                return
            mag = min(self.kp_track * dist, self.cross_speed)
            wx = mag * dx / dist
            wy = mag * dy / dist
            c = math.cos(self.cross_yaw0)
            s = math.sin(self.cross_yaw0)
            vx = c * wx + s * wy
            vy = -s * wx + c * wy
            vx = max(-self.max_vx, min(self.max_vx, vx))
            vy = max(-self.max_vy, min(self.max_vy, vy))
            # heading hold is only allowed before intrusion; once the mission
            # enters containment the yaw-lock constraint (wz == 0) is hard
            if self.mission_state.startswith(MISSION_CONTAINMENT_PREFIX):
                wz = 0.0
            else:
                wz = max(-0.3, min(0.3,
                         self.yaw_hold_gain * wrap_pi(self.cross_yaw0 - self.yaw)))
            self.publish_twist(vx, vy, wz)

        elif self.state == E_EVASION_ACTIVE:
            self._evasion_tick()

        elif self.state == E_CROSSED:
            self.publish_twist(0.0, 0.0, 0.0)

        elif self.state == E_ESCAPED:
            self.publish_twist(0.0, 0.0, 0.0)


if __name__ == "__main__":
    IntruderNode()
    rospy.spin()
