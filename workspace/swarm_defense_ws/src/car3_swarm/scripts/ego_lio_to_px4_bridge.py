#!/usr/bin/env python3
"""Convert an EGO/SWARM-LIO pose command into the PX4 local ENU frame.

EGO plans in the SWARM-LIO world frame (quadN/world), while MAVROS local
position is the PX4 local frame.  They are both ENU, but their origins and
heading can differ.  Passing an absolute EGO pose straight to
setpoint_raw/local therefore creates an artificial altitude/position jump.

The adapter applies the current relative rigid transform:

    p_px4_cmd = p_px4_now + R(px4_now) R(lio_now)^T
                              (p_lio_cmd - p_lio_now)

This makes a command equal to the current LIO pose become a hold at the
current PX4 pose, while preserving EGO's relative motion and yaw changes.
The adapter publishes only while a fresh EGO command and both pose inputs are
available; otherwise it holds the current PX4 pose.
"""

import math
import sys
import time

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float64


def q_norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_inv(q):
    x, y, z, w = q_norm(q)
    return (-x, -y, -z, w)


def q_rotate(q, v):
    return q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_inv(q))[:3]


def q_yaw(q):
    x, y, z, w = q_norm(q)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def q_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_q(pose):
    return q_norm((pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w))


class LioToPx4Bridge:
    def __init__(self, name):
        self.name = name
        self.lio_pose = None
        self.px4_pose = None
        self.command = None
        self.command_seen = False
        self.lio_wall = None
        self.px4_wall = None
        self.command_wall = None
        self.max_age = float(rospy.get_param("~max_pose_age", 1.0))
        self.max_command_age = float(rospy.get_param("~max_command_age", 0.8))
        self.max_xy = float(rospy.get_param("~max_relative_xy", 8.0))
        self.max_z = float(rospy.get_param("~max_relative_z", 2.0))
        # In the EKF integration EGO's odom_world and trajectory command are
        # both in PX4 local ENU.  The legacy lio mode is retained for older
        # launches, but must not be used together with PX4 odom remaps.
        self.command_frame = rospy.get_param("~command_frame", "lio")
        # The current EGO acceptance test is horizontal motion at a fixed
        # height.  SWARM-LIO-Z is not yet reliable in this simulation (it can
        # drift while PX4/baro remains stable), so do not let an LIO-Z drift
        # become a climb command.  Set hold_z=false only for a separately
        # validated vertical EGO experiment.
        self.hold_z = bool(rospy.get_param("~hold_z", True))
        self.hold_z_min = float(rospy.get_param("~hold_z_min", 0.5))
        self.hold_z_value = None
        self.hold_xy_value = None
        # The takeoff controller knows the requested PX4-local ENU height.
        # Use that value at EGO handover instead of the instantaneous PX4
        # estimate, which can still lag by 0.2--0.3 m while the vehicle is
        # already airborne.  Falling back to the measured value keeps the
        # adapter usable outside the takeoff script.
        self.takeoff_height = None
        # Do not let a pre-planned EGO trajectory steal the takeoff setpoint.
        # uav_offboard_takeoff.py publishes this handshake only after PX4 has
        # reached its requested takeoff altitude.
        self.wait_for_takeoff_signal = bool(
            rospy.get_param("~wait_for_takeoff_signal", False))
        self.ego_takeover = not self.wait_for_takeoff_signal
        # EGO's trajectory server can rotate the commanded yaw to point along
        # the direction of travel.  In this simulation a near-180 degree yaw
        # change can make the tilted/low-density Livox scan lose registration.
        # Keep the PX4 yaw captured at handover while validating translational
        # EGO control.  Set hold_yaw=false for a separately validated yaw test.
        self.hold_yaw = bool(rospy.get_param("~hold_yaw", False))
        self.handover_px4_yaw = None
        # LIO and PX4 local frames have different origins.  Capture one
        # rigid transform when EGO first takes control and keep it latched.
        # Recomputing the transform from the moving estimates on every
        # callback creates a feedback loop: an LIO drift is interpreted as a
        # new command, which moves the vehicle and causes more drift.
        self.handover_lio_position = None
        self.handover_px4_position = None
        self.handover_yaw_offset = None
        self.last_output = None
        # A moving Iris can briefly reach 20--25 degrees while the sparse
        # Mid360 registration is still valid.  Rejecting those samples froze
        # an otherwise healthy EGO command stream.  Roll/pitch are not used
        # for the LIO->PX4 translation (only the latched yaw offset is), so a
        # larger sanity bound is appropriate for this simulation.
        self.max_lio_tilt = float(rospy.get_param("~max_lio_tilt_deg", 35.0))
        self.max_lio_step = float(rospy.get_param("~max_lio_step", 0.80))
        self.max_lio_speed = float(rospy.get_param("~max_lio_speed", 5.0))
        self.lio_bad = False
        self.lio_valid_wall = None

    def _index(self):
        digits = ""
        for c in reversed(self.name):
            if not c.isdigit():
                break
            digits = c + digits
        if not digits:
            raise RuntimeError("vehicle name must end in a number: %s" % self.name)
        return int(digits)

    def attach_subscribers(self):
        # Import here so the module remains easy to inspect without ROS setup.
        from nav_msgs.msg import Odometry

        idx = self._index()
        self.lio_sub = None
        if self.command_frame != "px4_local":
            self.lio_sub = rospy.Subscriber(
                "/quad%d/lidar_slam/odom" % idx, Odometry,
                self._lio_cb, queue_size=1)
        self.px4_sub = rospy.Subscriber(
            "/%s/mavros/local_position/pose" % self.name, PoseStamped,
            self._px4_cb, queue_size=1)
        self.cmd_sub = rospy.Subscriber(
            "/xtdrone/%s/cmd_pose_lio" % self.name, Pose,
            self._cmd_cb, queue_size=1)
        self.takeover_sub = rospy.Subscriber(
            "/xtdrone/%s/ego_takeover" % self.name, Bool,
            self._takeover_cb, queue_size=1)
        self.takeoff_height_sub = rospy.Subscriber(
            "/xtdrone/%s/takeoff_height" % self.name, Float64,
            self._takeoff_height_cb, queue_size=1)
        self.pub = rospy.Publisher(
            "/xtdrone/%s/cmd_pose_enu" % self.name, Pose,
            queue_size=1)

    def _lio_cb(self, msg):
        candidate = msg.pose.pose
        now = time.monotonic()
        if not self._finite_pose(candidate):
            self.lio_bad = True
            self.lio_wall = now
            return
        r, pitch = self._roll_pitch_deg(candidate.orientation)
        bad = max(abs(r), abs(pitch)) > self.max_lio_tilt
        if self.lio_pose is not None and self.lio_valid_wall is not None:
            dt = max(now - self.lio_valid_wall, 1e-3)
            dx = candidate.position.x - self.lio_pose.position.x
            dy = candidate.position.y - self.lio_pose.position.y
            dz = candidate.position.z - self.lio_pose.position.z
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            if step > self.max_lio_step or step / dt > self.max_lio_speed:
                bad = True
        if bad:
            self.lio_bad = True
            self.lio_wall = now
            rospy.logwarn_throttle(
                1.0, "%s freezes EGO output: invalid LIO pose roll=%.1f pitch=%.1f",
                self.name, r, pitch)
            return
        if self.lio_bad:
            rospy.loginfo("%s LIO pose recovered; EGO output released", self.name)
        self.lio_bad = False
        self.lio_pose = candidate
        self.lio_wall = now
        self.lio_valid_wall = now

    def _px4_cb(self, msg):
        self.px4_pose = msg.pose
        self.px4_wall = time.monotonic()

    def _cmd_cb(self, msg):
        self.command = msg
        self.command_seen = True
        self.command_wall = time.monotonic()

    def _takeover_cb(self, msg):
        enabled = bool(msg.data)
        if enabled and not self.ego_takeover:
            rospy.loginfo("%s received EGO takeover handshake", self.name)
        if not enabled and self.ego_takeover:
            # A new takeoff run can reuse this long-lived adapter process.
            # Drop all state latched during the previous EGO handover so an
            # old height/origin cannot be reused for the next flight.
            self.command = None
            self.command_seen = False
            self.command_wall = None
            self.hold_z_value = None
            self.hold_xy_value = None
            self.handover_lio_position = None
            self.handover_px4_position = None
            self.handover_yaw_offset = None
            self.handover_px4_yaw = None
            self.last_output = None
            rospy.loginfo("%s reset EGO handover state", self.name)
        self.ego_takeover = enabled

    def _takeoff_height_cb(self, msg):
        value = float(msg.data)
        if math.isfinite(value) and value >= self.hold_z_min:
            self.takeoff_height = value

    @staticmethod
    def _finite_pose(p):
        values = (p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y,
                  p.orientation.z, p.orientation.w)
        return all(math.isfinite(v) for v in values)

    @staticmethod
    def _roll_pitch_deg(orientation):
        """Return roll and pitch in degrees for a ROS quaternion.

        SWARM-LIO publishes a body attitude in the world frame.  The guard
        only uses roll/pitch as a sanity check; yaw is intentionally ignored
        here because yaw changes are valid during flight.
        """
        x, y, z, w = q_norm((orientation.x, orientation.y,
                              orientation.z, orientation.w))
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sin_pitch)
        return math.degrees(roll), math.degrees(pitch)

    def converted_command(self):
        now = time.monotonic()
        if not self.ego_takeover:
            # Keep the takeoff publisher in control until the explicit
            # handover.  This is important when EGO has already received a
            # goal and is outputting a trajectory while the UAV is grounded.
            return None
        if self.px4_pose is None:
            return None
        if self.command_frame != "px4_local" and self.lio_pose is None:
            return None
        if self.command_frame != "px4_local" and self.lio_bad:
            # Freeze the last safe command while the estimator recovers.  Do
            # not extrapolate a bad LIO sample into a PX4 position target.
            return self.last_output
        if self.px4_wall is None:
            return None
        if (now - self.px4_wall > self.max_age or
                (self.command_frame != "px4_local" and
                 (self.lio_wall is None or now - self.lio_wall > self.max_age))):
            return None

        px4 = self.px4_pose
        cmd = self.command
        if cmd is None or self.command_wall is None:
            # EGO can legitimately be in WAIT_TARGET after the takeoff
            # script hands over control.  Returning None here stops the
            # XTDrone/PX4 position-setpoint stream and PX4 then leaves
            # OFFBOARD (or starts descending) even though the vehicle is
            # healthy.  Keep a valid hold stream until the first EGO
            # trajectory arrives.  The handshake is sent only after the
            # takeoff height has been reached, so this cannot overwrite the
            # ground takeoff target.
            if self.hold_z and self.hold_z_value is None:
                if px4.position.z < self.hold_z_min:
                    return None
                self.hold_z_value = (
                    self.takeoff_height
                    if self.takeoff_height is not None
                    else px4.position.z)
            # A hold must be a fixed setpoint.  Re-publishing the current
            # PX4 position on every loop merely follows drift and provides no
            # restoring error; with the old code a stationary aircraft could
            # slowly translate while EGO was in WAIT_TARGET.
            if self.hold_xy_value is None:
                self.hold_xy_value = (px4.position.x, px4.position.y)
            out = Pose()
            out.position.x = self.hold_xy_value[0]
            out.position.y = self.hold_xy_value[1]
            out.position.z = (
                self.hold_z_value if self.hold_z else px4.position.z)
            if self.hold_yaw:
                q_hold = q_from_yaw(q_yaw(pose_q(px4)))
            else:
                q_hold = pose_q(px4)
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_hold
            self.last_output = out
            return out
        if now - self.command_wall > self.max_command_age:
            # A missing/stale command is an explicit hold.  Prefer the last
            # valid converted setpoint so a stale LIO estimate cannot move
            # the aircraft through a newly recomputed transform.
            if self.last_output is not None:
                return self.last_output
            return None

        if self.hold_z and self.hold_z_value is None:
            # Capture the height only once, when EGO actually takes control.
            # Refuse a handover from a landed/invalid state; otherwise the
            # adapter could lock a ground height and make an airborne vehicle
            # descend on the first EGO message.
            if px4.position.z < self.hold_z_min:
                rospy.logwarn_throttle(
                    1.0, "%s refuses EGO handover at PX4 z=%.2f m (minimum %.2f m)",
                    self.name, px4.position.z, self.hold_z_min)
                return None
            if self.takeoff_height is not None:
                self.hold_z_value = self.takeoff_height
                rospy.loginfo(
                    "%s uses requested takeoff/EGO height %.3f m (PX4 at handover %.3f m)",
                    self.name, self.hold_z_value, px4.position.z)
            else:
                self.hold_z_value = px4.position.z
                rospy.loginfo("%s captured measured EGO height %.3f m", self.name, self.hold_z_value)
        if not self._finite_pose(cmd):
            return None

        if self.command_frame == "px4_local":
            # The trajectory was generated from PX4 EKF local odom and the
            # GridMap cloud was transformed into the same map frame.  Pass
            # the position through directly; applying the old handover
            # LIO->PX4 delta here would transform it twice.
            out = Pose()
            out.position.x = cmd.position.x
            out.position.y = cmd.position.y
            out.position.z = (self.hold_z_value if self.hold_z
                              else cmd.position.z)
            if self.hold_yaw:
                if self.handover_px4_yaw is None:
                    self.handover_px4_yaw = q_yaw(pose_q(px4))
                q_out = q_from_yaw(self.handover_px4_yaw)
            else:
                q_out = pose_q(cmd)
            q_out = q_norm(q_out)
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out
            self.last_output = out
            return out

        q_cmd = pose_q(cmd)
        lio = self.lio_pose
        if self.handover_lio_position is None:
            # Latch the transform once, at the exact EGO handover.  Only yaw
            # is used: roll/pitch from a tilted or temporarily degenerate LIO
            # estimate must never rotate horizontal motion into altitude.
            self.handover_lio_position = (
                lio.position.x, lio.position.y, lio.position.z)
            self.handover_px4_position = (
                px4.position.x, px4.position.y, px4.position.z)
            self.handover_yaw_offset = wrap_pi(
                q_yaw(pose_q(px4)) - q_yaw(pose_q(lio)))
            self.handover_px4_yaw = q_yaw(pose_q(px4))
            rospy.loginfo(
                "%s latched EGO frame: LIO=(%.3f, %.3f, %.3f), PX4=(%.3f, %.3f, %.3f), yaw_offset=%.1f deg, hold_yaw=%s",
                self.name,
                *self.handover_lio_position,
                *self.handover_px4_position,
                math.degrees(self.handover_yaw_offset), self.hold_yaw)

        q_rel = q_from_yaw(self.handover_yaw_offset)
        delta_lio = (
            cmd.position.x - self.handover_lio_position[0],
            cmd.position.y - self.handover_lio_position[1],
            cmd.position.z - self.handover_lio_position[2],
        )
        delta_px4 = q_rotate(q_rel, delta_lio)
        if self.hold_z:
            delta_px4 = (delta_px4[0], delta_px4[1], 0.0)
        if (math.hypot(delta_px4[0], delta_px4[1]) > self.max_xy or
                abs(delta_px4[2]) > self.max_z):
            rospy.logwarn_throttle(
                1.0, "%s rejected EGO command jump: d=(%.2f, %.2f, %.2f)",
                self.name, delta_px4[0], delta_px4[1], delta_px4[2])
            return self.last_output

        out = Pose()
        out.position.x = self.handover_px4_position[0] + delta_px4[0]
        out.position.y = self.handover_px4_position[1] + delta_px4[1]
        out.position.z = self.handover_px4_position[2] + delta_px4[2]
        if self.hold_z:
            out.position.z = self.hold_z_value
        if self.hold_yaw:
            q_out = q_from_yaw(self.handover_px4_yaw)
        else:
            q_out = q_from_yaw(wrap_pi(self.handover_yaw_offset + q_yaw(q_cmd)))
        q_out = q_norm(q_out)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out
        self.last_output = out
        return out

    def spin(self):
        rate = rospy.Rate(30)
        rospy.loginfo(
            "%s EGO adapter: /cmd_pose_lio -> /cmd_pose_enu (%s frame)",
            self.name, self.command_frame)
        while not rospy.is_shutdown():
            out = self.converted_command()
            if out is not None:
                self.pub.publish(out)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ego_lio_to_px4_bridge", anonymous=False)
    name = rospy.get_param("~vehicle", sys.argv[1] if len(sys.argv) > 1 else "iris_0")
    bridge = LioToPx4Bridge(name)
    bridge.attach_subscribers()
    bridge.spin()
