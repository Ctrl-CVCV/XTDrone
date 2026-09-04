#!/usr/bin/env python3
"""Start/verify the XTDrone bridges, switch UAVs to OFFBOARD, arm and take off."""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import rosnode
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import ParamValue, PositionTarget, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from std_msgs.msg import Bool, Float64, String


# MPC_LAND_CRWL: 着陆爬行速度阈值，PX4 地检据此判定"垂直运动"。
# 默认 0.5 -> 垂向运动阈值 0.225 m/s，SITL 中 EKF vz 噪声(~0.17-0.25)在 ARM 时
# 会误触发，导致 landed 标志清除 -> 起飞斜坡永不启动 -> 0 推力。调大后阈值 0.9。
LAND_CRWL_PARAM = "MPC_LAND_CRWL"
LAND_CRWL_VALUE = 2.0
LAND_CRWL_NORMAL_VALUE = 0.5

# The PX4 SITL airframe has a hover-thrust estimator enabled by default.
# With the extra 0.2 kg MID360 payload this estimator can overwrite the
# explicitly calibrated MPC_THR_HOVER while the vehicle is climbing.  That
# makes the throttle/physical altitude test non-repeatable, so this test uses
# the fixed hover value below.
HOVER_THR_ESTIMATOR_PARAM = "MPC_USE_HTE"
HOVER_THR_ESTIMATOR_VALUE = 0
HOVER_THR_PARAM = "MPC_THR_HOVER"
# The iris model is 1.5 kg and the attached Mid360 model adds 0.2 kg.  With
# HTE disabled, 0.60 still lost altitude after the EGO handover, especially
# when horizontal acceleration consumed part of the available thrust.  Use a
# slightly higher fixed hover point for the 1.5 kg iris + 0.2 kg Mid360 model;
# the position loop will settle the altitude back to its target.
# The attached Mid360 model adds 0.2 kg.  The motor model is calibrated below
# from the stock 1.5 kg Iris value, so do not compensate the same payload a
# second time with an overly large PX4 hover point.  0.62 is the measured
# starting point for the 1.7 kg combined model and leaves the position loop
# room to correct altitude without a runaway climb.
HOVER_THR_VALUE = 0.62

# Keep the PX4 position loop compatible with the conservative EGO trajectory
# used for the attached Mid360 payload.  Without this cap PX4 can request a
# sharp horizontal tilt even when the planner's nominal acceleration is low.
MPC_XY_VEL_MAX_PARAM = "MPC_XY_VEL_MAX"
MPC_XY_VEL_MAX_VALUE = 0.25
MPC_ACC_HOR_PARAM = "MPC_ACC_HOR"
MPC_ACC_HOR_VALUE = 0.5
MPC_ACC_HOR_MAX_PARAM = "MPC_ACC_HOR_MAX"
MPC_ACC_HOR_MAX_VALUE = 0.5

# This SITL is controlled entirely through MAVROS. A joystick-only setting
# makes PX4 immediately override OFFBOARD with NAV_RCL_ACT=AUTO.RTL.
COM_RC_IN_MODE_DISABLED = 4
COM_RCL_EXCEPT_OFFBOARD = 4

# PX4 v1.13 EKF2_AID_MASK: bit 3 = external-vision position,
# bit 4 = external-vision yaw. GPS bit 0 must stay disabled.
EKF2_AID_MASK_EXPECTED = 24
# The validated acceptance path is full SWARM-LIO external vision, including
# Z.  The script still guards stale/jumping LIO samples and the Gazebo truth
# watchdog still requests AUTO.LAND on a persistent fault.  Baro remains
# available as an explicit comparison mode (--height-source baro), but it must
# not be the default when the requested behavior is SWARM-LIO localization.
EKF2_HGT_MODE_BARO = 0
EKF2_HGT_MODE_VISION = 3
EKF2_EV_DELAY_MS = 20.0

# Gazebo with two high-density ray sensors can run slower than wall time while
# still advancing simulation time normally.  In that case a wall-clock
# watchdog would mistake scheduler delay for a missing LIO message.  Keep a
# separate wall-clock check for /clock itself, and use message/header time for
# topic freshness when /use_sim_time is enabled.
SIM_TOPIC_MAX_AGE = 0.6
SIM_CLOCK_MAX_WALL_AGE = 2.0

DEFAULT_UAVS = ("iris_0", "iris_1")
USE_TRUTH = False
USE_SIM_TIME = False
UAVS = DEFAULT_UAVS
BRIDGE_DIR = "/home/dev/XTDrone/communication"
BRIDGE_SCRIPT = os.path.join(BRIDGE_DIR, "multirotor_communication.py")


class UavStatus:
    def __init__(self, name):
        self.name = name
        self.state = None
        self.pose = None
        self.vision_pose = None
        self.vision_pose_count = 0
        self.vision_pose_last_wall = None
        self.vision_pose_last_stamp = rospy.Time(0)
        self.camera_pose_count = 0
        self.setpoint_count = 0
        self.reference_vision_z = None
        self.reference_truth_z = None
        self.watchdog_fault = None
        self.watchdog_fault_count = 0
        self.watchdog_reached = False
        rospy.Subscriber("/%s/mavros/state" % name, State, self._state_cb, queue_size=1)
        self._pose_sub = rospy.Subscriber(
            "/%s/mavros/local_position/pose" % name,
            PoseStamped,
            self._pose_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/%s/mavros/vision_pose/pose" % name,
            PoseStamped,
            self._vision_pose_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/%s/camera_pose" % name,
            PoseStamped,
            self._camera_pose_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/%s/mavros/setpoint_raw/local" % name,
            PositionTarget,
            self._setpoint_cb,
            queue_size=10,
        )
        self.cmd_pub = rospy.Publisher(
            "/xtdrone/%s/cmd" % name, String, queue_size=5
        )
        self.pose_pub = rospy.Publisher(
            "/xtdrone/%s/cmd_pose_enu" % name, Pose, queue_size=5
        )
        self.ego_takeover_pub = rospy.Publisher(
            "/xtdrone/%s/ego_takeover" % name,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.takeoff_height_pub = rospy.Publisher(
            "/xtdrone/%s/takeoff_height" % name,
            Float64,
            queue_size=1,
            latch=True,
        )
        self.set_mode = rospy.ServiceProxy(
            "/%s/mavros/set_mode" % name, SetMode
        )
        self.arm = rospy.ServiceProxy(
            "/%s/mavros/cmd/arming" % name, CommandBool
        )
        self.set_param = rospy.ServiceProxy(
            "/%s/mavros/param/set" % name, ParamSet
        )
        self.get_param = rospy.ServiceProxy(
            "/%s/mavros/param/get" % name, ParamGet
        )

    def relax_land_detector(self, timeout):
        """调大 MPC_LAND_CRWL，防止 SITL EKF vz 噪声在 ARM 时误清除 landed。

        PX4 地检的垂向运动阈值 = min(0.9*MPC_LAND_CRWL*0.5, LNDMC_Z_VEL_MAX)。
        默认 0.5 -> 0.225 m/s，过于敏感；2.0 -> 0.9 m/s。必须在 ARM 前设置。
        """
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                resp = self.set_param(
                    LAND_CRWL_PARAM, ParamValue(integer=0, real=LAND_CRWL_VALUE)
                )
            except rospy.ServiceException:
                rospy.sleep(0.5)
                continue
            if resp.success:
                rospy.loginfo("%s %s=%.2f 已设置", self.name, LAND_CRWL_PARAM, LAND_CRWL_VALUE)
                return
            rospy.logwarn("%s 设置 %s 被拒绝，重试", self.name, LAND_CRWL_PARAM)
            rospy.sleep(0.5)
        raise RuntimeError("%s 无法设置 %s=%s" % (self.name, LAND_CRWL_PARAM, LAND_CRWL_VALUE))

    def restore_land_detector(self, timeout):
        """起飞完成后恢复正常地检阈值，保证 AUTO.LAND 能识别落地。"""
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                resp = self.set_param(
                    LAND_CRWL_PARAM,
                    ParamValue(integer=0, real=LAND_CRWL_NORMAL_VALUE),
                )
                verify = self.get_param(LAND_CRWL_PARAM)
            except rospy.ServiceException:
                rospy.sleep(0.5)
                continue
            if (
                resp.success
                and verify.success
                and abs(verify.value.real - LAND_CRWL_NORMAL_VALUE) < 1e-3
            ):
                rospy.loginfo(
                    "%s %s=%.2f 已恢复，AUTO.LAND 地检启用",
                    self.name,
                    LAND_CRWL_PARAM,
                    LAND_CRWL_NORMAL_VALUE,
                )
                return
            rospy.sleep(0.5)
        raise RuntimeError(
            "%s 无法恢复 %s=%.2f"
            % (self.name, LAND_CRWL_PARAM, LAND_CRWL_NORMAL_VALUE)
        )

    def configure_autonomous_rc(self, timeout):
        """禁用不存在的摇杆输入，并在 OFFBOARD 中忽略 RC-loss。"""
        expected = (
            ("COM_RC_IN_MODE", COM_RC_IN_MODE_DISABLED),
            ("COM_RCL_EXCEPT", COM_RCL_EXCEPT_OFFBOARD),
        )
        for param_id, value in expected:
            deadline = time.monotonic() + timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    response = self.set_param(
                        param_id, ParamValue(integer=value, real=0.0)
                    )
                    verify = self.get_param(param_id)
                except rospy.ServiceException:
                    rospy.sleep(0.5)
                    continue
                if response.success and verify.success and verify.value.integer == value:
                    break
                rospy.sleep(0.5)
            else:
                raise RuntimeError(
                    "%s 无法设置 %s=%d" % (self.name, param_id, value)
                )
        rospy.loginfo(
            "%s 自主飞行 RC 配置已确认：COM_RC_IN_MODE=4, COM_RCL_EXCEPT=4",
            self.name,
        )

    def configure_hover_thrust(self, timeout):
        """Use a deterministic fixed hover thrust for the MID360 payload."""
        params = (
            (HOVER_THR_ESTIMATOR_PARAM,
             ParamValue(integer=HOVER_THR_ESTIMATOR_VALUE, real=0.0),
             lambda value: value.integer == HOVER_THR_ESTIMATOR_VALUE),
            (HOVER_THR_PARAM,
             ParamValue(integer=0, real=HOVER_THR_VALUE),
             lambda value: abs(value.real - HOVER_THR_VALUE) < 1e-3),
        )
        for param_id, value, valid in params:
            deadline = time.monotonic() + timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    response = self.set_param(param_id, value)
                    verify = self.get_param(param_id)
                except rospy.ServiceException:
                    rospy.sleep(0.5)
                    continue
                if response.success and verify.success and valid(verify.value):
                    break
                rospy.sleep(0.5)
            else:
                raise RuntimeError("%s 无法设置 %s" % (self.name, param_id))
        rospy.loginfo(
            "%s 固定悬停推力已确认：MPC_USE_HTE=0, MPC_THR_HOVER=%.2f",
            self.name, HOVER_THR_VALUE)

    def configure_motion_limits(self, timeout):
        """Limit horizontal PX4 demands during the first EGO flight test."""
        params = (
            (MPC_XY_VEL_MAX_PARAM, MPC_XY_VEL_MAX_VALUE),
            (MPC_ACC_HOR_PARAM, MPC_ACC_HOR_VALUE),
            (MPC_ACC_HOR_MAX_PARAM, MPC_ACC_HOR_MAX_VALUE),
        )
        for param_id, target in params:
            deadline = time.monotonic() + timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    response = self.set_param(
                        param_id, ParamValue(integer=0, real=target)
                    )
                    verify = self.get_param(param_id)
                except rospy.ServiceException:
                    rospy.sleep(0.5)
                    continue
                if response.success and verify.success and abs(verify.value.real - target) < 1e-3:
                    break
                rospy.sleep(0.5)
            else:
                raise RuntimeError("%s 无法设置 %s=%.2f" % (self.name, param_id, target))
        rospy.loginfo(
            "%s PX4 横向限制已确认：MPC_XY_VEL_MAX=%.2f, MPC_ACC_HOR=%.2f, MPC_ACC_HOR_MAX=%.2f",
            self.name, MPC_XY_VEL_MAX_VALUE, MPC_ACC_HOR_VALUE, MPC_ACC_HOR_MAX_VALUE)

    def _state_cb(self, msg):
        self.state = msg


    def _pose_cb(self, msg):
        self.pose = msg

    def _vision_pose_cb(self, msg):
        self.vision_pose = msg
        self.vision_pose_count += 1
        self.vision_pose_last_wall = time.monotonic()
        self.vision_pose_last_stamp = msg.header.stamp

    def _camera_pose_cb(self, _msg):
        self.camera_pose_count += 1

    def _setpoint_cb(self, _msg):
        self.setpoint_count += 1

    def require_swarm_lio_localization(self, expected_height_mode):
        """Refuse takeoff unless PX4 is configured to fuse this UAV's LIO pose."""
        expected_frame = "map" if USE_TRUTH else "quad%d/world" % _uav_index(self.name)
        if self.vision_pose is None or self.vision_pose.header.frame_id != expected_frame:
            got = "<none>" if self.vision_pose is None else self.vision_pose.header.frame_id
            raise RuntimeError(
                "%s vision pose frame=%s，期望 %s；拒绝使用非 SWARM-LIO 定位起飞"
                % (self.name, got, expected_frame)
            )

        if USE_TRUTH:
            rospy.loginfo("%s 已确认 Gazebo 真值 -> MAVROS，frame_id=map", self.name)
            return

        aid = self.get_param("EKF2_AID_MASK")
        height = self.get_param("EKF2_HGT_MODE")
        if not aid.success or aid.value.integer != EKF2_AID_MASK_EXPECTED:
            raise RuntimeError(
                "%s EKF2_AID_MASK=%s，期望 24（视觉位置+航向且禁用 GPS）；参数需重启 PX4 生效"
                % (self.name, aid.value.integer if aid.success else "读取失败")
            )
        if not height.success or height.value.integer != expected_height_mode:
            raise RuntimeError(
                "%s EKF2_HGT_MODE=%s，期望 %d"
                % (
                    self.name,
                    height.value.integer if height.success else "读取失败",
                    expected_height_mode,
                )
            )

        self.set_ev_delay_with_retry(30.0)
        rospy.loginfo(
            "%s 已确认 SWARM-LIO -> MAVROS：%s，EV_DELAY=%.1f ms",
            self.name,
            (
                "完整视觉位置/航向（含 LIO-Z）"
                if expected_height_mode == EKF2_HGT_MODE_VISION
                else "视觉水平位置/航向 + Baro 主高度"
            ),
            EKF2_EV_DELAY_MS,
        )

    def configure_height_source(self, timeout, height_mode):
        """Select and verify the requested EKF2 primary height source."""
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                response = self.set_param(
                    "EKF2_HGT_MODE",
                    ParamValue(integer=height_mode, real=0.0),
                )
                verify = self.get_param("EKF2_HGT_MODE")
            except rospy.ServiceException:
                rospy.sleep(0.5)
                continue
            if (
                response.success
                and verify.success
                and verify.value.integer == height_mode
            ):
                rospy.loginfo(
                    "%s EKF2_HGT_MODE=%d：%s",
                    self.name,
                    height_mode,
                    "完整 SWARM-LIO 高度" if height_mode == EKF2_HGT_MODE_VISION else "Baro 主高度",
                )
                return
            rospy.sleep(0.5)
        raise RuntimeError("%s 无法设置 EKF2_HGT_MODE=%d" % (self.name, height_mode))

    def set_ev_delay_with_retry(self, timeout):
        """设置并回读 EKF2_EV_DELAY；MAVROS 参数服务偶尔会短暂不可用。"""
        deadline = time.monotonic() + timeout
        service_name = "/%s/mavros/param/set" % self.name
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                rospy.wait_for_service(service_name, timeout=2.0)
                current = self.get_param("EKF2_EV_DELAY")
                if current.success and abs(current.value.real - EKF2_EV_DELAY_MS) < 1e-3:
                    return
                response = self.set_param(
                    "EKF2_EV_DELAY", ParamValue(integer=0, real=EKF2_EV_DELAY_MS)
                )
                if response.success:
                    verify = self.get_param("EKF2_EV_DELAY")
                    if verify.success and abs(verify.value.real - EKF2_EV_DELAY_MS) < 1e-3:
                        return
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn_throttle(2.0, "%s 等待 EKF2_EV_DELAY 服务：%s", self.name, exc)
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            raise RuntimeError("ROS 已关闭，无法设置 %s 的 EKF2_EV_DELAY" % self.name)
        raise RuntimeError("%s 无法设置 EKF2_EV_DELAY=%.1f ms" % (self.name, EKF2_EV_DELAY_MS))

    def set_watchdog_reference(self, truth_pose):
        self.reference_vision_z = self.vision_pose.pose.position.z
        self.reference_truth_z = truth_pose.position.z
        self.watchdog_fault = None
        self.watchdog_fault_count = 0


class GazeboTruth:
    """Gazebo truth is used only as a simulation safety watchdog, never as PX4 input."""

    def __init__(self):
        self.poses = {}
        self.last_wall = None
        self.last_stamp = rospy.Time(0)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._callback, queue_size=1)

    def _callback(self, msg):
        self.poses = dict(zip(msg.name, msg.pose))
        self.last_wall = time.monotonic()
        # gazebo_msgs/ModelStates has no Header.  The callback is dispatched
        # under the current simulated clock, so this receipt timestamp is the
        # correct freshness reference for the simulation-time watchdog.
        self.last_stamp = rospy.Time.now()

    def ready(self, names):
        return (
            self.last_wall is not None
            and time.monotonic() - self.last_wall < (SIM_CLOCK_MAX_WALL_AGE if USE_SIM_TIME else 1.0)
            and all(name in self.poses for name in names)
        )


class SimClockHealth:
    """Track whether Gazebo's /clock is advancing in wall time."""

    def __init__(self):
        self.last_wall = None
        self.last_stamp = rospy.Time(0)
        rospy.Subscriber("/clock", rospy.AnyMsg, self._callback, queue_size=10)

    def _callback(self, _msg):
        self.last_wall = time.monotonic()


def _sim_age(stamp):
    """Return age in ROS/simulation seconds, or None for an invalid stamp."""
    if stamp is None or stamp == rospy.Time(0):
        return None
    age = (rospy.Time.now() - stamp).to_sec()
    # A delayed callback can briefly deliver a message newer than the latest
    # /clock sample; that is not a stale-message condition.
    return max(0.0, age)


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        rate.sleep()
    raise RuntimeError("等待超时：%s" % description)


def _uav_index(name):
    match = re.search(r"(\d+)$", name)
    if match is None:
        raise RuntimeError("UAV 名称必须以数字结尾: %s" % name)
    return int(match.group(1))


def start_missing_bridges():
    started = []
    try:
        node_names = set(rosnode.get_node_names())
    except rosnode.ROSNodeIOException:
        node_names = set()

    for name in UAVS:
        node_name = "/%s_communication" % name
        if node_name in node_names:
            rospy.loginfo("%s 通信桥已运行", name)
            continue
        rospy.loginfo("启动 %s XTDrone 通信桥", name)
        process = subprocess.Popen(
            [sys.executable, BRIDGE_SCRIPT, "iris", str(_uav_index(name))],
            cwd=BRIDGE_DIR,
        )
        started.append(process)

    if started:
        expected = {"/%s_communication" % name for name in UAVS}
        wait_until(lambda: expected.issubset(set(rosnode.get_node_names())),
                   15.0, "XTDrone 通信桥注册")
    return started


def publish_hover(statuses):
    # A newly started bridge may not have received local_position on its own
    # subscriber yet. Repeat HOVER briefly so each bridge can take a valid hold
    # snapshot without racing its first pose callback.
    rate = rospy.Rate(10)
    for _ in range(20):
        for status in statuses:
            status.cmd_pub.publish(String(data="HOVER"))
        rate.sleep()


def request_mode(status, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if status.state and status.state.mode == "OFFBOARD":
            return
        response = status.set_mode(base_mode=0, custom_mode="OFFBOARD")
        if not response.mode_sent:
            rospy.logwarn("%s OFFBOARD 请求被拒绝，准备重试", status.name)
        rospy.sleep(1.0)
    raise RuntimeError("%s 无法进入 OFFBOARD" % status.name)


def request_arm(status, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if status.state and status.state.armed:
            return
        response = status.arm(value=True)
        if not response.success:
            rospy.logwarn("%s ARM 被拒绝，准备重试", status.name)
        rospy.sleep(1.0)
    raise RuntimeError("%s 无法 ARM" % status.name)


def make_takeoff_pose(status, altitude):
    target = Pose()
    target.position.x = status.pose.pose.position.x
    target.position.y = status.pose.pose.position.y
    target.position.z = altitude
    target.orientation = status.pose.pose.orientation
    if (
        target.orientation.x == 0.0
        and target.orientation.y == 0.0
        and target.orientation.z == 0.0
        and target.orientation.w == 0.0
    ):
        target.orientation.w = 1.0
    return target


def _roll_pitch_deg(orientation):
    """Return ROS/ENU roll and pitch from a normalized quaternion."""
    x, y, z, w = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return float("inf"), float("inf")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return math.degrees(roll), math.degrees(pitch)


def watchdog_fault(
    status,
    truth,
    clock,
    target_delta,
    max_height_error,
    max_tilt_deg,
    expected_height_mode,
):
    """Return a sustained LIO/truth disagreement, or None while healthy."""
    now = time.monotonic()
    if USE_SIM_TIME:
        if clock.last_wall is None or now - clock.last_wall > SIM_CLOCK_MAX_WALL_AGE:
            return "Gazebo /clock 超过 %.1f s 未推进" % SIM_CLOCK_MAX_WALL_AGE
        vision_age = _sim_age(status.vision_pose_last_stamp)
        if vision_age is None or vision_age > SIM_TOPIC_MAX_AGE:
            return "SWARM-LIO vision_pose 仿真时间超过 %.1f s 未更新" % SIM_TOPIC_MAX_AGE
        truth_age = _sim_age(truth.last_stamp)
        if truth_age is None or truth_age > SIM_TOPIC_MAX_AGE:
            return "Gazebo model_states 仿真时间超过 %.1f s 未更新" % SIM_TOPIC_MAX_AGE
    else:
        if status.vision_pose_last_wall is None or now - status.vision_pose_last_wall > SIM_TOPIC_MAX_AGE:
            return "SWARM-LIO vision_pose 超过 %.1f s 未更新" % SIM_TOPIC_MAX_AGE
        if truth.last_wall is None or now - truth.last_wall > SIM_TOPIC_MAX_AGE:
            return "Gazebo model_states 超过 %.1f s 未更新" % SIM_TOPIC_MAX_AGE
    if status.name not in truth.poses:
        return "Gazebo 中找不到模型 %s" % status.name

    truth_dz = truth.poses[status.name].position.z - status.reference_truth_z
    lio_dz = status.vision_pose.pose.position.z - status.reference_vision_z
    height_error = truth_dz - lio_dz
    roll, pitch = _roll_pitch_deg(status.vision_pose.pose.orientation)

    # Overshoot is immediately dangerous; the other tests are debounced below.
    if truth_dz > target_delta + 0.5:
        return (
            "真实高度超调：Gazebo Δz=%.2f m，目标 Δz=%.2f m"
            % (truth_dz, target_delta)
        )
    if status.watchdog_reached and truth_dz < target_delta - 0.35:
        return (
            "起飞后真实高度骤降：Gazebo Δz=%.2f m，目标 Δz=%.2f m"
            % (truth_dz, target_delta)
        )
    # In baro mode PX4 deliberately does not use LIO-Z as its primary height
    # source.  LIO-Z is still published for diagnostics, but comparing it to
    # Gazebo height here would make a valid baro flight fail by construction.
    if expected_height_mode == EKF2_HGT_MODE_VISION and abs(height_error) > max_height_error:
        return (
            "LIO 高度失锁：Gazebo Δz=%.2f m，LIO Δz=%.2f m，误差=%.2f m"
            % (truth_dz, lio_dz, height_error)
        )
    if max(abs(roll), abs(pitch)) > max_tilt_deg:
        return "LIO 姿态失锁：roll=%.1f°, pitch=%.1f°" % (roll, pitch)
    return None


def check_watchdog(
    statuses,
    truth,
    clock,
    target_deltas,
    max_height_error,
    max_tilt_deg,
    expected_height_mode,
):
    """Debounce noisy samples; raise after five consecutive bad measurements."""
    for status, target_delta in zip(statuses, target_deltas):
        fault = watchdog_fault(
            status,
            truth,
            clock,
            target_delta,
            max_height_error,
            max_tilt_deg,
            expected_height_mode,
        )
        if fault is None:
            status.watchdog_fault = None
            status.watchdog_fault_count = 0
            continue
        status.watchdog_fault = fault
        status.watchdog_fault_count += 1
        # Gazebo/PX4 altitude has a short transient overshoot when two
        # vehicles are released in sequence.  Treat it like the other
        # watchdog faults and require persistence; a runaway climb still
        # trips after five samples, while a one-sample controller transient
        # does not abort an otherwise recoverable flight.
        if status.watchdog_fault_count >= 5:
            raise RuntimeError("%s 安全看门狗触发：%s" % (status.name, fault))


def emergency_land(statuses, reason):
    """Best-effort transition to AUTO.LAND before relinquishing bridge control."""
    armed = [status for status in statuses if status.state and status.state.armed]
    if not armed:
        return
    rospy.logerr("触发紧急降落：%s", reason)
    for status in armed:
        try:
            status.set_param(
                LAND_CRWL_PARAM,
                ParamValue(integer=0, real=LAND_CRWL_NORMAL_VALUE),
            )
        except rospy.ServiceException as exc:
            rospy.logwarn("%s 恢复地检参数失败：%s", status.name, exc)
        status.cmd_pub.publish(String(data="AUTO.LAND"))
        try:
            response = status.set_mode(base_mode=0, custom_mode="AUTO.LAND")
            if not response.mode_sent:
                rospy.logwarn("%s AUTO.LAND 请求被拒绝", status.name)
        except rospy.ServiceException as exc:
            rospy.logwarn("%s AUTO.LAND 服务失败：%s", status.name, exc)
    # Keep bridges and ROS callbacks alive long enough for PX4 to latch the mode.
    deadline = time.monotonic() + 3.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if all(s.state and s.state.mode == "AUTO.LAND" for s in armed):
            break
        rospy.sleep(0.1)
    rospy.logerr(
        "降落模式状态：%s",
        ", ".join("%s=%s" % (s.name, s.state.mode if s.state else "unknown") for s in armed),
    )


def main():
    parser = argparse.ArgumentParser(description="Iris 一键 OFFBOARD、ARM、起飞")
    parser.add_argument("--altitude", type=float, default=1.5, help="本地起飞高度，默认 1.5 m")
    parser.add_argument("--timeout", type=float, default=30.0, help="每个阶段超时秒数")
    parser.add_argument(
        "--no-start-bridge",
        action="store_true",
        help="不自动启动缺失的 XTDrone 通信桥",
    )
    parser.add_argument(
        "--truth",
        action="store_true",
        help="使用 Gazebo 真值 map 位姿测试；禁止 SWARM-LIO vision_pose",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="只验证 SWARM-LIO/MAVROS/EKF2 定位链路，不启动通信桥、不切模式、不解锁",
    )
    parser.add_argument(
        "--uavs",
        default=",".join(DEFAULT_UAVS),
        help="要起飞的无人机名，逗号分隔；默认 %s" % ",".join(DEFAULT_UAVS),
    )
    parser.add_argument(
        "--allow-multi",
        action="store_true",
        help="显式允许多机同时起飞；单机看门狗测试通过前不要使用",
    )
    parser.add_argument(
        "--stagger-takeoff",
        action="store_true",
        help="多机时按列表顺序起飞；已起飞机保持高度，避免同时起飞瞬态掉高",
    )
    parser.add_argument(
        "--max-height-error",
        type=float,
        default=0.35,
        help="Gazebo 与 SWARM-LIO 高度增量最大允许误差，默认 0.35 m",
    )
    parser.add_argument(
        "--max-lio-tilt",
        type=float,
        default=15.0,
        help="SWARM-LIO 最大允许 roll/pitch，默认 15 度",
    )
    parser.add_argument(
        "--height-source",
        choices=("baro", "vision"),
        default="vision",
        help="PX4 主高度源；默认 vision（完整使用 SWARM-LIO-Z），baro 仅用于对照",
    )
    parser.add_argument(
        "--no-ego-handover",
        action="store_true",
        help="起飞成功后继续由本脚本保持高度，不把控制权交给 EGO-Swarm",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=30.0,
        help="--no-ego-handover 时保持目标高度的秒数，默认 30；设为 0 表示持续保持",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    global UAVS, USE_TRUTH, USE_SIM_TIME
    USE_TRUTH = args.truth
    USE_SIM_TIME = bool(rospy.get_param("/use_sim_time", False))
    UAVS = tuple(name.strip() for name in args.uavs.split(",") if name.strip())
    expected_height_mode = (
        EKF2_HGT_MODE_VISION if args.height_source == "vision" else EKF2_HGT_MODE_BARO
    )

    if args.altitude < 0.2:
        raise RuntimeError("起飞高度不能低于 0.2 m")
    if len(UAVS) > 1 and not args.allow_multi and not args.check_only:
        raise RuntimeError(
            "当前默认禁止多机同时起飞；请先逐机通过 0.3 m 看门狗测试，"
            "确认后才可加 --allow-multi"
        )

    rospy.init_node("uav_offboard_takeoff", anonymous=False)
    bridge_processes = []
    statuses = []
    try:
        statuses = [UavStatus(name) for name in UAVS]
        truth = GazeboTruth()
        clock = SimClockHealth()
        wait_until(
            lambda: all(
                s.vision_pose_count >= 3
                and s.vision_pose is not None
                and s.vision_pose.header.frame_id == ("map" if USE_TRUTH else "quad%d/world" % _uav_index(s.name))
                for s in statuses
            ),
            args.timeout,
            "SWARM-LIO vision pose；不要运行 get_local_pose.py，先启动 dual_mid360_distributed.launch",
        )
        if not args.check_only and not args.no_start_bridge:
            bridge_processes = start_missing_bridges()

        wait_until(
            lambda: all(s.state and s.state.connected and s.pose for s in statuses),
            args.timeout,
            "%d 套 MAVROS connected 且 local pose 有数据" % len(UAVS),
        )
        wait_until(
            lambda: truth.ready(UAVS),
            args.timeout,
            "Gazebo model_states 安全看门狗（只用于检测，不送入 PX4）",
        )

        for status in statuses:
            if not USE_TRUTH:
                status.configure_height_source(args.timeout, expected_height_mode)
            status.require_swarm_lio_localization(expected_height_mode)

        if args.check_only:
            rospy.loginfo(
                "%d 机 SWARM-LIO -> MAVROS -> PX4 EKF2 定位链路检查通过；未执行 OFFBOARD/ARM",
                len(statuses),
            )
            return

        if not USE_TRUTH:
            # SWARM-LIO is the localization source for this test.  The old
            # ego_swarm_transfer.py camera_pose topic is only a legacy EGO
            # visualization interface and is not consumed by the current
            # LIO->PX4 adapter.  Requiring it here made a valid test fail
            # before OFFBOARD whenever the legacy process was not running.
            wait_until(
                lambda: all(
                    s.vision_pose_count >= 3
                    and s.vision_pose is not None
                    and s.vision_pose_last_wall is not None
                    and time.monotonic() - s.vision_pose_last_wall < SIM_TOPIC_MAX_AGE
                    for s in statuses
                ),
                args.timeout,
                "SWARM-LIO vision_pose 持续更新",
            )

        # Give publishers/subscribers time to connect, then let the bridge hold position.
        # Explicitly keep EGO adapters out of control during takeoff, even if
        # their planner has already received a goal and is publishing a pose.
        for status in statuses:
            # Tell the EGO adapter the height that this takeoff run is
            # requesting.  It will use this latched value when ownership is
            # handed over, avoiding a transient PX4-height lag at handover.
            status.takeoff_height_pub.publish(Float64(data=args.altitude))
            status.ego_takeover_pub.publish(Bool(data=False))
        wait_until(
            lambda: all(s.cmd_pub.get_num_connections() > 0 for s in statuses),
            args.timeout,
            "%d 套 XTDrone 通信桥订阅控制命令" % len(UAVS),
        )

        # 禁用不存在的 joystick/RC，否则 PX4 会用 AUTO.RTL 覆盖 OFFBOARD。
        for status in statuses:
            status.configure_autonomous_rc(args.timeout)

        # Keep the extra MID360 payload's hover calibration fixed during the
        # test; otherwise PX4's estimator may change MPC_THR_HOVER in flight.
        for status in statuses:
            status.configure_hover_thrust(args.timeout)

        for status in statuses:
            status.configure_motion_limits(args.timeout)

        # 必须在 OFFBOARD/ARM 前调大地检垂向运动阈值，否则 SITL EKF vz 噪声
        # 会在 ARM 时误清除 landed -> 起飞斜坡不启动 -> 0 推力、飞机不起飞。
        for status in statuses:
            status.relax_land_detector(args.timeout)

        publish_hover(statuses)

        for status in statuses:
            status.setpoint_count = 0
        rospy.sleep(1.0)
        if not all(s.setpoint_count >= 3 for s in statuses):
            raise RuntimeError("通信桥没有持续发布 MAVROS setpoint，拒绝切换 OFFBOARD")

        # Baselines are captured as late as possible, immediately before OFFBOARD/ARM.
        for status in statuses:
            status.set_watchdog_reference(truth.poses[status.name])

        for status in statuses:
            request_mode(status, args.timeout)
        for status in statuses:
            request_arm(status, args.timeout)

        # Keep the XTDrone bridge control latch in sync with the MAVROS mode.
        # This releases the initial HOVER hold before takeoff pose streaming.
        for status in statuses:
            status.cmd_pub.publish(String(data="OFFBOARD"))
        rospy.sleep(0.5)

        targets = [make_takeoff_pose(s, args.altitude) for s in statuses]
        target_deltas = [
            target.position.z - status.pose.pose.position.z
            for status, target in zip(statuses, targets)
        ]
        if any(delta <= 0.05 for delta in target_deltas):
            raise RuntimeError(
                "目标高度必须至少高于当前 MAVROS 高度 0.05 m；当前增量=%s"
                % ", ".join("%.3f" % delta for delta in target_deltas)
            )
        rospy.loginfo("%d 机已进入 OFFBOARD 并 ARM，开始原地起飞到 %.2f m", len(UAVS), args.altitude)
        rate = rospy.Rate(10)
        deadline = time.monotonic() + args.timeout
        reached = False

        def one_reached(status, target, target_delta):
            return (
                status.pose
                and status.pose.pose.position.z >= target.position.z - 0.2
                and truth.poses[status.name].position.z - status.reference_truth_z
                >= target_delta - 0.2
                and (
                    expected_height_mode != EKF2_HGT_MODE_VISION
                    or status.vision_pose.pose.position.z - status.reference_vision_z
                    >= target_delta - 0.2
                )
            )

        if args.stagger_takeoff and len(statuses) > 1:
            # Keep every vehicle in the same OFFBOARD session, but only ask
            # one vehicle at a time to leave the ground.  A vehicle already
            # airborne keeps receiving its altitude target; the remaining
            # vehicles keep the HOVER target installed above.
            for active_index, active_status in enumerate(statuses):
                rospy.loginfo(
                    "分批起飞：%s 到 %.2f m；前序飞机保持高度",
                    active_status.name, args.altitude)
                # The inactive vehicle is deliberately kept in HOVER below.
                # The XTDrone bridge ignores pose callbacks while its
                # hover_flag is set, so release this vehicle before sending
                # its first takeoff setpoint.
                active_status.cmd_pub.publish(String(data="OFFBOARD"))
                active_deadline = time.monotonic() + args.timeout
                while not rospy.is_shutdown() and time.monotonic() < active_deadline:
                    for index, (status, target) in enumerate(zip(statuses, targets)):
                        if index <= active_index:
                            status.pose_pub.publish(target)
                        else:
                            status.cmd_pub.publish(String(data="HOVER"))
                    check_watchdog(
                        statuses,
                        truth,
                        clock,
                        target_deltas,
                        args.max_height_error,
                        args.max_lio_tilt,
                        expected_height_mode,
                    )
                    if one_reached(
                        active_status, targets[active_index], target_deltas[active_index]
                    ):
                        active_status.watchdog_reached = True
                        break
                    rate.sleep()
                else:
                    raise RuntimeError("分批起飞超时：%s" % active_status.name)
            reached = all(
                one_reached(status, target, target_delta)
                for status, target, target_delta in zip(
                    statuses, targets, target_deltas
                )
            )
        else:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                for status, target in zip(statuses, targets):
                    status.pose_pub.publish(target)
                check_watchdog(
                    statuses,
                    truth,
                    clock,
                    target_deltas,
                    args.max_height_error,
                    args.max_lio_tilt,
                    expected_height_mode,
                )
                reached = all(
                    one_reached(status, target, target_delta)
                    for status, target, target_delta in zip(
                        statuses, targets, target_deltas
                    )
                )
                if reached:
                    break
                rate.sleep()
        if not reached:
            details = []
            for status in statuses:
                details.append(
                    "%s: Gazebo Δz=%.2f, LIO Δz=%.2f, PX4 z=%.2f"
                    % (
                        status.name,
                        truth.poses[status.name].position.z - status.reference_truth_z,
                        status.vision_pose.pose.position.z - status.reference_vision_z,
                        status.pose.pose.position.z,
                    )
                )
            raise RuntimeError("起飞超时；" + "; ".join(details))
        # Keep the relaxed land-detector threshold while the vehicle is
        # airborne.  In this SITL the normal 0.5 threshold can classify a
        # nearly stationary hovering vehicle as landed after a few seconds,
        # which removes thrust and makes the vehicle descend even though the
        # OFFBOARD position setpoint is still valid.  emergency_land() sets
        # the normal value before AUTO.LAND; do not restore it here.
        for status in statuses:
            status.watchdog_reached = True
        rospy.loginfo("成功：%s 均已 OFFBOARD、ARM，并到达起飞高度", ", ".join(UAVS))
        if args.no_ego_handover:
            # Pure SWARM-LIO -> PX4 EKF2 acceptance test. Keep publishing the
            # same position target and keep the safety watchdog active; do
            # not let an already-running planner replace the setpoint.
            rospy.loginfo(
                "纯 SWARM-LIO 定位保持 %.1f s；不交给 EGO-Swarm",
                args.hold_seconds,
            )
            hold_deadline = (
                None
                if args.hold_seconds <= 0
                else time.monotonic() + args.hold_seconds
            )
            while not rospy.is_shutdown() and (
                hold_deadline is None or time.monotonic() < hold_deadline
            ):
                for status, target in zip(statuses, targets):
                    status.pose_pub.publish(target)
                check_watchdog(
                    statuses,
                    truth,
                    clock,
                    target_deltas,
                    args.max_height_error,
                    args.max_lio_tilt,
                    expected_height_mode,
                )
                if any(s.state and s.state.mode != "OFFBOARD" for s in statuses):
                    raise RuntimeError("纯定位保持期间 OFFBOARD 意外退出")
                rate.sleep()
            emergency_land(statuses, "纯 SWARM-LIO 保持测试结束")
            rospy.loginfo("纯 SWARM-LIO 定位保持测试通过，已请求 AUTO.LAND")
            return
        # EGO may have been planning since the vehicle was on the ground.
        # Hand over only now, after all takeoff checks have passed.
        for status in statuses:
            status.ego_takeover_pub.publish(Bool(data=True))
        rospy.sleep(0.5)
        # EGO's LIO->PX4 adapter must become the sole pose-command publisher
        # after takeoff.  Keeping this takeoff publisher alive would create
        # two competing writers on /cmd_pose_enu and intermittently overwrite
        # EGO's XY setpoint.  The XTDrone bridge itself remains alive and the
        # watchdog below continues monitoring/landing the vehicles.
        for status in statuses:
            status.pose_pub.unregister()
        rospy.loginfo("起飞命令流已释放给 EGO-Swarm；安全看门狗继续运行，降落后按 Ctrl+C。")
        while not rospy.is_shutdown():
            if all(s.state and not s.state.armed for s in statuses):
                for status in statuses:
                    try:
                        status.restore_land_detector(3.0)
                    except (RuntimeError, rospy.ServiceException) as exc:
                        rospy.logwarn("%s 降落后恢复地检参数失败：%s", status.name, exc)
                rospy.loginfo("全部无人机已解锁，看门狗正常结束")
                return
            if any(s.state and s.state.mode == "AUTO.LAND" for s in statuses):
                rospy.loginfo("检测到 AUTO.LAND，起飞看门狗停止；PX4 接管降落")
                return
            unexpected = [
                "%s=%s" % (s.name, s.state.mode)
                for s in statuses
                if s.state and s.state.mode != "OFFBOARD"
            ]
            if unexpected:
                raise RuntimeError("飞行模式意外退出 OFFBOARD：" + ", ".join(unexpected))
            check_watchdog(
                statuses,
                truth,
                clock,
                target_deltas,
                args.max_height_error,
                args.max_lio_tilt,
                expected_height_mode,
            )
            rate.sleep()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        emergency_land(statuses, str(exc))
        raise
    finally:
        for process in bridge_processes:
            if process.poll() is None:
                process.terminate()
        for process in bridge_processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("一键起飞失败：%s", exc)
        sys.exit(1)
