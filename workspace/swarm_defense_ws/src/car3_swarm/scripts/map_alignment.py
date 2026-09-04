#!/usr/bin/env python3
"""Generalized SWARM-LIO map-frame alignment (2 agents).

Generalizes swarm_lio/scripts/dual_map_alignment.py so the two LIO odometry
topics are parameters (the original hardcodes /quadN/lidar_slam/odom, which
cannot serve a ground car, e.g. car0).  Frozen once from Gazebo ground truth
in simulation_truth mode, then republished at ~rate:

  child LIO world  --(child_frame)-->  is expressed in -->  parent LIO world

i.e. TF parent_frame -> child_frame = inv(g_w0) * g_w1, where g_wi is the pose
of LIO world i in the shared Gazebo frame.

Use for the ground car exactly like the dual-UAV case: point parent at the
SWARM-LIO master LIO world (e.g. quad0/world), child at the car's own LIO
world (e.g. car0/world), model0/model1 at their Gazebo model names.
"""
import math
import threading
import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from swarm_msgs.msg import GlobalExtrinsicStatus
from tf.transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix


class MapAlignment:
    def __init__(self):
        self.source = rospy.get_param("~source", "simulation_truth")
        self.parent_frame = rospy.get_param("~parent_frame", "quad0/world").lstrip("/")
        self.child_frame = rospy.get_param("~child_frame", "car0/world").lstrip("/")
        self.model0 = rospy.get_param("~model0", "iris_0")
        self.model1 = rospy.get_param("~model1", "car0")
        self.odom0_topic = rospy.get_param("~odom0_topic", "/quad0/lidar_slam/odom")
        self.odom1_topic = rospy.get_param("~odom1_topic", "/car0/lidar_slam/odom")
        # SWARM-LIO anchors its local world at the IMU frame, which sits a fixed
        # body-frame offset away from the Gazebo *model* origin (model_states only
        # exposes the model root).  Provide ~imu_offsetN as "x y z" (body frame) so
        # the derived LIO-world<->Gazebo transform is exact; default 0 keeps the
        # original dual_map_alignment behavior.
        self.imu_offset0 = self._offset_param("~imu_offset0")
        self.imu_offset1 = self._offset_param("~imu_offset1")
        self.samples_required = max(1, int(rospy.get_param("~samples", 30)))
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.lock = threading.Lock()
        self.model_poses, self.samples = {}, []
        self.odom0 = self.odom1 = self.transform = None
        self.reported_wait = False
        self.br = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher("~transform", TransformStamped, queue_size=1, latch=True)
        rospy.Service("~recalibrate", Trigger, self.recalibrate)
        if self.source == "simulation_truth":
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_cb, queue_size=1)
            rospy.Subscriber(self.odom0_topic, Odometry, self.odom0_cb, queue_size=5)
            rospy.Subscriber(self.odom1_topic, Odometry, self.odom1_cb, queue_size=5)
        elif self.source == "swarm":
            self.swarm_topic = rospy.get_param("~swarm_topic", "/quad0/global_extrinsic_to_teammate")
            self.swarm_id = int(rospy.get_param("~swarm_id", 0))
            self.teammate_id = int(rospy.get_param("~teammate_id", 1))
            rospy.Subscriber(self.swarm_topic, GlobalExtrinsicStatus, self.swarm_cb, queue_size=10)
        else:
            raise rospy.ROSInitException("~source must be simulation_truth or swarm")
        rospy.Timer(rospy.Duration(1.0 / max(self.rate, 1.0)), self.timer_cb)
        rospy.loginfo("map alignment: source=%s, %s -> %s (odom %s | %s)",
                      self.source, self.parent_frame, self.child_frame, self.odom0_topic, self.odom1_topic)

    @staticmethod
    def _offset_param(name):
        raw = rospy.get_param(name, None)
        if raw is None:
            return np.zeros(3)
        vals = [float(v) for v in str(raw).split()]
        if len(vals) != 3:
            raise rospy.ROSInitException("%s must be 'x y z'" % name)
        return np.array(vals)

    def model_with_imu(self, body_m, offset):
        if not np.any(offset):
            return body_m
        m = body_m.copy()
        m[0:3, 3] = m[0:3, 3] + np.matmul(body_m[0:3, 0:3], offset)
        return m

    @staticmethod
    def pose_matrix(pose):
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        m = quaternion_matrix(q)
        m[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return m

    @staticmethod
    def average_transforms(samples):
        t = np.array([m[0:3, 3] for m in samples])
        qs, ref = [], None
        for m in samples:
            q = quaternion_from_matrix(m)
            if ref is None:
                ref = q
            elif np.dot(q, ref) < 0.0:
                q = -q
            qs.append(q)
        q = np.mean(np.array(qs), axis=0)
        q /= np.linalg.norm(q)
        result = quaternion_matrix(q)
        result[0:3, 3] = np.mean(t, axis=0)
        return result

    def model_cb(self, msg):
        try:
            i0, i1 = msg.name.index(self.model0), msg.name.index(self.model1)
        except ValueError:
            rospy.logwarn_throttle(5.0, "waiting for Gazebo models %s and %s" % (self.model0, self.model1))
            return
        with self.lock:
            self.model_poses[self.model0] = self.pose_matrix(msg.pose[i0])
            self.model_poses[self.model1] = self.pose_matrix(msg.pose[i1])

    def odom0_cb(self, msg):
        with self.lock:
            self.odom0 = self.pose_matrix(msg.pose.pose)

    def odom1_cb(self, msg):
        with self.lock:
            self.odom1 = self.pose_matrix(msg.pose.pose)

    def swarm_cb(self, msg):
        if msg.drone_id != self.swarm_id:
            return
        for ext in msg.extrinsic:
            if ext.teammate_id == self.teammate_id:
                rpy = [math.radians(float(v)) for v in ext.rot_deg]
                m = euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")
                m[0:3, 3] = np.array(ext.trans, dtype=float)
                with self.lock:
                    self.transform = m
                rospy.loginfo_throttle(5.0, "using SWARM-LIO global extrinsic: t=[%.3f %.3f %.3f]" % tuple(m[0:3, 3]))
                return

    def collect_truth_sample(self):
        with self.lock:
            if self.transform is not None:
                return
            ready = (self.model0 in self.model_poses and self.model1 in self.model_poses
                     and self.odom0 is not None and self.odom1 is not None)
            if not ready:
                if not self.reported_wait:
                    rospy.loginfo("waiting for Gazebo poses (%s,%s) and both LIO odometry topics (%s, %s)",
                                  self.model0, self.model1, self.odom0_topic, self.odom1_topic)
                    self.reported_wait = True
                return
            g_b0, g_b1 = self.model_poses[self.model0].copy(), self.model_poses[self.model1].copy()
            w0_b0, w1_b1 = self.odom0.copy(), self.odom1.copy()
        g_s0 = self.model_with_imu(g_b0, self.imu_offset0)
        g_s1 = self.model_with_imu(g_b1, self.imu_offset1)
        g_w0 = np.matmul(g_s0, np.linalg.inv(w0_b0))
        g_w1 = np.matmul(g_s1, np.linalg.inv(w1_b1))
        candidate = np.matmul(np.linalg.inv(g_w0), g_w1)
        if not np.all(np.isfinite(candidate)):
            return
        with self.lock:
            self.samples.append(candidate)
            count = len(self.samples)
            if count >= self.samples_required:
                self.transform = self.average_transforms(self.samples)
                t, q = self.transform[0:3, 3], quaternion_from_matrix(self.transform)
                rospy.loginfo("map calibration complete (%d samples): t=[%.4f %.4f %.4f], q=[%.5f %.5f %.5f %.5f]",
                              count, t[0], t[1], t[2], q[0], q[1], q[2], q[3])
            else:
                rospy.loginfo_throttle(1.0, "collecting map alignment samples: %d/%d" % (count, self.samples_required))

    def make_msg(self, m):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = rospy.Time.now(), self.parent_frame, self.child_frame
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = m[0, 3], m[1, 3], m[2, 3]
        q = quaternion_from_matrix(m)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        return msg

    def timer_cb(self, _event):
        if self.source == "simulation_truth":
            self.collect_truth_sample()
        with self.lock:
            m = None if self.transform is None else self.transform.copy()
        if m is not None:
            msg = self.make_msg(m)
            self.br.sendTransform(msg)
            self.pub.publish(msg)

    def recalibrate(self, _request):
        if self.source != "simulation_truth":
            return TriggerResponse(False, "only available in simulation_truth mode")
        with self.lock:
            self.samples, self.transform, self.reported_wait = [], None, False
        return TriggerResponse(True, "map alignment samples cleared")


if __name__ == "__main__":
    rospy.init_node("map_alignment")
    MapAlignment()
    rospy.spin()
