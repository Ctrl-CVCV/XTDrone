#!/usr/bin/env python3
"""M3 验证: SWARM-LIO 世界对齐 相对 Gazebo 真值的一致性。

收集若干采样点, 每个点同时取:
  - Gazebo model_states 里 model0/model1 位姿(→ +imu 偏移 = LIO 锚定体/IMU 位姿)
  - 两 agent 的 LIO odom(W_i_si: 体在各自 LIO 世界中的位姿)
  - map_alignment 已冻结发布的 child/world->parent/world TF(仅用于比对)

由此逐样本计算:
  g_w_i = G_si * inv(W_i_si)   (各 LIO 世界原点在 Gazebo 中的位姿)
指标:
  1) drift_x_w0/x_w1: g_w0/g_w1 平移随时间 std(越小=LIO 世界越稳, 静态下应 < ~2cm)
  2) T_spread: 逐样本 candidate=inv(g_w0)*g_w1 相对冻结 TF 的平移/转角极差
  3) residual: 冻结 TF * W1_s1(imu1 in car1/world) 应等于 Gazebo 真值 imu1 在
     car0/world 中的位姿(用 g_w0 折算)。取最大偏差。
用法(rosrun, 需 env.sh + LIO 双 odom + 一个 LIO 世界对齐已冻结):
  rosrun car3_swarm m3_verify.py  (或直接 python3, 见 __main__)
参数 ~model0 ~model1 ~odom0_topic ~odom1_topic ~imu_offset0 ~imu_offset1 ~seconds
"""
import sys
import threading
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def pose_mat(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    m = quaternion_matrix(q)
    m[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def transform_mat(tf):
    q = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
    m = quaternion_matrix(q)
    m[0:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return m


def body_offset(body, off):
    if not np.any(off):
        return body
    m = body.copy()
    m[0:3, 3] = m[0:3, 3] + np.matmul(body[0:3, 0:3], off)
    return m


def rel_err(m_rel):
    """最大平移/角偏差 from identity."""
    t = m_rel[0:3, 3]
    q = quaternion_from_matrix(m_rel)
    angle = 2.0 * np.arccos(np.clip(abs(q[3]), -1.0, 1.0))
    return np.linalg.norm(t), np.degrees(angle)


class M3Verify:
    def __init__(self):
        self.model0 = rospy.get_param("~model0", "car0")
        self.model1 = rospy.get_param("~model1", "car1")
        self.odom0_topic = rospy.get_param("~odom0_topic", "/car0/lidar_slam/odom")
        self.odom1_topic = rospy.get_param("~odom1_topic", "/car1/lidar_slam/odom")
        self.off0 = np.array([float(v) for v in rospy.get_param("~imu_offset0", "-0.07125 -0.00161 0.0806").split()])
        self.off1 = np.array([float(v) for v in rospy.get_param("~imu_offset1", "-0.07125 -0.00161 0.0806").split()])
        self.seconds = float(rospy.get_param("~seconds", 12.0))
        self.lock = threading.Lock()
        self.g0 = self.g1 = self.od0 = self.od1 = None
        self.samples = []
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_cb, queue_size=1)
        rospy.Subscriber(self.odom0_topic, Odometry, self.odom0_cb, queue_size=5)
        rospy.Subscriber(self.odom1_topic, Odometry, self.odom1_cb, queue_size=5)
        rospy.Subscriber("/map_alignment/transform", TransformStamped, self.align_cb, queue_size=5)
        self.align = None

    def model_cb(self, msg):
        try:
            i0, i1 = msg.name.index(self.model0), msg.name.index(self.model1)
        except ValueError:
            return
        with self.lock:
            self.g0 = body_offset(pose_mat(msg.pose[i0]), self.off0)
            self.g1 = body_offset(pose_mat(msg.pose[i1]), self.off1)

    def odom0_cb(self, m):
        with self.lock:
            self.od0 = pose_mat(m.pose.pose)

    def odom1_cb(self, m):
        with self.lock:
            self.od1 = pose_mat(m.pose.pose)

    def align_cb(self, m):
        with self.lock:
            self.align = transform_mat(m.transform)

    def run(self):
        rate = rospy.Rate(10)
        t0 = rospy.Time.now()
        # warm-up: wait until every source has delivered at least once (LIO may
        # still be streaming in its first seconds), then start the timed window.
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 20.0:
            with self.lock:
                ready = all(v is not None for v in (self.g0, self.g1, self.od0, self.od1))
            if ready:
                break
            rate.sleep()
        with self.lock:
            self.g0 = self.g1 = self.od0 = self.od1 = None
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < self.seconds:
            with self.lock:
                if all(v is not None for v in (self.g0, self.g1, self.od0, self.od1)):
                    self.samples.append((self.g0.copy(), self.g1.copy(), self.od0.copy(), self.od1.copy()))
                    self.g0 = self.g1 = self.od0 = self.od1 = None
            rate.sleep()
        n = len(self.samples)
        if n < 5:
            rospy.logerr("too few samples (%d)" % n)
        else:
            self.report(n)

    def report(self, n):
        g_w0s = [np.matmul(g0, np.linalg.inv(o0)) for g0, _, o0, _ in self.samples]
        g_w1s = [np.matmul(g1, np.linalg.inv(o1)) for _, g1, _, o1 in self.samples]
        t0_arr = np.array([m[0:3, 3] for m in g_w0s]); t1_arr = np.array([m[0:3, 3] for m in g_w1s])
        q0s = np.array([quaternion_from_matrix(m) for m in g_w0s])
        q1s = np.array([quaternion_from_matrix(m) for m in g_w1s])

        def rot_drift(qs):
            qm = np.mean(qs, axis=0)
            qm /= np.linalg.norm(qm)
            for i in range(len(qs)):
                if np.dot(qs[i], qm) < 0:
                    qs[i] = -qs[i]
            angs = np.degrees(2 * np.arccos(np.clip(np.abs(np.einsum("ij,j->i", qs, qm)), -1, 1)))
            return angs.max(), angs.std()
        r0 = rot_drift(q0s)
        r1 = rot_drift(q1s)
        print("samples: %d" % n)
        print("LIO world0 origin drift: trans x/y/z std (m)=%.3f/%.3f/%.3f  rot max/std(deg)=%.2f/%.2f"
              % (t0_arr[:, 0].std(), t0_arr[:, 1].std(), t0_arr[:, 2].std(), r0[0], r0[1]))
        print("LIO world1 origin drift: trans x/y/z std (m)=%.3f/%.3f/%.3f  rot max/std(deg)=%.2f/%.2f"
              % (t1_arr[:, 0].std(), t1_arr[:, 1].std(), t1_arr[:, 2].std(), r1[0], r1[1]))
        if self.align is not None:
            d_t, d_a = [], []
            for g0, g1, o0, o1 in self.samples:
                cand = np.matmul(np.linalg.inv(np.matmul(g0, np.linalg.inv(o0))),
                                 np.matmul(g1, np.linalg.inv(o1)))
                err = rel_err(np.matmul(np.linalg.inv(cand), self.align))
                d_t.append(err[0]); d_a.append(err[1])
            print("frozen-TF vs per-sample candidate: t_max=%.3f t_std=%.3f  rot_max=%.2fdeg"
                  % (max(d_t), np.std(d_t), max(d_a)))
        if self.align is not None:
            res = []
            for g0, g1, o0, o1 in self.samples:
                g_w0 = np.matmul(g0, np.linalg.inv(o0))
                pred = np.matmul(self.align, o1)
                truth = np.matmul(np.linalg.inv(g_w0), g1)
                err = rel_err(np.matmul(np.linalg.inv(pred), truth))
                res.append(err)
            res = np.array(res)
            print("cross-frame residual (frozen TF aligns car1 odom to gazebo truth): "
                  "t_mean=%.3f t_max=%.3f  rot_mean=%.2fdeg" % (res[:, 0].mean(), res[:, 0].max(), res[:, 1].mean()))
        print("DONE")


if __name__ == "__main__":
    rospy.init_node("m3_verify")
    ver = M3Verify()
    try:
        ver.run()
    except rospy.ROSInterruptException:
        if len(ver.samples) >= 5:
            ver.report(len(ver.samples))
        else:
            rospy.logerr("interrupted with too few samples (%d)" % len(ver.samples))
