#!/usr/bin/env python3
"""M4b: 用 SWARM-LIO 定位结果建 2D 占用栅格地图(坐标系 = LIO 世界 car0/world)。

订阅 /carN/lidar_slam/odom(体/IMU 在 LIO 世界的位姿) 与 /carN/cloud_registered
(已注册进 LIO 世界的稠密点云)。建图车行驶期间累计:
  - 命中: 高度带内点云点落入的 xy 格 +occ
  - 自由: 从每帧传感器原点(odom+imu->livox 偏移)向各命中点 Bresenham 扫过格 +free
结束时(收到 Ctrl-C / rosservice ~/save 或建图轮结束后 kill -INT)导出:
  <out>_pgm/.yaml (frame 即 carN/world 的坐标, resolution, origin=-extent/2)。
用法(建图车跑的时候):
  rosrun car3_control build_lio_2d_map.py _odom:=/car0/lidar_slam/odom
      _cloud:=/car0/cloud_registered _out:=/tmp/car3_lio/nesting_lio
      _extent:=14.0 _res:=0.05 _band_lo:=0.05 _band_hi:=1.6
kill -INT <pid> 后写文件。
"""
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_srvs.srv import Trigger, TriggerResponse

OFS = np.array([0.07125, 0.00161, 0.1789])  # livox 相对 IMU(lio 体) 偏移


class Lio2DMap:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom", "/car0/lidar_slam/odom")
        self.cloud_topic = rospy.get_param("~cloud", "/car0/cloud_registered")
        self.out = rospy.get_param("~out", "/tmp/car3_lio/nesting_lio")
        self.extent = float(rospy.get_param("~extent", 14.0))   # 半边长
        self.res = float(rospy.get_param("~res", 0.05))
        self.z_lo = float(rospy.get_param("~band_lo", 0.05))
        self.z_hi = float(rospy.get_param("~band_hi", 1.60))
        self.max_pts = int(rospy.get_param("~max_pts", 600))     # 每帧抽样上限
        self.n = int(round(2.0 * self.extent / self.res))
        self.off = self.n / 2
        self.occ = np.zeros((self.n, self.n), dtype=np.float32)
        self.free = np.zeros((self.n, self.n), dtype=np.float32)
        self.pose = np.eye(4)
        self.have_pose = False
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=5)
        self.srv = rospy.Service("~save", Trigger, self.save_cb)
        self.frame = rospy.get_param("~frame_id", "/car0/world").lstrip("/")
        rospy.loginfo("lio2dmap: %s + %s -> %s (extent %.0f res %.2f z[%.2f,%.2f])",
                      self.odom_topic, self.cloud_topic, self.out, self.extent, self.res, self.z_lo, self.z_hi)

    def odom_cb(self, m):
        q = m.pose.pose.orientation
        s = math.sin, math.cos
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = np.array([[s[1](yaw), -s[0](yaw), 0, m.pose.pose.position.x],
                              [s[0](yaw),  s[1](yaw), 0, m.pose.pose.position.y],
                              [0, 0, 1, m.pose.pose.position.z],
                              [0, 0, 0, 1]], dtype=np.float64)
        self.have_pose = True

    def cloud_cb(self, msg):
        if not self.have_pose:
            return
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        try:
            pts = np.array([(p[0], p[1], p[2]) for p in gen], dtype=np.float64)
        except Exception:
            return
        if len(pts) < 8:
            return
        keep = (pts[:, 2] >= self.z_lo) & (pts[:, 2] <= self.z_hi)
        pts = pts[keep]
        if len(pts) > self.max_pts:
            idx = np.random.choice(len(pts), self.max_pts, replace=False)
            pts = pts[idx]
        # 传感器原点(世界): pose(imu) + R_yaw*OFS
        ox = self.pose[0, 3] + self.pose[0, 0] * OFS[0] - self.pose[1, 0] * OFS[1]
        oy = self.pose[1, 3] + self.pose[1, 0] * OFS[0] + self.pose[0, 0] * OFS[1]
        x0 = (ox + self.extent) / self.res
        y0 = (self.extent - oy) / self.res
        xi = ((pts[:, 0] + self.extent) / self.res).astype(int)
        yi = ((self.extent - pts[:, 1]) / self.res).astype(int)
        ok = (xi >= 0) & (xi < self.n) & (yi >= 0) & (yi < self.n)
        # 命中(略去太近点)
        d2 = (pts[:, 0] - ox) ** 2 + (pts[:, 1] - oy) ** 2
        near = d2 < (0.3 * self.res) ** 2
        m = ok & ~near
        np.add.at(self.occ, (yi[m], xi[m]), 1.0)
        # 自由: 逐点 Bresenham 从传感器到命中前一格
        for (x1, y1, x2, y2) in zip(xi[m].astype(float), yi[m].astype(float),
                                    xi[m].astype(float), yi[m].astype(float)):
            self._ray_free(x0, y0, x1, y1)

    def _ray_free(self, x0, y0, x1, y1):
        dx = abs(x1 - x0); dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = int(round(x0)), int(round(y0))
        steps = 0
        while steps < 2000:
            if x == int(round(x1)) and y == int(round(y1)):
                break
            if 0 <= x < self.n and 0 <= y < self.n:
                self.free[y, x] += 1.0
            e2 = 2 * err
            if e2 >= dy:
                err += dy; x += sx
            if e2 <= dx:
                err += dx; y += sy
            steps += 1

    def save_cb(self, _req):
        self._write()
        return TriggerResponse(True, "saved %s.pgm" % self.out)

    def _write(self):
        occ = self.occ.copy()
        free = self.free.copy()
        # 占用度 p = occ/(occ+free); 障碍 = p 高, 自由 = 多次扫过且 p 低
        tot = occ + free
        with np.errstate(divide="ignore", invalid="ignore"):
            p = np.where(tot > 0, occ / np.maximum(tot, 1e-6), np.nan)
        occ_cell = p >= 0.60
        free_cell = (free >= 3.0) & (p < 0.10)
        # map_server 约定: 255=free, 0=occupied, 205=unknown
        img = np.full((self.n, self.n), 205, dtype=np.uint8)
        img[free_cell] = 254
        img[occ_cell] = 0
        # PGM 用 P5 写入(左上为 -y 最大即图像行0 = 世界 y=max)
        hdr = "P5\n%d %d\n255\n" % (self.n, self.n)
        with open(self.out + ".pgm", "wb") as f:
            f.write(hdr.encode())
            f.write(img.astype(np.uint8).tobytes())
        with open(self.out + ".yaml", "w") as f:
            f.write("image: %s.pgm\n" % self.out.split("/")[-1])
            f.write("resolution: %.4f\n" % self.res)
            f.write("origin: [%.4f, %.4f, 0.0000]\n" % (-self.extent, -self.extent))
            f.write("negate: 0\noccupied_thresh: 0.45\nfree_thresh: 0.25\n")
        rospy.loginfo("map saved: %s (occ=%d free=%d unknown=%d)", self.out,
                      int(occ_cell.sum()), int(free_cell.sum()),
                      self.n * self.n - int(occ_cell.sum()) - int(free_cell.sum()))


if __name__ == "__main__":
    rospy.init_node("build_lio_2d_map")
    node = Lio2DMap()
    rospy.on_shutdown(node._write)
    rospy.spin()
