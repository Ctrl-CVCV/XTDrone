#!/usr/bin/env python3
"""M4c: 用 LIO odom 位姿 + 稠密 2D LaserScan 做经典光投影建 2D 占用栅格图。

建图车(开 mid360_to_scan 近距版 /car0/scan_map + SWARM-LIO /car0/lidar_slam/odom)
慢速行驶/原地转圈期间:
  - 每束有限 beam: 命中格(世界 hit point) occ += 1
  - 从传感器原点(odom+imu->livox 偏移)向命中格 Bresenham 扫过格 free += 1
导出坐标系 = car0/world(LIO 世界), resolution, origin=-extent/2。用法:
  rosrun car3_control build_scan_2d_map.py _odom:=/car0/lidar_slam/odom
      _scan:=/car0/scan_map _out:=/tmp/car3_lio/nesting_scan _extent:=14.0 _res:=0.05
Ctrl-C / rosservice ~/save 后写文件。
"""
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse

OFS = np.array([0.07125, 0.00161, 0.1789])  # livox 相对 IMU(lio 体) 偏移


class Scan2DMap:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom", "/car0/lidar_slam/odom")
        self.scan_topic = rospy.get_param("~scan", "/car0/scan_map")
        self.out = rospy.get_param("~out", "/tmp/car3_lio/nesting_scan")
        self.extent = float(rospy.get_param("~extent", 14.0))
        self.res = float(rospy.get_param("~res", 0.05))
        self.min_r = float(rospy.get_param("~min_r", 0.40))   # 自噪/太近丢弃
        self.n = int(round(2.0 * self.extent / self.res))
        self.occ = np.zeros((self.n, self.n), dtype=np.float32)
        self.free = np.zeros((self.n, self.n), dtype=np.float32)
        self.pose = np.eye(4)
        self.have_pose = False
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=5)
        self.srv = rospy.Service("~save", Trigger, self.save_cb)
        rospy.loginfo("scan2dmap: %s + %s -> %s (extent %.0f res %.2f min_r %.2f)",
                      self.odom_topic, self.scan_topic, self.out, self.extent, self.res, self.min_r)

    def odom_cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        self.pose = np.array([[c, -s, 0, m.pose.pose.position.x],
                              [s,  c, 0, m.pose.pose.position.y],
                              [0,  0, 1, m.pose.pose.position.z],
                              [0,  0, 0, 1]], dtype=np.float64)
        self.have_pose = True

    def scan_cb(self, m):
        if not self.have_pose:
            return
        n = len(m.ranges)
        ang = m.angle_min + np.arange(n) * m.angle_increment
        rng = np.asarray(m.ranges, dtype=np.float64)
        ok = np.isfinite(rng) & (rng >= self.min_r)
        if not ok.any():
            return
        # 传感器原点(世界)
        ox = self.pose[0, 3] + self.pose[0, 0] * OFS[0] - self.pose[1, 0] * OFS[1]
        oy = self.pose[1, 3] + self.pose[1, 0] * OFS[0] + self.pose[0, 0] * OFS[1]
        yaw = math.atan2(self.pose[1, 0], self.pose[0, 0])
        a = yaw + ang[ok]
        rr = rng[ok]
        hx = ox + rr * np.cos(a)
        hy = oy + rr * np.sin(a)
        x0 = (ox + self.extent) / self.res
        y0 = (self.extent - oy) / self.res
        xi = ((hx + self.extent) / self.res).astype(int)
        yi = ((self.extent - hy) / self.res).astype(int)
        m2 = (xi >= 0) & (xi < self.n) & (yi >= 0) & (yi < self.n)
        np.add.at(self.occ, (yi[m2], xi[m2]), 1.0)
        for (u, v) in zip(xi[m2].astype(float), yi[m2].astype(float)):
            self._ray_free(x0, y0, u, v)

    def _ray_free(self, x0, y0, x1, y1):
        dx = abs(x1 - x0); dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = int(round(x0)), int(round(y0))
        steps = 0
        while steps < 4000:
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
        occ, free = self.occ, self.free
        tot = occ + free
        with np.errstate(divide="ignore", invalid="ignore"):
            p = np.where(tot > 0, occ / np.maximum(tot, 1e-6), np.nan)
        occ_cell = p >= 0.45                 # 光投影命中端, 通常 occ 高 free 0
        free_cell = (free >= 2.0) & (p < 0.20)
        img = np.full((self.n, self.n), 205, dtype=np.uint8)
        img[free_cell] = 254
        img[occ_cell] = 0
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
    rospy.init_node("build_scan_2d_map")
    node = Scan2DMap()
    rospy.on_shutdown(node._write)
    rospy.spin()
