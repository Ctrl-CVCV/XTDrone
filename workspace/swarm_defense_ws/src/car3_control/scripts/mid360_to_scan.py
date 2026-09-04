#!/usr/bin/env python3
"""M4: mid360 CustomMsg -> 2D LaserScan（平装 mid360，取传感器系水平 ±band 带）。

把 SWARM-LIO 车侧 /carN/livox/lidar（livox_ros_driver2/CustomMsg）按每帧全 360°
投影成 /carN/scan（frame = carN/livox_link），喂给 move_base/AMCL 那套 2D 导航。
只保留 |z| < band 的水平点(排除地板/自车身)，range_min 切掉紧贴雷达的自部件。
用法: rosrun car3_control mid360_to_scan.py _lidar_topic:=/car0/livox/lidar
                                          _scan_topic:=/car0/scan
"""
import math
import numpy as np
import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import LaserScan


class Mid360ToScan:
    def __init__(self):
        self.lidar_topic = rospy.get_param("~lidar_topic", "/car0/livox/lidar")
        self.scan_topic = rospy.get_param("~scan_topic", "/car0/scan")
        self.frame = rospy.get_param("~frame_id", "/car0/livox_link").lstrip("/")
        # z 带: 墙面回波中位数在传感器上方 ~0.75m(p25~-0.1, p75~1.6),
        # 取 [-0.1, 1.0] 抓墙, 地板(z~-0.27)与近距自车体用 range_min 切掉。
        self.z_lo = float(rospy.get_param("~z_lo", -0.10))
        self.z_hi = float(rospy.get_param("~z_hi", 1.0))
        self.range_min = float(rospy.get_param("~range_min", 0.90))
        self.range_max = float(rospy.get_param("~range_max", 25.0))
        self.beams = int(rospy.get_param("~beams", 1440))
        # mid360 逐帧非重复扫描很稀; 滚动累积 ~window 秒的点再投 bin(静止/缓动可用)
        self.window = float(rospy.get_param("~window_s", 0.30))
        self._buf = []  # (stamp, n, xyz)
        self.pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=5)
        rospy.Subscriber(self.lidar_topic, CustomMsg, self.cb, queue_size=4)
        rospy.loginfo("mid360_to_scan: %s -> %s (frame %s, z [%.2f,%.2f], win %.2fs)",
                      self.lidar_topic, self.scan_topic, self.frame, self.z_lo, self.z_hi, self.window)

    def cb(self, msg):
        n = min(msg.point_num, len(msg.points))
        if n == 0:
            return
        pts = [(p.x, p.y, p.z) for p in msg.points[:n]]
        xyz = np.array(pts, dtype=np.float32).reshape(-1, 3)
        now = msg.header.stamp
        self._buf.append((now, xyz))
        # 丢掉太老的帧
        self._buf = [(st, a) for st, a in self._buf
                     if (now - st).to_sec() <= self.window]
        parts = [a for _, a in self._buf]
        xyz = np.concatenate(parts, axis=0) if len(parts) > 1 else parts[0]
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        # z 带(排除地板与大部分自车身) + 有效量程
        keep = (z >= self.z_lo) & (z <= self.z_hi)
        r2 = x * x + y * y
        keep &= (r2 >= self.range_min * self.range_min) & (r2 <= self.range_max * self.range_max)
        if not keep.any():
            return
        ang = np.arctan2(y[keep], x[keep])          # [-pi, pi]
        rng = np.sqrt(r2[keep])
        # 每 bin 取最近点
        bin_i = np.floor((ang + math.pi) / (2.0 * math.pi / self.beams)).astype(int)
        bin_i[bin_i >= self.beams] = self.beams - 1
        order = np.argsort(bin_i, kind="stable")
        bin_i = bin_i[order]; rng = rng[order]
        uniq, idx = np.unique(bin_i, return_index=True)   # first (smallest range) per bin
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.beams
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        ranges = np.full(self.beams, math.inf, dtype=np.float32)
        ranges[uniq] = rng[idx]
        scan.ranges = ranges.tolist()
        scan.intensities = [0.0] * self.beams
        self.pub.publish(scan)


if __name__ == "__main__":
    rospy.init_node("mid360_to_scan")
    Mid360ToScan()
    rospy.spin()
