#!/usr/bin/env python3
"""建图行驶(LIO 友好慢速版): 用 /car0/odom(gazebo 真值) 在"口袋"内巡航。

背景: 车出生点(2.3,2.4)被 nesting_room 内墙围成口袋:
  北墙 y in [2.97,3.12](x 0.48..3.0), 东墙 x in [2.86,3.01](y -0.61..2.97)。
经验: 原地快速自转(0.5rad/s)会让 SWARM-LIO 世界位姿漂移 1m+, 造成建图糊。
故用"连续小半径圆周巡航"(vx>0 同时 wz 很小, 半径 ~0.55m)整圈缓慢扫过 360 方位,
全程都在平移, 无原地快转。先 go_slow 到口袋中部, 再巡航 ~50s。开 build_scan_2d_map.py 记录。
"""
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ORBIT_CENTER = (1.7, 1.85)   # 口袋中部(离各墙 >0.6)
ORBIT_R = 0.55               # 巡航半径 -> 轨迹 x in [1.15,2.25] y in [1.3,2.4]
V_LIN = 0.10
W_CRUISE = V_LIN / ORBIT_R   # 0.18 rad/s
DUR = 52.0                   # 巡航秒数(≈1 整圈)


def norm(yaw):
    while yaw > math.pi: yaw -= 2 * math.pi
    while yaw < -math.pi: yaw += 2 * math.pi
    return yaw


class Driver:
    def __init__(self):
        self.pose = None
        rospy.Subscriber("/car0/odom", Odometry, self.cb)
        self.pub = rospy.Publisher("/car0/cmd_vel", Twist, queue_size=1)
        self.tw = Twist()

    def cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def setvel(self, vx=0.0, vy=0.0, wz=0.0):
        self.tw.linear.x = vx; self.tw.linear.y = vy; self.tw.angular.z = wz

    def move(self, vx=0.0, vy=0.0, wz=0.0, dur=0.0):
        self.setvel(vx, vy, wz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < dur:
            self.pub.publish(self.tw)
            rospy.sleep(0.02)
        self.setvel()

    def look(self, dur=6.0, wz=0.08):
        """极慢小角度环视(≈28°/6s), 补方位且不扰动 LIO."""
        self.move(wz=wz, dur=dur)
        self.move()

    def go_slow(self, target, lin=0.22):
        """慢速逼近: 逐步转向+前进, 到位即停(不做原地快转)."""
        tx, ty = target
        while not rospy.is_shutdown():
            p = self.pose
            if p is None:
                rospy.sleep(0.05); continue
            x, y, yaw = p
            dx, dy = tx - x, ty - y
            if math.hypot(dx, dy) < 0.12:
                break
            want = math.atan2(dy, dx)
            d = norm(want - yaw)
            if abs(d) > 0.08:
                # 转到 <0.2 rad/s 量级, 避免 LIO 漂移
                self.move(wz=0.3 * (1.0 if d > 0 else -1.0), dur=0.12)
            else:
                # 边慢转边前进(纯 forward)
                self.move(vx=lin * (1.0 if abs(d) < 0.5 else 0.0), wz=0.3 * d, dur=0.12)
        self.move()

    def cruise(self, dur=DUR):
        """口袋中心小圆巡航: 连续平移+缓慢转, 扫 360°."""
        print("cruise r=%.2f v=%.2f w=%.3f for %.0fs" % (ORBIT_R, V_LIN, W_CRUISE, dur))
        self.setvel(V_LIN, 0.0, -W_CRUISE)   # 顺时针
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < dur:
            self.pub.publish(self.tw)
            # 万一巡航把车带出安全框则暂停(撞墙保护)
            if self.pose is not None:
                px, py, _ = self.pose
                if not (0.8 <= px <= 2.7 and 0.9 <= py <= 2.85):
                    print("edge! abort cruise at (%.2f,%.2f)" % (px, py))
                    break
            rospy.sleep(0.02)
        self.move()


if __name__ == "__main__":
    rospy.init_node("mapping_drive")
    d = Driver()
    rospy.sleep(0.5)
    # 口袋内安全矩形四角 + 中心 + 两条对角, 直行往返, 每角极慢环视
    way = [(1.3, 2.5), (2.3, 2.5), (2.3, 1.3), (1.3, 1.3),
           (1.7, 1.85), (2.2, 2.3), (1.2, 1.4), (1.7, 1.85)]
    for i, p in enumerate(way):
        print("leg", i, p)
        d.go_slow(p)
        if i % 2 == 0:
            d.look(5.0, 0.08)      # 隔点环视一下, 换朝向
    print("done")
