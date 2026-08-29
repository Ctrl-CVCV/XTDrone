#!/usr/bin/env python3
# 抓取一帧 /gimbal_camera/image_raw 并分析，验证相机话题输出真实画面（非黑屏）
# 输出: 亮度均值/标准差、彩色像素统计（世界中有红/绿/蓝/黄等彩色方块）
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

rospy.init_node('gimbal_cam_check')
msg = rospy.wait_for_message('/gimbal_camera/image_raw', Image, timeout=15)
arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
mean = arr.mean()
std = arr.std()
r, g, b = arr[:, :, 0].astype(int), arr[:, :, 1].astype(int), arr[:, :, 2].astype(int)
colored = int(((r > 120) | (g > 120) | (b > 120)).sum())
print('frame %dx%d mean=%.1f std=%.1f colored_pixels=%d' %
      (msg.width, msg.height, mean, std, colored))
ok = mean > 15 and std > 25 and colored > 500
print('CAMERA_CHECK %s' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
