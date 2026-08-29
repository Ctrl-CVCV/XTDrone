#!/usr/bin/env python3
# 对比相机话题帧与 RViz 窗口截图，验证 RViz 正在显示相机画面（调试用）
import sys

import numpy as np
import rospy
from PIL import Image
from sensor_msgs.msg import Image as ROSImage


def colored_px(arr):
    r, g, b = arr[:, :, 0].astype(int), arr[:, :, 1].astype(int), arr[:, :, 2].astype(int)
    return int(((r > 120) | (g > 120) | (b > 120)).sum())


rospy.init_node('gcam_analyze', anonymous=True)
msg = rospy.wait_for_message('/gimbal_camera/image_raw', ROSImage, timeout=10)
cam = np.frombuffer(msg.data, dtype=np.uint8).reshape(480, 640, 3)
Image.fromarray(cam).save('/tmp/gcam_frame.png')
print('CAM frame mean=%.1f std=%.1f colored=%d' % (cam.mean(), cam.std(), colored_px(cam)))

shot = sys.argv[1] if len(sys.argv) > 1 else '/tmp/gcam_shot2.png'
w = np.asarray(Image.open(shot).convert('RGB'))[:, 2200:5120, :]
h, wd, _ = w.shape
print('RViz window %s mean=%.1f std=%.1f colored=%d' % (w.shape, w.mean(), w.std(), colored_px(w)))
view = w[:, 350:, :]
print('RViz 3D view mean=%.1f std=%.1f colored=%d' % (view.mean(), view.std(), colored_px(view)))
