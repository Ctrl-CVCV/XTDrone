#!/usr/bin/env python3
"""Debug: one camera above the car aimed down; confirm red marker renders."""
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from sensor_msgs.msg import Image

from capture_car_views import quat_from_aim

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "camera_view.sdf")


def red_centroid(rgb):
    r = rgb[:, :, 0].astype(np.int32)
    g = rgb[:, :, 1].astype(np.int32)
    b = rgb[:, :, 2].astype(np.int32)
    mask = (r > 90) & (g < r * 0.5) & (b < r * 0.5)
    n = int(mask.sum())
    if n == 0:
        return None
    ys, xs = np.nonzero(mask)
    return (round(float(xs.mean()) - rgb.shape[1] / 2.0, 1),
            round(float(ys.mean()) - rgb.shape[0] / 2.0, 1), n)


def main():
    rospy.init_node('dbg_car_cam', anonymous=True)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    dele = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    name = 'car_dbg_cam'
    sdf = open(TEMPLATE).read().replace('@{MODEL_NAME}', name).replace('@{CAMERA_NAME}', 'cam')
    # camera 1.2m above car body center, looking straight down at the car
    w, x, y, z = quat_from_aim([0.0, 0.0, -1.0])
    pose = Pose(position=Point(0.0, 0.0, 1.35),
                orientation=Quaternion(x=x, y=y, z=z, w=w))
    resp = spawn(model_name=name, model_xml=sdf, robot_namespace='',
                 initial_pose=pose, reference_frame='world')
    print('spawn success=%s' % resp.success)
    try:
        img = rospy.wait_for_message('/%s/cam/image_raw' % name, Image, timeout=10)
        print('frame %dx%d %s len=%d' % (img.width, img.height, img.encoding, len(img.data)))
        arr = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3)
        rc = red_centroid(arr)
        print('red:', rc if rc else 'NONE')
    except rospy.ROSException:
        print('NO FRAME')
    try:
        dele(model_name=name)
    except rospy.ServiceException as e:
        print('delete failed: %s' % e)


if __name__ == '__main__':
    main()
