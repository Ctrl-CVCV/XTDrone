#!/usr/bin/env python3
"""Debug: several cameras around the car, report red-marker visibility each."""
import os

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from sensor_msgs.msg import Image

from capture_car_views import quat_from_aim, red_centroid

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "camera_view.sdf")

VIEWS = [
    ('top',       (0.00, 0.00, 1.35), (0.0, 0.0, 0.1445)),
    ('side_front',(0.80, 0.00, 0.25), (0.0, 0.0, 0.1445)),
    ('side_y',    (0.00, 0.80, 0.25), (0.0, 0.0, 0.1445)),
    ('slant',     (0.60, 0.60, 0.60), (0.0, 0.0, 0.1445)),
]


def main():
    rospy.init_node('dbg_multi', anonymous=True)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    dele = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for label, cam, aim in VIEWS:
        name = 'dbg_%s' % label
        sdf = open(TEMPLATE).read().replace('@{MODEL_NAME}', name).replace('@{CAMERA_NAME}', 'cam')
        fwd = [aim[0]-cam[0], aim[1]-cam[1], aim[2]-cam[2]]
        w, x, y, z = quat_from_aim(fwd)
        resp = spawn(model_name=name, model_xml=sdf, robot_namespace='',
                     initial_pose=Pose(position=Point(*cam),
                                       orientation=Quaternion(x=x, y=y, z=z, w=w)),
                     reference_frame='world')
        if not resp.success:
            print('%-12s spawn FAILED' % label)
            continue
        try:
            img = rospy.wait_for_message('/%s/cam/image_raw' % name, Image, timeout=6)
            arr = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3)
            rc = red_centroid(arr)
            print('%-12s red: %s' % (label, rc if rc else 'NONE'))
        except rospy.ROSException:
            print('%-12s NO FRAME' % label)
        dele(model_name=name)
        rospy.sleep(0.2)


if __name__ == '__main__':
    main()
