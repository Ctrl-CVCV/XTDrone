#!/usr/bin/env python3
"""box_tool.py <spawn|delete> <name> [x y] [sdf]  -- spawn/delete a test obstacle.
Default sdf is box_obs.sdf; pass a path (e.g. /workspace/wall_obs.sdf) to
spawn a different shape."""
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

cmd, name = sys.argv[1], sys.argv[2]
rospy.init_node('box_tool', anonymous=True, disable_signals=True)

if cmd == 'spawn':
    x, y = float(sys.argv[3]), float(sys.argv[4])
    sdf_path = sys.argv[5] if len(sys.argv) > 5 else '/workspace/box_obs.sdf'
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    sdf = open(sdf_path).read()
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, 0.3
    p.orientation.w = 1.0
    resp = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
        name, sdf, '', p, 'world')
    print('spawn', name, 'at', (x, y), '->', resp.status_message)
elif cmd == 'delete':
    rospy.wait_for_service('/gazebo/delete_model')
    resp = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(name)
    print('delete', name, '->', resp.status_message)
