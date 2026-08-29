#!/usr/bin/env python3
"""Capture viewpoint photos of the hovering intruder UAV (iris_0).

A virtual sphere of given radius is centered on the aircraft geometric center
(base_link origin = gazebo model pose).  N near-uniform points on the sphere
(Fibonacci) each get an INVISIBLE camera (gimbal spec: 640x480 R8G8B8,
horizontal_fov 1.0472, clip 0.01..100) aimed at the aircraft.  One JPEG per
point is saved to <task>/images/<subdir>/view_XXX.jpg.

Usage:
    capture_views.py [--radius 1.0] [--num 32] [--subdir r_100]

Verification (no display available): each view logs the red-marker pixel
centroid offset from frame center so aim accuracy can be checked numerically.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from sensor_msgs.msg import Image
from PIL import Image as PILImage

AIRBORNE_Z = 7.0          # world z considered "airborne" after takeoff to 8 m
CAPTURE_TIMEOUT = 12.0    # seconds to wait for a frame per view

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "camera_view.sdf")
IMAGES_ROOT = os.path.join(os.path.dirname(HERE), "images")


def fibonacci_sphere(n):
    """Near-uniform points on unit sphere (golden-angle method)."""
    pts = []
    ga = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(n):
        y = 1.0 - 2.0 * (i + 0.5) / n
        r = math.sqrt(max(0.0, 1.0 - y * y))
        th = ga * i
        pts.append((math.cos(th) * r, y, math.sin(th) * r))
    return pts


def quat_from_aim(forward):
    """Quaternion (w,x,y,z) whose local +X points along `forward` and whose
    local +Z (image up) is as close to world-up as possible."""
    f = np.asarray(forward, dtype=float)
    f /= np.linalg.norm(f)
    up_goal = np.array([0.0, 0.0, 1.0])
    r = np.cross(up_goal, f)
    nr = np.linalg.norm(r)
    if nr < 1e-6:
        r = np.array([1.0, 0.0, 0.0])   # looking straight up/down: arbitrary right
    else:
        r /= nr
    u = np.cross(f, r)                  # right-handed: F x R = U
    m = np.column_stack([f, r, u])
    # rotation matrix -> quaternion (Shepperd)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


def red_centroid(rgb):
    """Pixel-centroid of the red intruder marker, offset from frame center (px)."""
    r = rgb[:, :, 0].astype(np.int32)
    g = rgb[:, :, 1].astype(np.int32)
    b = rgb[:, :, 2].astype(np.int32)
    mask = (r > 90) & (g < r * 0.5) & (b < r * 0.5)
    n = int(mask.sum())
    if n == 0:
        return None
    ys, xs = np.nonzero(mask)
    cy = float(ys.mean())
    cx = float(xs.mean())
    h, w = mask.shape
    return (cx - w / 2.0), (cy - h / 2.0), n


def wait_airborne(timeout=120.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            ms = rospy.wait_for_message(
                '/gazebo/model_states', ModelStates,
                timeout=max(0.5, min(5.0, timeout - (time.time() - t0))))
        except rospy.ROSException:
            continue
        try:
            i = ms.name.index('iris_0')
        except ValueError:
            i = -1
        if i >= 0 and ms.pose[i].position.z > AIRBORNE_Z:
            return ms.pose[i]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radius', type=float, default=0.5, help='sphere radius in meters')
    ap.add_argument('--num', type=int, default=32, help='number of sphere points')
    ap.add_argument('--subdir', default='', help='subfolder under images/ (e.g. r_100)')
    args = ap.parse_args()

    radius = args.radius
    num = args.num
    outdir = os.path.join(IMAGES_ROOT, args.subdir) if args.subdir else IMAGES_ROOT
    ndigits = max(2, len(str(num - 1)))

    rospy.init_node('capture_views', anonymous=False)
    os.makedirs(outdir, exist_ok=True)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    dele = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    rospy.loginfo('waiting for iris_0 to be airborne (z > %.1f m) ...', AIRBORNE_Z)
    pose = wait_airborne()
    if pose is None:
        rospy.logerr('iris_0 never reached airborne height')
        sys.exit(1)
    center = np.array([pose.position.x, pose.position.y, pose.position.z])
    rospy.loginfo('aircraft geometric center at (%.3f, %.3f, %.3f)', *center)

    template = open(TEMPLATE).read()
    views = fibonacci_sphere(num)
    results = []
    for k, (dx, dy, dz) in enumerate(views):
        cam_pos = center + radius * np.array([dx, dy, dz])
        w, x, y, z = quat_from_aim(center - cam_pos)
        model_name = 'view_cam_%0*d' % (ndigits, k)
        topic = '/%s/cam/image_raw' % model_name

        sdf = template.replace('@{MODEL_NAME}', model_name).replace('@{CAMERA_NAME}', 'cam')
        pose_msg = Pose(position=Point(*cam_pos),
                        orientation=Quaternion(x=x, y=y, z=z, w=w))
        spawn(model_name=model_name, model_xml=sdf, robot_namespace='',
              initial_pose=pose_msg, reference_frame='world')

        img = None
        t0 = rospy.Time.now().to_sec()
        while img is None and rospy.Time.now().to_sec() - t0 < CAPTURE_TIMEOUT:
            try:
                img = rospy.wait_for_message(topic, Image, timeout=3)
            except rospy.ROSException:
                pass
        ok = img is not None
        if ok:
            arr = np.frombuffer(img.data, dtype=np.uint8).reshape(
                img.height, img.width, 3)
            out = os.path.join(outdir, 'view_%0*d.jpg' % (ndigits, k))
            PILImage.fromarray(arr).save(out, quality=95)
            rc = red_centroid(arr)
            if rc is not None:
                rospy.loginfo('view %0*d saved cam=(%.2f,%.2f,%.2f) '
                              'red-centroid-offset=(%.1f,%.1f)px redpx=%d',
                              ndigits, k, *cam_pos, rc[0], rc[1], rc[2])
            else:
                rospy.loginfo('view %0*d saved cam=(%.2f,%.2f,%.2f) no-red-pixels',
                              ndigits, k, *cam_pos)
        else:
            rospy.logerr('view %0*d: no frame on %s', ndigits, k, topic)

        try:
            dele(model_name=model_name)
        except rospy.ServiceException as exc:
            rospy.logwarn('delete %s failed: %s', model_name, exc)
        rospy.sleep(0.2)
        results.append(ok)

    n_ok = sum(results)
    rospy.loginfo('capture finished: %d/%d views saved to %s', n_ok, num, outdir)
    sys.exit(0 if n_ok == num else 2)


if __name__ == '__main__':
    main()
