#!/usr/bin/env python3
"""Post-hoc numeric summary of saved viewpoint JPEGs (no display needed).

For each view_XX.jpg prints: mean RGB, red-marker pixel count and its centroid
offset from frame center (px).  A small offset means the intruder's red marker
is near the frame center -> aim was on target.
"""
import argparse
import glob
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGES_ROOT = os.path.join(os.path.dirname(HERE), "images")


def red_stats(arr):
    r = arr[:, :, 0].astype(np.int32)
    g = arr[:, :, 1].astype(np.int32)
    b = arr[:, :, 2].astype(np.int32)
    mask = (r > 90) & (g < r * 0.5) & (b < r * 0.5)
    n = int(mask.sum())
    if n == 0:
        return None
    ys, xs = np.nonzero(mask)
    h, w = mask.shape
    return (round(float(xs.mean()) - w / 2.0, 1),
            round(float(ys.mean()) - h / 2.0, 1), n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--subdir', default='', help='subfolder under images/ (e.g. r_100)')
    args = ap.parse_args()
    outdir = os.path.join(IMAGES_ROOT, args.subdir) if args.subdir else IMAGES_ROOT
    files = sorted(glob.glob(os.path.join(outdir, 'view_*.jpg')))
    print('found %d jpgs in %s\n' % (len(files), outdir))
    print('%-10s %-14s %-10s %-22s' % ('file', 'mean RGB', 'redpx', 'red offset (dx,dy)'))
    print('-' * 60)
    for fp in files:
        arr = np.asarray(Image.open(fp).convert('RGB'))
        mean = arr.reshape(-1, 3).mean(axis=0).round(1)
        rc = red_stats(arr)
        off = '%.1f,%.1f' % rc[:2] if rc else 'no-red'
        print('%-10s %-14s %-10d %-22s' % (
            os.path.basename(fp), str(tuple(mean)), rc[2] if rc else 0, off))


if __name__ == '__main__':
    main()
