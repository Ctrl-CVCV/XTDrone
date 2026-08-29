#!/usr/bin/env python3
"""Per-radius-folder summary of captured JPEGs (no display needed).

For each images/r_XXX prints: file count, views with red pixels, median redpx,
and median |offset| from frame center.
"""
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
    return n, abs(xs.mean() - w / 2.0), abs(ys.mean() - h / 2.0)


def main():
    dirs = sorted(glob.glob(os.path.join(IMAGES_ROOT, 'r_*')))
    print('%-8s %-6s %-10s %-10s %-12s' % (
        'folder', 'files', 'with_red', 'med_redpx', 'med|offset|px'))
    print('-' * 52)
    for d in dirs:
        files = sorted(glob.glob(os.path.join(d, 'view_*.jpg')))
        counts = []
        offsets = []
        for fp in files:
            arr = np.asarray(Image.open(fp).convert('RGB'))
            s = red_stats(arr)
            if s:
                counts.append(s[0])
                offsets.append(s[1] + s[2])
        name = os.path.basename(d)
        if files:
            print('%-8s %-6d %-10d %-10.0f %-12.1f' % (
                name, len(files), len(counts),
                float(np.median(counts)) if counts else 0.0,
                float(np.median(offsets)) if offsets else 0.0))
        else:
            print('%-8s %-6d (EMPTY)' % (name, len(files)))


if __name__ == '__main__':
    main()
