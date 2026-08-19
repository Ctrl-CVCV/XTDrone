#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V2 multi-waypoint navigation for car1 (defender, SW inner corner).

Path: SW corner -> contract toward room center -> flank the north door
facing north to block the intruder. Goals are sent one at a time; each
waits for its terminal state before the next (no overwrite).
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from swarm_waypoints import run_waypoints

WAYPOINTS = [
    (-1.2, -1.2, 0.785),     # 向房间中心收缩
    (-0.6, 0.4, 1.5708),     # 北门内侧左翼, 车头朝北堵截
]

if __name__ == "__main__":
    run_waypoints("car1", WAYPOINTS)
