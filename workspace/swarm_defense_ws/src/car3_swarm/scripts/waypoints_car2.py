#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V2 multi-waypoint navigation for car2 (intruder, NW outer corner).

Intrusion path: NW corner -> east along the north corridor -> south through
the north door -> into the inner room center. Goals are sent one at a time;
each waits for its terminal state before the next (no overwrite).
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from swarm_waypoints import run_waypoints

WAYPOINTS = [
    (0.0, 6.0, -1.5708),     # 沿北走廊东行, 车头朝南对准北门
    (0.0, 1.8, -1.5708),     # 穿北门进入内房间
    (0.0, 0.8, -1.5708),     # 向内房间中心推进
]

if __name__ == "__main__":
    run_waypoints("car2", WAYPOINTS)
