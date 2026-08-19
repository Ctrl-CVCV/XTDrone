#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1 keyboard control for car1 (defender, SW inner corner)."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from swarm_teleop import run_teleop

if __name__ == "__main__":
    run_teleop("car1")
