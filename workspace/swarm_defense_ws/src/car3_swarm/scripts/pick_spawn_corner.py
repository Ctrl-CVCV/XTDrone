#!/usr/bin/env python3
"""Pick iris_2's spawn corner from the air-intruder mission YAML.

Prints three lines: CORNER_NAME, X, Y (and optionally YAW with --yaw).
Used by launch_air_intruder_sim.sh to choose spawn values at sim start.

Flags:
  --spawn-mode random|fixed   override YAML intruder.spawn_mode
  --corner CORNER_0|CORNER_1  override YAML intruder.fixed_corner (fixed mode)
  --seed N                     random seed for reproducible corner choice
"""

import argparse
import os
import random
import sys

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_YAML = os.path.join(
    SCRIPT_DIR, "..", "config", "air_intruder_mission.yaml"
)


def load_config(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--spawn-mode", choices=["random", "fixed"], default=None)
    parser.add_argument("--corner", default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--yaw", action="store_true")
    args = parser.parse_args()

    config = load_config(args.yaml)
    intruder = config["intruder"]
    spawn_mode = args.spawn_mode or intruder["spawn_mode"]
    seed = args.seed if args.seed is not None else intruder.get("random_seed", 0)

    if spawn_mode == "random":
        rng = random.Random(seed)
        corner = rng.choice(["CORNER_0", "CORNER_1"])
    else:
        corner = (args.corner or intruder["fixed_corner"]).upper()
        if corner not in ("CORNER_0", "CORNER_1"):
            sys.exit("unknown corner %r" % corner)

    key = "spawn_corner_0" if corner == "CORNER_0" else "spawn_corner_1"
    x, y, yaw = intruder[key]
    print(corner)
    print("%.4f" % x)
    print("%.4f" % y)
    if args.yaw:
        print("%.4f" % yaw)


if __name__ == "__main__":
    main()
