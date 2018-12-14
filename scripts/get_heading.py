#! /usr/bin/env python

import sys

from odometry_cnn_data import load_kitti_poses, Odometry
from math import acos, sqrt

for p in load_kitti_poses(sys.stdin):
    x = p.M[0, 2]
    z = p.M[2, 2]
    theta = acos(z / sqrt(x**2 + z**2))
    if x < 0:
        theta = -theta
    fix = Odometry().move([0, 0, 0, 0, theta, 0])
    print fix
