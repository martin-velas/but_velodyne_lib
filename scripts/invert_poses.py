#! /usr/bin/env python

import sys

from odometry_cnn_data import load_kitti_poses

for p in load_kitti_poses(sys.stdin):
    print p.inv()
