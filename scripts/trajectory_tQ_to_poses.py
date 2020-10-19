#! /usr/bin/env python

import sys
import numpy as np

from transformations_kittilike import quaternion_to_matrix
from odometry_cnn_data import kitti_pose_string


for line in sys.stdin.readlines():
    tokens = map(float, line.split())
    assert len(tokens) == 8

    time = tokens[0]
    # x, y, z, qx, qy, qz, qw
    tQ = tokens[1:]

    M = quaternion_to_matrix(tQ[6:] + tQ[3:6])
    M[0:3, 3] = tQ[:3]

    print time, kitti_pose_string(M)
