#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses, get_delta_odometry
from pose_graph import Edge3D

odoms = get_delta_odometry(load_kitti_poses(sys.stdin))
for i in range(1, len(odoms)):
    print Edge3D(i-1, i, odoms[i].dof)
