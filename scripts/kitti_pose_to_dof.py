#! /usr/bin/env python

import sys
from odometry_cnn_data import get_delta_odometry, load_kitti_poses, Odometry

odometries = get_delta_odometry(load_kitti_poses(sys.stdin))

# to ROS
axis_correction = Odometry([
        0, -1,  0,  0,
        0,  0, -1,  0,
        1,  0,  0,  0 ])

for o in odometries:
    print ("%s "*6)%(o.dof[0], o.dof[1], o.dof[2], o.dof[3], o.dof[4], o.dof[5])
