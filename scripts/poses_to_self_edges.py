#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses, get_delta_odometry
from pose_graph import Edge3DSelf

inf_matrix_diagonal = None
if len(sys.argv) > 1:
    if len(sys.argv) == 7:
        inf_matrix_diagonal = map(float, sys.argv[1:])
        assert len(inf_matrix_diagonal) == 6
    else:
        sys.stderr.write("ERROR: exprecting 6 values (diagonal of information matrix)\n")
        sys.exit(1)

poses = load_kitti_poses(sys.stdin)

for (i, p) in enumerate(poses):
    print Edge3DSelf(i, p, inf_matrix_diagonal)
