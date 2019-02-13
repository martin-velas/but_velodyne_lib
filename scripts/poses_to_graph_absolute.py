#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses
from pose_graph import Edge3D


def load_inf_matrix_diagonals(filename):
    diagonals = []
    for line in open(filename).readlines():
        diagonals.append(map(float, line.split()))
    return diagonals


poses = load_kitti_poses(sys.stdin)

if len(sys.argv) > 1:
    if len(sys.argv) == 2:
        inf_matrix_diagonals = load_inf_matrix_diagonals(sys.argv[1])
        assert len(inf_matrix_diagonals) == len(poses)
    elif len(sys.argv) == 7:
        diagonal = map(float, sys.argv[1:])
        assert len(diagonal) == 6
        inf_matrix_diagonals = [diagonal]*len(poses)
    else:
        sys.stderr.write("ERROR: expecting 6 values (diagonal of information matrix)\n")
        sys.exit(1)
else:
    inf_matrix_diagonals = [None]*len(poses)

print "VERTEX3 -1 0 0 0 0 0 0"

for (i, pose) in enumerate(poses):
    print Edge3D(-1, i, pose.dof, inf_matrix_diagonals[i])
