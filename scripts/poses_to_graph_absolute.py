#! /usr/bin/env python

import argparse
from odometry_cnn_data import load_kitti_poses
from pose_graph import Edge3D


parser = argparse.ArgumentParser(description="Absolute pose graph edges from GNSS/INS poses.")
parser.add_argument("--poses", dest="poses", type=str, required=True)
parser.add_argument("--precisions", dest="precisions", type=str, required=True)
parser.add_argument("--fixes", dest="fixes", type=str, required=True)
parser.add_argument("--stdevs", dest="stdevs", type=str, required=True)
parser.add_argument("--max_stdev", dest="max_stdev", type=float, required=True)
args = parser.parse_args()


def load_inf_matrix_diagonals(file):
    diagonals = []
    for line in file.readlines():
        diagonals.append(map(float, line.split()))
    return diagonals


ACCEPTED_FIXES = ["RTK_FIX", "RTK_FLOAT"]


with open(args.poses) as poses_file, open(args.precisions) as precisions_file, open(args.fixes) as fixes_file, open(args.stdevs) as stdevs_file:

    poses = load_kitti_poses(poses_file)

    inf_matrix_diagonals = load_inf_matrix_diagonals(precisions_file)
    assert len(poses) == len(inf_matrix_diagonals)

    fixes = map(lambda l: l.strip(), fixes_file.readlines())
    assert len(poses) == len(fixes)
    stdevs = map(lambda l: max(map(float, l.split()[0:3])), stdevs_file.readlines())
    assert len(poses) == len(stdevs)

    print "VERTEX3 -1 0 0 0 0 0 0"

    for i, (pose, fix, stdev, inf_matrix) in enumerate(zip(poses, fixes, stdevs, inf_matrix_diagonals)):
        if (fix in ACCEPTED_FIXES) and (stdev < args.max_stdev):
            print Edge3D(-1, i, pose.dof, inf_matrix)
