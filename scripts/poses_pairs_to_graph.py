#! /usr/bin/env python

import argparse, sys
from odometry_cnn_data import load_kitti_poses, load_poses_corrections
from pose_graph import Edge3D

parser = argparse.ArgumentParser(description="Registered frames pairs to EDGE3D")
parser.add_argument("--poses", dest="poses", type=str, required=False)
parser.add_argument("--corrections", dest="corrections", type=str, required=False)
parser.add_argument("--edges", dest="edges", type=str, required=False)
parser.add_argument("--cumulated_frames", dest="cumulated_frames", type=int, required=False, default=1)
parser.add_argument("-w", "--weights", nargs='+', dest="inf_diagonal", type=float, help='The diagonal of information matrix', required=False)
args = parser.parse_args()

if (args.poses is not None) and (args.corrections is not None) and (args.edges is None):
    poses = load_kitti_poses(args.poses)
    corrections = load_poses_corrections(args.corrections)
elif (args.poses is None) and (args.corrections is None) and (args.edges is not None):
    poses = None
    corrections = load_poses_corrections(args.edges)
else:
    sys.stderr.write("ERROR: corrections+poses OR edges are needed\n")
    sys.exit(1)

inf_diagonal = [100.0, 100.0, 100.0, 250000.0, 250000.0, 250000.0] if args.inf_diagonal is None else args.inf_diagonal


for c in corrections:
    src_i, trg_i = c["src_i"], c["trg_i"]
    correction = c["pose"]
    for i in range(args.cumulated_frames):
        if poses is None:
            t = correction
        else:
            src_pose = poses[src_i+i]
            trg_pose = correction * poses[trg_i+i]
            t = src_pose.inv() * trg_pose
        print Edge3D(src_i+i, trg_i+i, t.dof, inf_diagonal)
