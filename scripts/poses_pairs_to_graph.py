#! /usr/bin/env python

import argparse
from odometry_cnn_data import load_kitti_poses, load_poses_corrections
from pose_graph import Edge3D

parser = argparse.ArgumentParser(description="Registered frames pairs to EDGE3D")
parser.add_argument("--poses", dest="poses", type=str, required=True)
parser.add_argument("--corrections", dest="corrections", type=str, required=True)
parser.add_argument("--cumulated_frames", dest="cumulated_frames", type=int, required=True)
args = parser.parse_args()

poses = load_kitti_poses(args.poses)
corrections = load_poses_corrections(args.corrections)

for c in corrections:
    src_i, trg_i = c["src_i"], c["trg_i"]
    correction = c["pose"]
    for i in range(args.cumulated_frames):
        src_pose = poses[src_i+i]
        trg_pose = correction * poses[trg_i+i]
        t = src_pose.inv() * trg_pose
        print Edge3D(src_i+i, trg_i+i, t.dof)
