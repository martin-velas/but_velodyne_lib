#! /usr/bin/env python

import sys

from odometry_cnn_data import load_kitti_poses, Odometry

pose_sets = []
poses_count = -1
for a in sys.argv[1:]:
    poses = load_kitti_poses(a)
    poses_count = max(poses_count, len(poses))
    pose_sets.append(poses)

for i,poses in enumerate(pose_sets):
    if len(poses) == 1:
        pose_sets[i] = poses*poses_count
    poses_i_cnt = len(pose_sets[i])
    if poses_i_cnt != poses_count:
        sys.stderr.write("Warning: poses count do not match (%s vs %s)\n" % (poses_i_cnt, poses_count))
        poses_count = min(poses_count, poses_i_cnt)

for i in range(poses_count):
    output = Odometry()
    for slot in range(len(pose_sets)):
        output = output * pose_sets[slot][i]
    print output
