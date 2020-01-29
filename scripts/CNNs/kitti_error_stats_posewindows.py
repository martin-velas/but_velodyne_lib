#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses
import numpy as np
from transformations import matrix_to_axis_angle
import math
from os import path


VELODYNE_FPS = 10.0


def distance(pose):
    return np.linalg.norm(pose.M[0:3, 3])


def subseq_len(poses, start, end):
    if 0 <= start < end < len(poses):
        return distance(poses[end] - poses[start])
    else:
        return 10e6


def get_deltas_cumulated(poses, cumulate_distance, step_distance):
    deltas = []
    start_stops = []
    start_idx = 0
    end_idx = start_idx+1
    while end_idx < len(poses):
        while subseq_len(poses, start_idx, end_idx) < cumulate_distance:
            end_idx += 1
        if end_idx < len(poses):
            deltas.append(poses[end_idx] - poses[start_idx])
            start_stops.append((start_idx, end_idx))
        end_idx = start_idx + 1
        while subseq_len(poses, start_idx, end_idx) < step_distance:
            end_idx += 1
        start_idx = end_idx
        end_idx = start_idx+1
    return deltas, start_stops


def get_deltas_by_borders(poses, start_stops):
    deltas = []
    for start,end in start_stops:
        deltas.append(poses[end] - poses[start])
    return deltas


def rotation_angle(matrix3x3):
    return abs(matrix_to_axis_angle(matrix3x3)[1] * 180 / math.pi)


if len(sys.argv) != 5:
    sys.stderr.write("ERROR, expected arguments: [gt-poses.txt] [estimated-poses.txt] [cumulation-dist] [step-dist]\n")
    sys.exit(1)

cumulated_distance = int(sys.argv[3])
step_distance = int(sys.argv[4])
gt_filename = sys.argv[1]
seq_name = path.splitext(path.basename(gt_filename))[0]
gt_poses, start_stops = get_deltas_cumulated(load_kitti_poses(gt_filename), cumulated_distance, step_distance)
odom_poses = get_deltas_by_borders(load_kitti_poses(sys.argv[2]), start_stops)

assert len(gt_poses) == len(odom_poses)

for idx, ((start_idx, end_idx), gt, odom) in enumerate(zip(start_stops, gt_poses, odom_poses)):
    diff = odom - gt
    # gt_R err_t
    output = [rotation_angle(gt.M[0:3, 0:3]), distance(diff)]
    sys.stdout.write("%s %d %d " % (seq_name, start_idx, end_idx))
    for nm in output:
        sys.stdout.write("%.4f " % nm)
    print ""
