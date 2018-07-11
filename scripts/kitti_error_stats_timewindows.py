#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses, odom_rad_to_deg
import numpy as np
from transformations import matrix_to_axis_angle
import math


VELODYNE_FPS = 10.0


def get_deltas_cumulated(poses, cumulated_frames):
    result = []
    for i in range(len(poses)-cumulated_frames):
        result.append(poses[i+cumulated_frames] - poses[i])
    return result


def distance(pose):
    return np.linalg.norm(pose.M[0:3, 3])


def speed(pose, cumulated_frames):
    return distance(pose) * (VELODYNE_FPS / cumulated_frames) * 3.6


def rotation_angle(matrix3x3):
    return abs(matrix_to_axis_angle(matrix3x3)[1] * 180 / math.pi)


# rx, ry, rz, |R|
def rotation_magnitudes(pose):
    return map(lambda x: abs(x*180.0/math.pi), list(pose.dof[3:])) + [rotation_angle(pose.M[0:3, 0:3])]


if len(sys.argv) != 4:
    sys.stderr.write("ERROR, expected arguments: [gt-poses.txt] [estimated-poses.txt] [cumulation]\n")
    sys.exit(1)

gt_filename = sys.argv[1]
cumulated_frames = int(sys.argv[3])
gt_poses = get_deltas_cumulated(load_kitti_poses(gt_filename), cumulated_frames)
odom_poses = get_deltas_cumulated(load_kitti_poses(sys.argv[2]), cumulated_frames)

assert len(gt_poses) == len(odom_poses)

for idx, (gt, odom) in enumerate(zip(gt_poses, odom_poses)):
    diff = odom - gt
    # speed gt_rx gt_ry gt_rz gt_R err_t err_rx err_ry err_rz err_R
    output = [speed(gt, cumulated_frames)] + rotation_magnitudes(gt) + [distance(diff)*(VELODYNE_FPS/cumulated_frames)] + rotation_magnitudes(diff)
    for nm in output:
        sys.stdout.write("%.4f " % nm)
    print ""
