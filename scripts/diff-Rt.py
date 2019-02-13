#! /usr/bin/env python

import argparse
from odometry_cnn_data import load_kitti_poses, load_poses_corrections
import numpy as np
from math import acos, pi
from transformations import matrix_to_axis_angle

parser = argparse.ArgumentParser(description="Compute Rt difference of corrections ")
parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
parser.add_argument("-c", "--corrections", dest="corrections", type=str, required=True)
args = parser.parse_args()

poses = load_kitti_poses(args.poses)
corrections = load_poses_corrections(args.corrections)

for c in corrections:
    src_i, trg_i = c["src_i"], c["trg_i"]
    correction = c["pose"]
    expectation = poses[src_i].inv() * poses[trg_i]
    diff = correction.inv() * expectation

    _, angle_diff = matrix_to_axis_angle(diff.M)

    print src_i, trg_i, angle_diff*180/pi, np.linalg.norm(diff.M[0:3, 3])
