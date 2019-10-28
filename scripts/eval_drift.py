#! /usr/bin/env python

import argparse
from odometry_cnn_data import load_kitti_poses
import numpy as np

parser = argparse.ArgumentParser(description="Drift estimation")
parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
parser.add_argument("-l", "--loop", dest="loop", type=str, required=True)
args = parser.parse_args()

poses = load_kitti_poses(args.poses)
loop_poses = load_kitti_poses(args.loop)

loop_gt = loop_poses[0].inv() * loop_poses[-1]
loop_estimation = poses[0].inv() * poses[-1]

diff = loop_gt.inv() * loop_estimation

print "%.4f" % (np.linalg.norm(diff.dof[0:3]),)
