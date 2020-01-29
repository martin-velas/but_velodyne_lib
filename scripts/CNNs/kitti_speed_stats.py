#! /usr/bin/env python

from __future__ import division
import sys
from math import sqrt

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry

PRESERVE_EACH = 3   # preserve only each third pose
MINIMAL_DIST = 0.1  # 10cm

def compute_speed(odometry):
    return sqrt(sum([t**2 for t in odometry.dof[0:3]]))*10  # 10fps

def print_stats():
    odometries = []
    for file in sys.argv[1:]:
        poses = load_kitti_poses(file)
        odometries += get_delta_odometry(poses)
    
    speeds = [compute_speed(o) for o in odometries]
    
    avg_speed = sum(speeds) / len(speeds)
    stdev_speed = sqrt(sum([(s-avg_speed)**2 for s in speeds])/len(speeds))
    
    print "avg[m/s]:", avg_speed, "standard deviation:", stdev_speed

poses = load_kitti_poses(sys.stdin)
mask = [i%PRESERVE_EACH == 0 for i in range(len(poses))]

last_pose = poses[0]
for i in range(1, len(poses)):
    if not mask[i]:
        continue
    if last_pose.distanceTo(poses[i]) < MINIMAL_DIST:
        mask[i] = False
    else:
        last_pose = poses[i]

for m in mask:
    print 1 if m else 0
