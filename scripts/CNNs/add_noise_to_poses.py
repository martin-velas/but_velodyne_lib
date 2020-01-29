#! /usr/bin/env python

import sys, random
from odometry_cnn_data import load_kitti_poses

if len(sys.argv) != 7:
    sys.stderr.write("ERROR, expecting arguments - values of errors in: tx ty tz [m] roll pitch yaw [rad]\n")
    sys.exit()

error_limits = map(float, sys.argv[1:])
sys.stderr.write("Errors: %s\n" % error_limits)

error_limits[3:] = map(lambda x: x*3.14159/180.0, error_limits[3:])

for o in load_kitti_poses(sys.stdin):
    errors = map(lambda x: (random.random()*2-1)*x, error_limits)
    #sys.stderr.write("errors: %s\n" % errors)
    o.move(errors)
    print o
