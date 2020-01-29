#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from string import strip
import json

from odometry_cnn_data import Odometry
from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from kitti_poses_rot_quantization import angles2classes
from numpy_point_cloud import DEG_TO_RAD

rots_x = []
rots_y = []
rots_z = []

for file in sys.argv[1:]:
    odometries = get_delta_odometry(load_kitti_poses(file))
    for o in odometries:
        rots_x.append(o.dof[3])
        rots_y.append(o.dof[4])
        rots_z.append(o.dof[5])

rots_x.sort()
rots_y.sort()
rots_z.sort()

for i in range(len(rots_x)):
    print "%s;%s;%s" % (rots_x[i]/DEG_TO_RAD, rots_y[i]/DEG_TO_RAD, rots_z[i]/DEG_TO_RAD)
