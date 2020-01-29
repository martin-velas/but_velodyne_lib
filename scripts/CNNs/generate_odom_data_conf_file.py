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

# ls $PWD/*/sequences/te0000/velodyne/*.gz | while read i; do echo $(dirname $i) $(basename $i); done | sort --version-sort -k1 | sort -sgk2 | tr " " "/"
if len(sys.argv) != 3:
    sys.stderr.write("ERROR, expected arguments: [pose-file.txt] [file-list-sorted]\n")
    sys.exit(1)

odometries = get_delta_odometry(load_kitti_poses(sys.argv[1]))
files = map(strip, open(sys.argv[2]).readlines())

assert len(files) % len(odometries) == 0
shifts_count = len(files) / len(odometries)

FILES_SCHEMA = [6, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
CLASSES_COUNTS=[13, 56, 13]
MAX_ANGLES=[1.2*DEG_TO_RAD, 5.6*DEG_TO_RAD, 1.2*DEG_TO_RAD]


frames = []
for odom in odometries[1:]:
    data = [files[fi] for fi in FILES_SCHEMA]
    classes = angles2classes(odom.dof[3:], MAX_ANGLES, CLASSES_COUNTS)

    for dato in odom.dof.tolist() + classes + data:
        sys.stdout.write("%s "%dato)
    print ""
    #frames.append({"odometry":odom.dof, "data":data, "rot_class_x":classes[0], "rot_class_y":classes[1], "rot_class_z":classes[2]})

    FILES_SCHEMA = map(lambda fi: fi+shifts_count, FILES_SCHEMA)

#print json.dumps(frames, indent=2, sort_keys=True)
