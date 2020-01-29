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
if len(sys.argv) != 2:
    sys.stderr.write("ERROR, expected arguments: [file-list-sorted]\n")
    sys.exit(1)

files = map(strip, open(sys.argv[1]).readlines())

#FILES_SCHEMA = [6, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
FILES_SCHEMA = [0, 1]

shifts_count = len(FILES_SCHEMA)-1

while max(FILES_SCHEMA) < len(files):
    for dato in [files[fi] for fi in FILES_SCHEMA]:
        sys.stdout.write("%s " % dato)
    print ""

    FILES_SCHEMA = map(lambda fi: fi+shifts_count, FILES_SCHEMA)
