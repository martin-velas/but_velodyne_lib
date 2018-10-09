#! /usr/bin/env python

import sys
import numpy as np
from odometry_cnn_data import load_poses_corrections

if len(sys.argv) > 1:
    skip = float(sys.argv[1])
else:
    skip = 0.05

REF_PT = np.array([[10], [10], [10], [1]])

corrections = load_poses_corrections(sys.stdin)

for c in corrections:
    new_pt = np.dot(c["pose"].M, REF_PT)
    dist = np.linalg.norm(new_pt-REF_PT)
    c["dist"] = dist

corrections.sort(key=(lambda x: x["dist"]))

for c in corrections[:int((1.0-skip)*len(corrections))]:
    print "%d %d %s"%(c["src_i"], c["trg_i"], c["pose"])
