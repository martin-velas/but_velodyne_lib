#! /usr/bin/env python

import sys, math
import matplotlib.pyplot as plt

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry

yaw_diffs = []
for pose_file in sys.argv[1:]:
    deltas = get_delta_odometry(load_kitti_poses(pose_file))
    for dist in range(1, 4):
        yaw_diffs += [(deltas[i].dof[4]-deltas[i-dist].dof[4])*180/math.pi for i in range(dist, len(deltas))]

yaw_diffs.sort()
N = len(yaw_diffs)
yaw_diffs = yaw_diffs[int(0.001*N):int(0.999*N)]

mean_yaw = sum(yaw_diffs)/N
sdev_yaw = math.sqrt(sum([(x-mean_yaw)**2 for x in yaw_diffs])/N)
print "min", min(yaw_diffs), "max", max(yaw_diffs), "mean:", mean_yaw, "stdev:", sdev_yaw

plt.title("Histogram of angle diffs")
plt.hist(yaw_diffs, bins=100)  # plt.hist passes it's arguments to np.histogram
plt.show()
plt.clf()
