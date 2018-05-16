#! /usr/bin/env python

import sys
import numpy as np
from transformations import quaternion_matrix
from odometry_cnn_data import load_kitti_poses, Odometry
import json


VELO_TO_IMU = np.array([0,0,1,0,
                        1,0,0,0,
                        0,1,0,0,
                        0,0,0,1]).reshape((4,4))

CALIBRATION = Odometry()
CALIBRATION.move([0,0,0,-0.05,0,0]) # Velodyne is tilted a bit


def get_imu_orientations(imu_data, frame_times):
    orientations_timed = {}
    for record in imu_data:
        if record["type"] == "quat_data":
            orientations_timed[record["gpsTimeOfWeek"]] = record["quaternion"]

    orientations = []
    for t in frame_times:
        quat = orientations_timed[t]
        M = quaternion_matrix(quat[3:] + quat[0:3])
        M = np.matmul(np.matmul(np.linalg.inv(VELO_TO_IMU), M), VELO_TO_IMU)
        o = Odometry()
        o.M = M
        o.setDofFromM()
        orientations.append(o)
    return orientations


if len(sys.argv) != 6:
    sys.stderr.write("ERROR, expecting arguments: [slam-poses] [frame-times] [imu-data-log] [out-aligned.poses] [out-imu.poses]\n")
    sys.exit(1)

slam_poses = load_kitti_poses(sys.argv[1])
frame_times = map(float, open(sys.argv[2]).readlines())
imu_data = json.load(open(sys.argv[3]))

imu_orientations = get_imu_orientations(imu_data, frame_times)

alignment = slam_poses[0].inv() * imu_orientations[0] * CALIBRATION

aligned_poses_file = open(sys.argv[4], "w")
out_imu_poses_file = open(sys.argv[5], "w")

for i,p in enumerate(slam_poses):
    p = alignment * p
    aligned_poses_file.write("%s\n" %(p,))

    imu_orientations[i].M[0:3, 3] = p.M[0:3, 3].ravel()
    imu_orientations[i].setDofFromM()
    out_imu_poses_file.write("%s\n" %(imu_orientations[i],))
