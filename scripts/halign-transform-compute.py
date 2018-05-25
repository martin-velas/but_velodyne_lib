#! /usr/bin/env python

import sys
import numpy as np
from transformations_kittilike import quaternion_to_matrix
from odometry_cnn_data import load_kitti_poses, Odometry
import json
import math


VELO_TO_IMU = np.array([0,0,1,0,
                        1,0,0,0,
                        0,1,0,0,
                        0,0,0,1]).reshape((4,4))

CALIBRATION = Odometry()
CALIBRATION.move([0, 0, 0, math.radians(-3), 0, 0])     # Velodyne is tilted -3deg


def find_closest_value(dict, key):
    return dict[key] if key in dict else dict[min(dict.keys(), key=lambda k: abs(k - key))]


def get_imu_orientations(imu_data, frame_times):
    orientations_timed = {}
    for record in imu_data:
        if record["type"] == "quat_data":
            orientations_timed[float(record["gpsTimeOfWeek"])] = record["quaternion"]

    orientations = []
    for t in frame_times:
        quat = find_closest_value(orientations_timed, t) # [x, y, z, w] in JSON
        M = quaternion_to_matrix(quat[3:] + quat[0:3])
        M = np.matmul(np.matmul(np.linalg.inv(VELO_TO_IMU), M), VELO_TO_IMU)
        o = Odometry()
        o.M = M
        o.setDofFromM()
        orientations.append(o)
    return orientations


if len(sys.argv) != 6:
    sys.stderr.write("ERROR, expecting arguments: [slam-poses] [frame-times] [imu-data-log] [out-imu-aligned-slam.poses] [out-imu-in-velodyne-body.poses]\n")
    sys.exit(1)

slam_poses = load_kitti_poses(sys.argv[1])
frame_times = map(float, open(sys.argv[2]).readlines())
imu_data = json.load(open(sys.argv[3]))

imu_orientations = get_imu_orientations(imu_data, frame_times)
imu_orientations_in_velodyne_body = map(lambda x: x*CALIBRATION, imu_orientations)

alignment = imu_orientations_in_velodyne_body[0]
slam_poses_imu_aligned = map(lambda x: alignment*x, slam_poses)

aligned_poses_file = open(sys.argv[4], "w")
out_imu_poses_file = open(sys.argv[5], "w")

for i,(imu_p,slam_p) in enumerate(zip(imu_orientations_in_velodyne_body, slam_poses_imu_aligned)):
    aligned_poses_file.write("%s\n" %(slam_p,))

    imu_p.M[0:3, 3] = slam_p.M[0:3, 3]
    imu_p.setDofFromM()
    out_imu_poses_file.write("%s\n" %(imu_p,))
