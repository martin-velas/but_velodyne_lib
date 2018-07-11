#! /usr/bin/env python

import sys
import numpy as np
from transformations_kittilike import quaternion_to_matrix, matrix_to_quaternion
from odometry_cnn_data import load_kitti_poses, Odometry
import json
import math
from averageQuaternions import averageQuaternions
from imu_orientations_extraction import get_imu_orientations


CALIBRATION = Odometry()
CALIBRATION.move([0, 0, 0, math.radians(-3), 0, 0])     # Velodyne is tilted -3deg


def get_alignment(poses_by_slam, poses_by_imu):
    delta_quaternions = np.zeros((len(poses_by_imu), 4))
    for i, (slam_p, imu_p) in enumerate(zip(poses_by_slam, poses_by_imu)):
        delta_m = imu_p.M[0:3, 0:3] * np.linalg.inv(slam_p.M[0:3, 0:3])
        delta_quaternions[i, :] = matrix_to_quaternion(delta_m)
    avg_q = averageQuaternions(delta_quaternions)
    alignment = Odometry()
    alignment.M = quaternion_to_matrix(avg_q)
    alignment.setDofFromM()
    return alignment


if len(sys.argv) != 6:
    sys.stderr.write("ERROR, expecting arguments: [slam-poses] [frame-times] [imu-data-log] [out-imu-aligned-slam.poses] [out-imu-in-velodyne-body.poses]\n")
    sys.exit(1)

slam_poses = load_kitti_poses(sys.argv[1])
frame_times = map(float, open(sys.argv[2]).readlines())
imu_data = json.load(open(sys.argv[3]))

imu_orientations = get_imu_orientations(imu_data, frame_times)
imu_orientations_in_velodyne_body = map(lambda x: x*CALIBRATION, imu_orientations)

alignment = get_alignment(slam_poses, imu_orientations_in_velodyne_body)
print alignment
slam_poses_imu_aligned = map(lambda x: alignment*x, slam_poses)

aligned_poses_file = open(sys.argv[4], "w")
out_imu_poses_file = open(sys.argv[5], "w")

for i,(imu_p,slam_p) in enumerate(zip(imu_orientations_in_velodyne_body, slam_poses_imu_aligned)):
    aligned_poses_file.write("%s\n" %(slam_p,))

    imu_p.M[0:3, 3] = slam_p.M[0:3, 3]
    imu_p.setDofFromM()
    out_imu_poses_file.write("%s\n" %(imu_p,))
