#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses, Odometry
import numpy as np

VECTOR_LEN = 1
GRAVITY = np.asarray([0, VECTOR_LEN, 0]).reshape((3,1))


def get_down_vector(pose):
    return np.squeeze(np.asarray(pose.M[0:3,0:3]*GRAVITY))


if len(sys.argv) != 3:
    sys.stderr.write("ERROR, expected arguments: [slam-poses.txt] [imu-poses.txt]\n")
    sys.exit(1)

slam_poses = load_kitti_poses(sys.argv[1])
imu_poses = load_kitti_poses(sys.argv[2])
transformation = np.identity(4)

slam_points = np.zeros((len(slam_poses), 3))
imu_points = np.zeros((len(slam_poses), 3))

for _ in range(100):

    for i, (slam_p, imu_p) in enumerate(zip(slam_poses, imu_poses)):
        slam_p_transformed = np.matmul(transformation, slam_p.M)
        slam_points[i, :] = (slam_p_transformed[0:3,0:3]*GRAVITY + slam_p_transformed[0:3,3]).ravel()
        imu_points[i, :] = (imu_p.M[0:3,0:3]*GRAVITY + slam_p_transformed[0:3,3]).ravel()

    A = np.matmul(slam_points.T, imu_points)
    [U, _, Vt] = np.linalg.svd(A)
    det = np.linalg.det(np.matmul(Vt.T, U.T))

    E = np.identity(3)
    E[2, 2] = 1.0 if (det >= 0) else -1.0

    transformation[0:3, 0:3] = np.matmul(np.matmul(Vt.T, E), U.T)

for p in slam_poses:
    po = Odometry()
    po.M = np.matmul(transformation, p.M)
    print po
