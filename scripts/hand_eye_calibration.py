#! /usr/bin/env python

import sys, math
from math import sin, cos, acos
import numpy as np

from odometry_cnn_data import load_kitti_poses, Odometry
from transformations import quaternion_from_matrix, quaternion_matrix
from numpy_point_cloud import *

def axis_angle_to_quaternion(axis, angle):
    return [cos(angle / 2.0)] + [x * sin(angle / 2.0) for x in axis]


def quaternion_to_axis_angle(quaternion):
    if abs(quaternion[0] - 1.0) < 1e-6:
        return [1.0, 0.0, 0.0], 0.0
    theta = 2*acos(quaternion[0])
    return quaternion[1:] / sin(theta / 2.0), theta


def split_Rt(transformation):
    return transformation.M[:3, :3], np.squeeze(np.asarray(transformation.M[0:3, 3]))


def matrix_to_axis_angle(R):
    quaternion = quaternion_from_matrix(np.transpose(R))
    return quaternion_to_axis_angle(quaternion)


def build_eq(nA, nB):
    a = nA + nB
    A_part = np.asarray([0,   -a[2], a[1],
                         a[2], 0,   -a[0],
                        -a[1], a[0], 0]).reshape((3,3))
    return A_part, (nA - nB)


def trajectories_to_calibration(poses1, poses2):
    assert len(poses1) == len(poses2)

    halfway_idx = len(poses1) / 2
    A = np.zeros((3 * halfway_idx, 3))
    b = np.zeros((3 * halfway_idx))
    for i in range(halfway_idx):
        delta1 = poses1[i + halfway_idx] - poses1[i]
        delta2 = poses2[i + halfway_idx] - poses2[i]

        _, t1 = split_Rt(delta1)
        _, t2 = split_Rt(delta2)
        nA = t1 / np.linalg.norm(t1)
        nB = t2 / np.linalg.norm(t2)

        Ai, bi = build_eq(nA, nB)
        A[3 * i: 3 * (i + 1), :], b[3 * i: 3 * (i + 1)] = Ai, bi

    nx, residual, rank, singular = np.linalg.lstsq(A, b)
    nx *= -1  # from B->A to A->B
    nx_norm = np.linalg.norm(nx)
    theta = 2 * math.atan(nx_norm)
    nx /= nx_norm

    print "residual:", residual[0]
    # print "axis-angle:", nx, theta
    quaternion = axis_angle_to_quaternion(nx, theta)
    print "quaternion:", quaternion

    p1_avg = np.zeros(3)
    p2_avg = np.zeros(3)
    for p1,p2 in zip(poses1, poses2):
        p1_avg += split_Rt(p1)[1]
        p2_avg += split_Rt(p2)[1]
    p1_avg /= len(poses1)
    p2_avg /= len(poses2)

    calibration = Odometry()
    R = quaternion_matrix(quaternion)[0:3, 0:3]
    calibration.M[0:3, 0:3] = R
    calibration.M[0:3, 3] = (p1_avg - np.dot(R, p2_avg)).reshape((3,1))
    calibration.setDofFromM()
    return calibration


if len(sys.argv) != 3:
    sys.stderr.write("ERROR, 2 pose files are required!\n")
    sys.exit(1)

poses1 = load_kitti_poses(sys.argv[1])
# translation: [0.0113782870643, 0.118657435794, 0.173699919874]
# rotation (xyzw): [0.0362682036379, -0.803976727751, 0.593376488207, 0.0145045469662]
c = Odometry([-0.996948, -0.075531, 0.019719, 0.011378,
              -0.041104, 0.293178, -0.955174, 0.118658,
              0.066364, -0.953070, -0.295388, 0.173698])
c_axis = [-2.50047713, 55.42915989, -40.90959752]
poses2_test = [c.inv() * p * c for p in poses1]
poses2 = load_kitti_poses(sys.argv[2])

calibration = Odometry()
for _ in range(10):
    subcalibration = trajectories_to_calibration(poses1, poses2)
    poses2 = map(lambda x: subcalibration * x * subcalibration.inv(), poses2)
    calibration = subcalibration*calibration
    print calibration.M

print calibration
