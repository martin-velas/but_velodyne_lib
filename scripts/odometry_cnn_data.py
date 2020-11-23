#! /usr/bin/env python

import numpy as np
import math
from math import sin, cos
import sys
import eulerangles_lib
import random

from transformations import matrix_to_axis_angle


VELO_TO_IMU = np.array([0,0,1,0,
                        1,0,0,0,
                        0,1,0,0,
                        0,0,0,1]).reshape((4,4))


def roll_to_matrix(roll_angle):
    sr = sin(roll_angle)
    cr = cos(roll_angle)
    M = np.eye(3)
    M[1,1] = cr; M[1,2] = -sr
    M[2,1] = sr; M[2,2] = cr
    return M


def pitch_to_matrix(pitch_angle):
    sp = sin(pitch_angle)
    cp = cos(pitch_angle)
    M = np.eye(3)
    M[0,0] = cp; M[0,2] = sp
    M[2,0] = -sp; M[2,2] = cp
    return M


def heading_to_matrix(heading_angle):
    sh = sin(heading_angle)
    ch = cos(heading_angle)
    M = np.eye(3)
    M[0,0] = ch; M[0,1] = -sh
    M[1,0] = sh; M[1,1] = ch
    return M


def roll_pitch_heading_to_matrix(roll, pitch, heading):
    # this order is verified (H*P*R)
    return np.dot(heading_to_matrix(heading), np.dot(pitch_to_matrix(pitch), roll_to_matrix(roll)))


def omega_phi_kappa_to_matrix(omega, phi, kappa):
    # this order is verified (O*P*K)
    return np.dot(roll_to_matrix(omega), np.dot(pitch_to_matrix(phi), heading_to_matrix(kappa)))


def odom_rad_to_deg(odom):
    return odom[0:3] + [odom[i]*180.0/math.pi for i in range(3, 6)]

def odom_deg_to_rad(odom):
    return odom[0:3] + [odom[i]*math.pi/180.0 for i in range(3, 6)]

def dof2matrix(dof):
    M = np.matrix(np.eye(4, 4))
    M[:3, :3] = np.transpose(eulerangles_lib.euler2matXYZ(-dof[3], -dof[4], -dof[5]))
    M[0, 3], M[1, 3], M[2, 3] = dof[0], dof[1], dof[2]
    return M

def matrix2dof(M):
    R = np.transpose(M[:3, :3])
    dof = [0]*6
    dof[0], dof[1], dof[2] = M[0, 3], M[1, 3], M[2, 3]
    dof[5], dof[4], dof[3] = map(lambda x: -x, eulerangles_lib.mat2eulerZYX(R))
    return dof

def kitti_pose_string(matrix):
    output = ""
    for i in range(12):
        output += str(matrix[i / 4, i % 4]) + " "
    return output[:-1]


class Odometry:
    def __init__(self, kitti_pose=[1, 0, 0, 0,
                                     0, 1, 0, 0,
                                     0, 0, 1, 0]):
        assert len(kitti_pose) == 12
        self.dof = [0] * 6
        self.M = np.matrix([[0] * 4, [0] * 4, [0] * 4, [0, 0, 0, 1]], dtype=np.float64)
        for i in range(12):
            self.M[i / 4, i % 4] = kitti_pose[i]
        self.setDofFromM()

    def setDofFromM(self):
        self.dof = np.array(matrix2dof(self.M))

    def setMFromDof(self):
        self.M = dof2matrix(self.dof)

    def distanceTo(self, other):
        sq_dist = 0
        for i in range(3):
            sq_dist += (self.dof[i] - other.dof[i]) ** 2
        return math.sqrt(sq_dist)

    def move(self, dof, inverse = False):
        assert len(dof) == 6
        Rt = dof2matrix(dof)
        if inverse:
            Rt = np.linalg.inv(Rt)
        self.M = np.dot(self.M, Rt)
        self.setDofFromM()
        return self

    def __mul__(self, other):
        out = Odometry()
        out.M = np.dot(self.M, other.M)
        out.setDofFromM()
        return out

    def __sub__(self, other):
        out = Odometry()
        out.M = np.dot(np.linalg.inv(other.M), self.M)
        out.setDofFromM()
        return out

    def __str__(self):
        return kitti_pose_string(self.M)

    def inv(self):
        inverted = Odometry()
        inverted.M = np.linalg.inv(self.M)
        inverted.setDofFromM()
        return inverted

    def transform_pt(self, pt):
        pt_col = np.transpose(np.matrix(pt + [1]))
        pt_transformed = np.dot(self.M, pt_col)
        return np.array(np.transpose(pt_transformed)).flatten().tolist()

    def getDofAxisAngles(self):
        axis, angle = matrix_to_axis_angle(self.M)
        return [self.dof[i] for i in range(3)] + [axis[i]*angle for i in range(3)]


def load_kitti_poses(file):
    poses = []
    if not hasattr(file, "readlines"):
        file = open(file)
    for line in file.readlines():
        kitti_pose = map(float, line.strip().split())
        o = Odometry(kitti_pose)
        poses.append(o)
    return poses

def load_poses_corrections(in_file):
    corrections = []
    if not hasattr(in_file, "readlines"):
        in_file = open(in_file)
    for line in in_file.readlines():
        tokens = line.split()
        src_i, trg_i = map(int, tokens[0:2])
        kitti_pose = map(float, tokens[2:14])
        o = Odometry(kitti_pose)
        corrections.append({"src_i":src_i, "trg_i":trg_i, "pose":o})
    return corrections

def get_delta_odometry(odometries):
    output = [Odometry()]
    for i in range(1, len(odometries)):
        output.append(odometries[i] - odometries[i-1])
    return output

def remove_ones_sequences(mask):
    preserve = [1]*len(mask)
    preserve[0] = 0 if (mask[0] == 1) and (mask[1] == 1) else 1
    preserve[-1] = 0 if (mask[-2] == 1) and (mask[-1] == 1) else 1
    for i in range(1, len(mask)-1):
        if (mask[i-1] == 1) and (mask[i] == 1) and (mask[i+1] == 1):
            preserve[i] = 0
    return [mask[i]*preserve[i] for i in range(len(mask))]

def gen_preserve_mask(poses, skip_prob, max_speed):
    if skip_prob == 0:
        return [1]*len(poses)
    mask = [1]
    prev_pose = poses[0]
    current_pose = poses[1]
    for next_pose in poses[2:]:
        distance = next_pose.distanceTo(prev_pose)
        rndnum = random.random()
        if (distance < max_speed * 0.1) and (rndnum < skip_prob):
            mask.append(0)
        else:
            mask.append(1)
            prev_pose = current_pose
        current_pose = next_pose
    mask.append(1)
#    print "original mask:", mask
#    mask = remove_ones_sequences(mask)
#    print "filtered mask:", mask
    return mask

def mask_list(list, mask):
    if len(list) != len(mask):
        sys.stderr.write("Number of poses (%s) and velodyne frames (%s) differ!\n" % (len(mask), len(list)))
    output = []
    for i in range(min(len(mask), len(list))):
        if mask[i] != 0:
            output.append(list[i])
    return output


def poses_diff(A, B, distance):
    delta = A.inv() * B
    return pose_magnitude(delta, distance)


def pose_magnitude(P, distance):
    axis, angle = matrix_to_axis_angle(P.M)
    return (math.tan(angle) * distance) + np.linalg.norm(P.M[0:3, 3])


if __name__ == "__main__":
    dof = [1, 2, 3, -4.807891009/57.3, -20.324063515/57.3, -5.047515356/57.3]
    print dof
    M = dof2matrix(dof)
    print M
    dof = matrix2dof(M)
    print dof

    odom = Odometry().move(dof)
    t_dof = [2, 2, 2, 0, 0, 0]

    t_odom = Odometry().move(t_dof)
    print odom
    print t_odom
    print t_odom * odom

    t_odom = Odometry()
    t_odom.dof = t_dof
    t_odom.setMFromDof()
    odom = Odometry()
    odom.dof = dof
    odom.setMFromDof()
    print odom
    print t_odom
    print t_odom * odom

    print type(odom.M)

    print "--------------------------------------------------------------------------------"

    print odom
    print odom.dof
    print odom.getDofAxisAngles()
