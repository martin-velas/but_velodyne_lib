#! /usr/bin/env python
# -*- coding: utf-8 -*-

# This file implements correct quaternion averaging.
#
# This method is computationally expensive compared to naive mean averaging.
# If only low accuracy is required (or the quaternions have similar orientations),
# then quaternion averaging can possibly be done through simply averaging the
# components.
#
# Based on:
#
# Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman.
# "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
# no. 4 (2007): 1193-1197.
# Link: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
#
# Code based on:
#
# Tolga Birdal. "averaging_quaternions" Matlab code.
# http://jp.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging-quaternions
#
# Comparison between different methods of averaging:
#
# Claus Gramkow. "On Averaging Rotations"
# Journal of Mathematical Imaging and Vision 15: 7–16, 2001, Kluwer Academic Publishers.
# https://pdfs.semanticscholar.org/ebef/1acdcc428e3ccada1a047f11f02895be148d.pdf
#
# Side note: In computer graphics, averaging or blending of two quaternions is often done through
# spherical linear interploation (slerp). Even though it's often used it might not be the best
# way to do things, as described in this post:
#
# Jonathan Blow.
# "Understanding Slerp, Then Not Using It", February 2004
# http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
#

import numpy
import numpy.matlib as npm
from odometry_cnn_data import load_kitti_poses, Odometry
import sys
from transformations_kittilike import matrix_to_quaternion, quaternion_to_matrix

# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = numpy.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


# Average multiple quaternions with specific weights
# The weight vector w must be of the same length as the number of rows in the
# quaternion maxtrix Q
def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * numpy.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


if __name__ == "__main__":
    poses = load_kitti_poses(sys.stdin)
    quaternions = numpy.zeros((len(poses), 4))
    for i,p in enumerate(poses):
        quaternions[i] = matrix_to_quaternion(p.M)

    avg = averageQuaternions(quaternions)

    o = Odometry()
    o.M = quaternion_to_matrix(avg)
    o.setDofFromM()
    print o
