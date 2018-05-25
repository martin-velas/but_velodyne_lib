# -*- coding: utf-8 -*-

# Expecting KITTI like form of transformation matrix:
# r1 r2 r3 tx
# r4 r5 r6 ty
# r7 r8 r9 tz
# 0  0  0  1

# transformations operates with transposed matrices
import transformations

import numpy as np


# matrix is full Rt of just 3x3 R KITTI like matrix
# returns [qw, qx, qy, qz]
def matrix_to_quaternion(matrix):
    q = transformations.quaternion_from_matrix(matrix)
    return q


# Q = [qw, qx, qy, qz]
# returns 3x3 R KITTI like matrix
def quaternion_to_matrix(quaternion):
    return transformations.quaternion_matrix(quaternion)


if __name__ == "__main__":
    M = np.matrix([[1, 0, 0, 0],
                   [0, 0,-1, 0],
                   [0, 1, 0, 0],
                   [0, 0, 0, 1]])
    Q = matrix_to_quaternion(M)
    print Q
    assert np.isclose(Q[0], 0.70710678) and np.isclose(Q[1], 0.70710678) and np.isclose(Q[2], 0.0) and np.isclose(Q[3], 0.0)

    Mnew = quaternion_to_matrix(Q)
    print Mnew
    assert np.array_equal(M, Mnew)

    Qeye = np.asarray([1, 0, 0, 0])
    Meye = quaternion_to_matrix(Qeye)
    print Meye
    assert np.array_equal(Meye, np.identity(4))
