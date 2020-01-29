#! /usr/bin/env python

from odometry_cnn_data import load_kitti_poses, get_delta_odometry
import argparse
import numpy as np


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Get trajectory length")
    parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
    parser.add_argument("-i", "--indices", dest="indices", type=str, required=True)
    args = parser.parse_args()

    deltas = get_delta_odometry(load_kitti_poses(args.poses))[1:]
    deltas = map(lambda x: np.linalg.norm(x.M[0:3,3]), deltas)

    for line in open(args.indices):
        src, trg = map(int, line.split()[0:2])
        trajectory = 0.0
        for i in range(src, trg):
            trajectory += deltas[i]

        print trajectory
