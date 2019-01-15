#! /usr/bin/env python

import argparse
from odometry_cnn_data import load_kitti_poses
from pose_graph import Edge3D
from matches_to_graph import get_graph_index


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Matches to pose graph")
    parser.add_argument("-p", "--poses_cnt", dest="poses_cnt", type=int, required=True)
    parser.add_argument("-c", "--calibration", dest="calibration", type=str, required=True)
    parser.add_argument("-wc", "--weights_calibration", nargs='+', dest="weights_calibration", type=float, help='The diagonal of information matrix for calibration', required=False)
    args = parser.parse_args()

    calibration = load_kitti_poses(args.calibration)

    weights_calibration = [10000.0, 10000.0, 10000.0, 10000000.0, 10000000.0, 10000000.0] if args.weights_calibration is None else args.weights_calibration

    for ci, c in enumerate(calibration):
        for pi in range(args.poses_cnt):
            print Edge3D(pi, get_graph_index(pi, args.poses_cnt, ci), c.dof, weights_calibration)
