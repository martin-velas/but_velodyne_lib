#! /usr/bin/env python

import argparse
import sys
from odometry_cnn_data import load_kitti_poses
from pose_graph import Edge3D


def get_graph_index(pose_index, poses_cnt, sensor_index):
    return pose_index + poses_cnt*(1 + sensor_index)


class EdgePoseToLandmark:
    def __init__(self, src_id, trg_id, position, inf_matrix_diagonal):
        self.srcId = src_id
        self.trgId = trg_id
        self.position = position
        self.inf_matrix_upper_half = [inf_matrix_diagonal[0], 0.0, 0.0,
                                           inf_matrix_diagonal[1], 0.0,
                                                 inf_matrix_diagonal[2]]

    def __gt__(self, other):
        if self.srcId > other.srcId:
            return True
        elif self.srcId < other.srcId:
            return False
        else:
            return self.trgId > other.trgId

    def __str__(self):
        output = "EDGE_SE3_XYZ %s %s " % (self.srcId, self.trgId)
        for d in self.position:
            output += "%s " % d
        for m in self.inf_matrix_upper_half:
            output += "%s " % m
        return output


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Matches to pose graph")
    parser.add_argument("-s", "--src_index", dest="src_index", type=int, required=True)
    parser.add_argument("-t", "--trg_index", dest="trg_index", type=int, required=True)
    parser.add_argument("-p", "--poses_cnt", dest="poses_cnt", type=int, required=True)
    parser.add_argument("-i", "--index_to_start", dest="index_to_start", type=int, required=True)
    parser.add_argument("-wm", "--weights_matches", nargs='+', dest="weights_matches", type=float, help='The diagonal of information matrix for matches', required=False)
    args = parser.parse_args()

    weights_matches = [100.0, 100.0, 100.0] if args.weights_matches is None else args.weights_matches

    next_vertex = args.index_to_start
    for line in sys.stdin.readlines():
        tokens = line.split()
        src_i = int(tokens[0])
        src_landmark = map(float, tokens[1:4])
        trg_i = int(tokens[4])
        trg_landmark = map(float, tokens[5:8])
        print EdgePoseToLandmark(get_graph_index(args.src_index, args.poses_cnt, src_i), next_vertex, src_landmark, weights_matches)
        print EdgePoseToLandmark(get_graph_index(args.trg_index, args.poses_cnt, trg_i), next_vertex, trg_landmark, weights_matches)
        next_vertex += 1
