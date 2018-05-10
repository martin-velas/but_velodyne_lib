#! /usr/bin/env python

# ~/workspace/but_velodyne_lib/scripts/poses_to_graph.py < 03-poses-cls-m10.txt > 04-cls-subseq.unclosed.graph; ~/workspace/but_velodyne_lib/scripts/pose_to_edge.py --src_index_from 253 --src_index_to 495 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.1-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph; cat loop.txt >>04-cls-subseq.unclosed.graph; ~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i 04-cls-subseq.unclosed.graph --pose-only --no-detailed-timing; ~/workspace/but_velodyne_lib/bin/slampp-solution-to-poses < solution.txt > 04-cls-subseq.closed.txt; ~/workspace/but_velodyne_lib/bin/build-3d-model -p 04-cls-subseq.closed.txt $(ls fixed-by-03-poses-cls-m10/*.pcd | sort) -o 04-cls-subseq.closed.pcd; pcl_viewer 04-cls-subseq.closed.pcd.rgb.pcd 

import argparse
from odometry_cnn_data import load_kitti_poses
from pose_graph import EdgesGenerator


parser = argparse.ArgumentParser(description="Pose from subsequence registration to EDGE3D")
parser.add_argument("--src_index_from", dest="src_index_from", type=int, required=True)
parser.add_argument("--src_index_to", dest="src_index_to", type=int, required=True)
parser.add_argument("--trg_index_from", dest="trg_index_from", type=int, required=True)
parser.add_argument("--trg_index_to", dest="trg_index_to", type=int, required=True)
parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
parser.add_argument("-o", "--original_poses", dest="original_poses", type=str, required=False)
parser.add_argument("-r", "--registration_pose", dest="registration_pose", type=str, required=True)
parser.add_argument("-g", "--graph_file", dest="graph_file", type=str, required=True)
args = parser.parse_args()

reg_poses = load_kitti_poses(args.registration_pose)
assert len(reg_poses) == 1
reg_pose = reg_poses[0]
poses = load_kitti_poses(args.poses)

if hasattr(args, "original_poses") and args.original_poses:
    original_poses = load_kitti_poses(args.original_poses)
else:
    original_poses = poses

generator = EdgesGenerator(poses, original_poses, reg_pose, args.graph_file)
edges = generator.genEdges(args.src_index_from, args.src_index_to, args.trg_index_from, args.trg_index_to)
for e in edges:
    print e
