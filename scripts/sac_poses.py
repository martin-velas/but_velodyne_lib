#! /usr/bin/env python

import argparse
import sys
from odometry_cnn_data import load_kitti_poses, pose_magnitude
from averageQuaternions import averagePoses


def get_inliers(picked_pose, all_poses, inliers_portion, ref_distance):
    picked_pose_inv = picked_pose.inv()
    errors = [pose_magnitude(picked_pose_inv * p, ref_distance) for p in all_poses]
    errors_poses = zip(errors, all_poses)
    errors_poses.sort(key=lambda x: x[0])
    errors_poses = errors_poses[0: int(len(errors_poses)*inliers_portion)]
    return sum([e for e, _ in errors_poses]), [p for _, p in errors_poses]


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="SAC for pick the representative for the poses.")
    parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
    parser.add_argument("-d", "--ref_distance", dest="ref_distance", type=float, required=True)
    parser.add_argument("-i", "--inliers_portion", dest="inliers_portion", type=float, required=True)
    args = parser.parse_args()


    poses = load_kitti_poses(args.poses)

    min_error = 1e10
    best_inliers = []
    for representative in poses:
        error, inliers = get_inliers(representative, poses, args.inliers_portion, args.ref_distance)

        if error < min_error:
            min_error = error
            best_inliers = inliers

    for i in best_inliers:
        sys.stderr.write("%s\n" % (i,))

    print averagePoses(best_inliers)
