#! /usr/bin/env python

import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import numpy
import random
import itertools

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import gen_preserve_mask
from odometry_cnn_data import mask_list

# for nomal data
# ROTATIONS_BINS = [20, 100, 20]
# MAX_ROTATIONS = [r * 180 / math.pi for r in [0.025, 0.1, 0.02]]  # deg

# for yaw by comparison
#ROTATIONS_BINS = [12, 56, 12]
#MAX_ROTATIONS = [1.5, 5.6, 1.5]  # deg

ROTATIONS_BINS = [20,100,20]
MAX_ROTATIONS = [1.5,6.0,1.5]  # deg

# for skipped frames
# ROTATIONS_BINS = [20, 200, 20]
# MAX_ROTATIONS = [r*180/math.pi for r in [0.02, 0.2, 0.02]]     # to deg

OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

def get_bin_sizes(max_rotations, rotation_bins):
    return [(max_rotations[i] / rotation_bins[i]) * 2.0 for i in range(3)]

def display_histograms(odometries, bins):
    rotations = {}
    rotations["x"] = [o.dof[3]*180.0/math.pi for o in odometries]
    rotations["y"] = [o.dof[4]*180.0/math.pi for o in odometries]
    rotations["z"] = [o.dof[5]*180.0/math.pi for o in odometries]

    for key, rots in rotations.iteritems():
        rots.sort()
        rots = rots[int(0.01*len(rots)):int(0.99*len(rots))]
        plt.title("Histogram of angles %s" % (key))
        plt.hist(rots, bins=100)  # plt.hist passes it's arguments to np.histogram
        plt.show()
        plt.clf()

def round_angles(odom):
    bin_sizes_rad = [bs / 180.0 * math.pi for bs in BIN_SIZES]
    for i in range(3):
        odom.dof[i + 3] = round(odom.dof[i + 3] / bin_sizes_rad[i]) * bin_sizes_rad[i]
    odom.setMFromDof()
    return odom

def round_pose_files():
    print MAX_ROTATIONS, BIN_SIZES
    for file in sys.argv[1:]:
        odoms = map(round_angles, get_delta_odometry(load_kitti_poses(file)))
        with open(file + ".round", "w") as out_file:
            pose = Odometry()
            for o in odoms:
                pose.move(o.dof)
                out_file.write("%s\n" % (pose))

def angles2classes(angles, max_rotations=MAX_ROTATIONS, rotation_bins=ROTATIONS_BINS):
    classes = [0] * 3
    bin_sizes = get_bin_sizes(max_rotations, rotation_bins)
    for i in range(3):
        cls = round(angles[i] / (bin_sizes[i])) + rotation_bins[i] / 2
        cls = max(0, min(cls, rotation_bins[i] - 1))
        classes[i] = cls
    return classes

def class_i2angle(dimension_idx, class_idx, max_rotations, rotation_bins):
    bin_sizes = get_bin_sizes(max_rotations, rotation_bins)
    return (class_idx - rotation_bins[dimension_idx] / 2) * (bin_sizes[dimension_idx])

def get_best_prob_combination(probabilities, comb_size):
    best_start = -1
    best_end = -1
    best_prob = -1
    for i in range(len(probabilities)):
        start = max(0, i - comb_size / 2)
        end = min(len(probabilities), i + comb_size / 2 + 1)
        prob_slice = probabilities[start:end]
        avg_prob = sum(prob_slice) / len(prob_slice)
        if avg_prob > best_prob:
            best_prob = avg_prob
            best_start = start
            best_end = end
    mask = [0] * len(probabilities)
    for i in range(best_start, best_end):
        mask[i] = 1
    return mask

def weight_avg_angles(angles, probabilities, window_size):
    assert len(angles) == len(probabilities)
    angles_sum = 0
    prob_sum = 0
    mask = get_best_prob_combination(probabilities, window_size)
    for i in range(len(angles)):
        angles_sum += angles[i] * probabilities[i] * mask[i]
        prob_sum += probabilities[i] * mask[i]
    return angles_sum / prob_sum

def classes2single_angle(classes_probabilities, dim_i, max_rotations, rotation_bins, window_size):
    probabilities = []
    angles = []
    for cls_i in range(rotation_bins[dim_i]):
        prob = classes_probabilities[cls_i]
        angle = class_i2angle(dim_i, cls_i, max_rotations, rotation_bins)
        probabilities.append(prob)
        angles.append(angle)
    return weight_avg_angles(angles, probabilities, window_size)

def classes_blob2angles(predicion_blob, blob_i, out_indexes=[0, 1, 2], layer_names=OUT_LAYERS,
                        max_rotations=MAX_ROTATIONS, rotation_bins=ROTATIONS_BINS, window_size=7):
    rotations = [0] * 3
    for i in range(len(out_indexes)):
        classes_probabilities = predicion_blob[layer_names[i]][blob_i]
        dim_i = out_indexes[i]
        rotations[dim_i] = classes2single_angle(classes_probabilities, dim_i, max_rotations, rotation_bins, window_size)
    return rotations

if __name__ == "__main__":

#     odometries = []
#     for file in sys.argv[1:]:
#         poses = load_kitti_poses(file)
#         for prob in [0, 0.8]:
#             mask = gen_preserve_mask(poses, prob, 60*3.6)
#             odometries += get_delta_odometry(mask_list(poses, mask))
#     display_histograms(odometries, 100)
#     sys.exit(0)

    import h5py

    round_pose_files()
    sys.exit()

    for filename in sys.argv[1:]:
        with h5py.File(filename, "r") as in_hf:
            with h5py.File(filename + ".rotclasses", "w") as out_hf:
                data = in_hf["data"][:]
                odometries = in_hf["odometry"][:]
                odom_count = np.shape(odometries)[0]

                out_hf.create_dataset('data', np.shape(data), dtype='f4')
                out_hf.create_dataset('odometry', np.shape(odometries), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[0], (odom_count,), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[1], (odom_count,), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[2], (odom_count,), dtype='f4')

                out_hf['data'][...] = data
                out_hf['odometry'][...] = odometries

                for i in range(odom_count):
                    out_hf[OUT_LAYERS[0]][i], out_hf[OUT_LAYERS[1]][i], out_hf[OUT_LAYERS[2]][i] = angles2classes(odometries[i][3:])
