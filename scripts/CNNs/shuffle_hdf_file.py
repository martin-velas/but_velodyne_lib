#! /usr/bin/env python

import sys
import math
import numpy as np
import h5py
import random

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry

BATCH_SIZE = 1
OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

class Line:
    def __init__(self, k, q):
        self.k = k
        self.q = q
        
    def y(self, x):
        return self.k*x + self.q

def data_index_from_odom(odom_i, hist_size, batch_size, hist_i):
    batch_i = odom_i / batch_size
    slot_i = odom_i % batch_size
    return batch_i*batch_size*hist_size + slot_i + hist_i*batch_size

for filename in sys.argv[1:]:
    indices = []
    with h5py.File(filename, "r") as hf:
        data = hf["data"]
        odometry = hf["odometry"]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count
        for i in range(odom_count):
            angle = np.linalg.norm(odometry[i][3:6])
            indices += [i]
        indices_count = len(indices)/BATCH_SIZE * BATCH_SIZE
        indices = indices[:indices_count]
        random.shuffle(indices)
        output_file = h5py.File(filename + ".shuffled", 'w')
        output_file.create_dataset('data', (len(indices)*hsize,) + np.shape(data)[1:], dtype='f4')
        output_file.create_dataset('odometry', (len(indices),) + np.shape(odometry)[1:], dtype='f4')

        rot_classes = {}
        if OUT_LAYERS[0] in hf:
            for rot_layer in OUT_LAYERS:
                output_file.create_dataset(rot_layer, (len(indices),), dtype='f4')
                rot_classes[rot_layer] = hf[rot_layer][:]

        for new_i in range(len(indices)):
            output_file['odometry'][new_i] = odometry[indices[new_i]]
            if len(rot_classes) > 0:
                for rot_layer in OUT_LAYERS:
                    output_file[rot_layer][new_i] = rot_classes[rot_layer][indices[new_i]]
            for h in range(hsize):
                from_i = data_index_from_odom(indices[new_i], hsize, BATCH_SIZE, h)
                to_i = data_index_from_odom(new_i, hsize, BATCH_SIZE, h)
                output_file['data'][to_i] = data[from_i]
