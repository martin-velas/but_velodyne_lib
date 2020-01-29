#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min
from eulerangles_lib import eulerXYZ2angle_axis

from odometry_cnn_data import horizontal_split
from odometry_cnn_data import schema_to_dic
from odometry_cnn_data import odom_rad_to_deg
from odometry_cnn_data import odom_deg_to_rad
from odometry_cnn_data import Odometry
from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import gen_preserve_mask
from odometry_cnn_data import mask_list

import cv_yaml

class OutputFiles:
    def __init__(self, batch_size, history_size, frames_to_join, odometries_to_join, features, division, overlay, output_prefix, 
                 max_seq_len, min_in_odom_schema):
        self.batchSize = batch_size
        self.historySize = history_size
        self.framesToJoin = frames_to_join
        self.odometriesToJoin = odometries_to_join
        self.horizontalDivision = division
        self.horizontalOverlay = overlay
        self.features = features
        self.outputPrefix = output_prefix
        self.maxFramesPerFile = max_seq_len
        self.outFileSeqIndex = -1
        self.minInOdomSchema = min_in_odom_schema
    
    def newSequence(self, frames_count):
        framesCountUnaligned = (frames_count - self.minInOdomSchema)
        self.framesToWriteCount = framesCountUnaligned - (framesCountUnaligned % self.batchSize)
        self.outFileSeqIndex += 1
        self.out_files = []
        out_files_count = self.framesToWriteCount / self.maxFramesPerFile + (1 if self.framesToWriteCount % self.maxFramesPerFile > 0 else 0)
        for split_index in range(out_files_count):
            if (split_index + 1) * self.maxFramesPerFile <= self.framesToWriteCount:
                frames_in_file = self.maxFramesPerFile  
            else:
                frames_in_file = self.framesToWriteCount % self.maxFramesPerFile
            new_output_file = h5py.File(self.outputPrefix + "." + str(self.outFileSeqIndex) + "." + str(split_index) + ".hdf5", 'w')
            new_output_file.create_dataset('data', (frames_in_file * self.historySize,
                                                    self.features * self.framesToJoin * self.horizontalDivision,
                                                    FRAME_HEIGHT,
                                                    FRAME_WIDTH / self.horizontalDivision + self.horizontalOverlay * 2), dtype='f4')
            new_output_file.create_dataset('odometry', (frames_in_file, 6*self.odometriesToJoin), dtype='f4')
            self.out_files.append(new_output_file)
    
    def putData(self, db_name, frame_i, ch_i, data):
        if db_name == 'odometry':
            multiply = 1 
        else:
            multiply = self.historySize
        if frame_i < (self.framesToWriteCount) * multiply:
            file_index = frame_i / (self.maxFramesPerFile * multiply)
            self.out_files[file_index][db_name][frame_i % (self.maxFramesPerFile * multiply), ch_i] = data
        else:
            sys.stderr.write("Warning: frame %s out of the scope\n" % frame_i)

    def close(self):
        for f in self.out_files:
            f.close()

BATCH_SCHEMA_DATA = [[0, 1]]
BATCH_SCHEMA_ODOM = [[1]]

CUMMULATE_ODOMS = 1
ODOMS_UNITS = "deg" # rad or deg
ROT_TYPE = "euler"  # euler or "axis-angle"
DOF_WEIGHTS = [1.0] * 6

BATCH_SIZE = len(BATCH_SCHEMA_ODOM)
JOINED_FRAMES = len(BATCH_SCHEMA_DATA[0])
JOINED_ODOMETRIES = len(BATCH_SCHEMA_ODOM[0])
FEATURES = 3
FRAME_HEIGHT=64
FRAME_WIDTH=1800
HISTORY_SIZE = len(BATCH_SCHEMA_DATA) / BATCH_SIZE
max_in_data_schema = max(reduce(lambda x, y: x + y, BATCH_SCHEMA_DATA))
min_in_odom_schema = min(reduce(lambda x, y: x + y, BATCH_SCHEMA_ODOM))

MIN_SKIP_PROB = 0.0
MAX_SKIP_PROB = 0.09
STEP_SKIP_PROB = 0.8
MAX_SPEED = 60 / 3.6
FILES_PER_HDF5 = 96    # 3*5*16

HORIZONTAL_DIVISION = 1  # divide into the 4 cells
HORIZONTAL_DIVISION_OVERLAY = 0  # 19deg    =>    128deg per divided frame
CHANNELS = FEATURES * HORIZONTAL_DIVISION

# ZNORM_MEAN = [-5.11542848e-04, -1.75650713e-02, 9.54909532e-01, -5.55075555e-05, 4.41994586e-04, 1.85761792e-06]
# ZNORM_STD_DEV = [0.024323927907799907, 0.017388835121575155, 0.43685033540416063, 0.003018560507387704, 0.017281427121920292, 0.002632398885115511]
ZNORM_MEAN = [0] * 6
ZNORM_STD_DEV = [1] * 6

def znorm_odom(odom):
    result = Odometry()
    result.dof = [(odom.dof[i] - ZNORM_MEAN[i]) / ZNORM_STD_DEV[i] for i in range(6)]
    result.setMFromDof()
    return result

def odom_cumulate(odometries, N):
    cumulated = []
    for dest_i in range(len(odometries)):
        cum_odom = Odometry()
        for cum_i in range(dest_i-N+1, dest_i+1):
            cum_odom = cum_odom * odometries[cum_i]
        cumulated.append(cum_odom)
    return cumulated

def euler_rad_to_axis_angle(dof):
    aa = eulerXYZ2angle_axis(dof[3], dof[4], dof[5])
    return dof[0:3] + (aa[1]*aa[0]).tolist()

if len(sys.argv) < 2 + max_in_data_schema + 1:
    sys.stderr.write("Expected arguments: <pose-file> <out-file-prefix> <frames.yaml>^{%s+}\n" % JOINED_FRAMES)
    sys.exit(1)

poses_6dof = load_kitti_poses(sys.argv[1])

random.seed()
skip_prob = MIN_SKIP_PROB
out_files = OutputFiles(BATCH_SIZE, HISTORY_SIZE, JOINED_FRAMES, JOINED_ODOMETRIES, FEATURES, 
                        HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, sys.argv[2], FILES_PER_HDF5,
                        min_in_odom_schema)
data_dest_index = schema_to_dic(BATCH_SCHEMA_DATA)
odom_dest_index = schema_to_dic(BATCH_SCHEMA_ODOM)
while skip_prob < MAX_SKIP_PROB:
    mask = gen_preserve_mask(poses_6dof, skip_prob, MAX_SPEED)
    # TODO - maybe also duplication = no movement

    out_files.newSequence(sum(mask))
    files_to_use = mask_list(sys.argv[3:], mask)
    odometry_to_use = get_delta_odometry(mask_list(poses_6dof, mask))
    odometry_to_use = odom_cumulate(odometry_to_use, CUMMULATE_ODOMS)
    odometry_to_use = map(znorm_odom, odometry_to_use)

    for i in range(len(files_to_use)):
        if files_to_use[i].endswith(".npy"):
            data_i = np.load(files_to_use[i])
        else:
            data_i = cv_yaml.load(files_to_use[i], 'range-y-intensity')
        data_i = np.moveaxis(data_i, [0, 1, 2], [1, 2, 0])
        data_i = horizontal_split(data_i, HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, FEATURES, FRAME_HEIGHT, FRAME_WIDTH)

        if ROT_TYPE == "axis-angle":
            odometry_i = euler_rad_to_axis_angle(odometry_to_use[i].dof)
        elif ROT_TYPE == "euler":
            odometry_i = odometry_to_use[i].dof
        else:
            raise ValueError("Unknown rotation representaiton '%s'" % ROT_TYPE)

        if ODOMS_UNITS == "deg":
            odometry_i = odom_rad_to_deg(odometry_i)
        elif ODOMS_UNITS != "rad":
            raise ValueError("Unknown odometry units '%s'" % ODOMS_UNITS) 

        odometry_i = [odometry_i[j]*DOF_WEIGHTS[j] for j in range(len(odometry_i))]

        bias = 0
        while i - bias >= 0:
            # print "i", i, "bias", bias
            schema_i = i - bias
            # for feature data
            if schema_i in data_dest_index:
                for slot_frame_i in data_dest_index[schema_i]:
                    slot_i = slot_frame_i["slot"]
                    frame_i = slot_frame_i["frame"]
                    for fi in range(CHANNELS):
                        out_files.putData('data', frame_i + bias * HISTORY_SIZE, slot_i * CHANNELS + fi, data_i[fi])
            # for odometry data
            if schema_i in odom_dest_index:
                for slot_frame_i in odom_dest_index[schema_i]:
                    slot_i = slot_frame_i["slot"]
                    frame_i = slot_frame_i["frame"]
                    for ch_i in range(len(odometry_i)):
                        out_files.putData('odometry', frame_i + bias, slot_i * len(odometry_i) + ch_i, odometry_i[ch_i])
            
            bias += BATCH_SIZE
        
        if i % FILES_PER_HDF5 == 0 and i > 0:
            print i, "/", len(files_to_use)
    
    skip_prob += STEP_SKIP_PROB
    out_files.close()
