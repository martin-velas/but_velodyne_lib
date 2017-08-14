#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min
from eulerangles import mat2eulerZYX, euler2matXYZ
from odometry_cnn_data import horizontal_split
from odometry_cnn_data import schema_to_dic
import cv_yaml

class Odometry:
    def __init__(self, kitti_pose=[1, 0, 0, 0,
                                     0, 1, 0, 0,
                                     0, 0, 1, 0]):
        assert len(kitti_pose) == 12
        self.dof = [0] * 6
        self.M = np.matrix([[0] * 4, [0] * 4, [0] * 4, [0, 0, 0, 1]], dtype=np.float64)
        for i in range(12):
            self.M[i / 4, i % 4] = kitti_pose[i]
        self.setDofFromM()
    
    def setDofFromM(self):
        R = self.M[:3, :3]
        self.dof[0], self.dof[1], self.dof[2] = self.M[0, 3], self.M[1, 3], self.M[2, 3]
        self.dof[5], self.dof[4], self.dof[3] = mat2eulerZYX(R)

    def setMFromDof(self):
        self.M[:3, :3] = euler2matXYZ(self.dof[3], self.dof[4], self.dof[5])
        self.M[0, 3], self.M[1, 3], self.M[2, 3] = self.dof[0], self.dof[1], self.dof[2]
  
    def distanceTo(self, other):
        sq_dist = 0
        for i in range(3):
            sq_dist += (self.dof[i] - other.dof[i]) ** 2
        return math.sqrt(sq_dist)

    def __mul__(self, other):
        out = Odometry()
        out.M = self.M * other.M
        out.setDofFromM()
        return out
    
    def __sub__(self, other):
        out = Odometry()
        out.M = np.linalg.inv(other.M) * self.M
        out.setDofFromM()
        return out

def gen_preserve_mask(poses, skip_prob):
    mask = [1]
    prev_pose = poses[0]
    current_pose = poses[1]
    for next_pose in poses[2:]:
        distance = next_pose.distanceTo(prev_pose)
        rndnum = random.random()
        if (distance < MAX_SPEED * 0.1) and (rndnum < skip_prob):
            mask.append(0)
        else:
            mask.append(1)
            prev_pose = current_pose
        current_pose = next_pose
    mask.append(1)
    return mask

def mask_list(list, mask):
    if len(list) != len(mask):
        sys.stderr.write("Number of poses (%s) and velodyne frames (%s) differ!\n" % (len(mask), len(list)))
    output = []
    for i in range(min(len(mask), len(list))):
        if mask[i] != 0:
            output.append(list[i])
    return output

def get_delta_odometry(odometries, mask):
    if len(odometries) != len(mask):
        sys.stderr.write("Number of poses (%s) and velodyne frames (%s) differ!\n" % (len(mask), len(odometries)))
    output = [Odometry()]
    last_i = 0
    for i in range(1, min(len(mask), len(odometries))):
        if mask[i] != 0:
            output.append(odometries[i] - odometries[last_i])
            last_i = i
    return output
            
class OutputFiles:
    def __init__(self, batch_size, history_size, frames_to_join, odometries_to_join, features, division, overlay, output_prefix, max_seq_len):
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
    
    def newSequence(self, frames_count, max_in_schema):
        self.framesToWriteCount = (frames_count - max_in_schema)
        self.outFileSeqIndex += 1
        self.out_files = []
        out_files_count = self.framesToWriteCount / self.maxFramesPerFile + 1 if self.framesToWriteCount % self.maxFramesPerFile > 0 else 0
        for split_index in range(out_files_count):
            if (split_index + 1) * self.maxFramesPerFile <= self.framesToWriteCount:
                frames_in_file = self.maxFramesPerFile  
            else:
                frames_in_file = self.framesToWriteCount % self.maxFramesPerFile
            new_output_file = h5py.File(self.outputPrefix + "." + str(self.outFileSeqIndex) + "." + str(split_index) + ".hdf5", 'w')
            new_output_file.create_dataset('data', (frames_in_file * self.historySize,
                                                    self.features * self.framesToJoin * self.horizontalDivision,
                                                    64,
                                                    360 / self.horizontalDivision + self.horizontalOverlay * 2), dtype='f4')
            new_output_file.create_dataset('odometry', (frames_in_file, 6*self.odometriesToJoin), dtype='f4')
            self.out_files.append(new_output_file)
    
    def putData(self, db_name, frame_i, ch_i, data):
        if db_name == 'odometry':
            multiply = 1 
        else:
            multiply = self.historySize
        if frame_i < self.framesToWriteCount * multiply:
            file_index = frame_i / (self.maxFramesPerFile * multiply)
            self.out_files[file_index][db_name][frame_i % (self.maxFramesPerFile * multiply), ch_i] = data
            # print file_index, db_name, frame_i%(self.maxFramesPerFile*multiply), ch_i
        else:
            sys.stderr.write("Warning: frame %s out of the scope\n" % frame_i)

    def close(self):
        for f in self.out_files:
            f.close()

BATCH_SCHEMA_DATA = [[1, 0],
                     [2, 1],
                     [3, 2],
                     [4, 3]]
BATCH_SCHEMA_ODOM = [[1],
                     [2],
                     [3],
                     [4]]

BATCH_SIZE = len(BATCH_SCHEMA_ODOM)
JOINED_FRAMES = len(BATCH_SCHEMA_DATA[0])
JOINED_ODOMETRIES = len(BATCH_SCHEMA_ODOM[0])
FEATURES = 3
HISTORY_SIZE = len(BATCH_SCHEMA_DATA) / BATCH_SIZE
max_in_schema = max(reduce(lambda x, y: x + y, BATCH_SCHEMA_DATA))

MIN_SKIP_PROB = 0.0
MAX_SKIP_PROB = 0.01
STEP_SKIP_PROB = 0.9
MAX_SPEED = 60 / 3.6
FILES_PER_HDF5 = 200

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

if len(sys.argv) < 2 + max_in_schema + 1:
    sys.stderr.write("Expected arguments: <pose-file> <out-file-prefix> <frames.yaml>^{%s+}\n" % JOINED_FRAMES)
    sys.exit(1)

poses_6dof = []
for line in open(sys.argv[1]).readlines():
    kitti_pose = map(float, line.strip().split())
    o = Odometry(kitti_pose)
    poses_6dof.append(o)

random.seed()
skip_prob = MIN_SKIP_PROB
out_files = OutputFiles(BATCH_SIZE, HISTORY_SIZE, JOINED_FRAMES, JOINED_ODOMETRIES, FEATURES, HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, sys.argv[2], FILES_PER_HDF5)
data_dest_index = schema_to_dic(BATCH_SCHEMA_DATA)
odom_dest_index = schema_to_dic(BATCH_SCHEMA_ODOM)
while skip_prob < MAX_SKIP_PROB:
    mask = gen_preserve_mask(poses_6dof, skip_prob)
    # TODO - maybe also duplication = no movement

    frames = sum(mask) - JOINED_FRAMES + 1
    out_files.newSequence(frames, max_in_schema)
    files_to_use = mask_list(sys.argv[3:], mask)
    odometry_to_use = map(znorm_odom, get_delta_odometry(poses_6dof, mask))

    for i in range(len(files_to_use)):
        data_i = np.empty([FEATURES, 64, 360])
        data_i[0] = cv_yaml.load(files_to_use[i], 'range')
        data_i[1] = cv_yaml.load(files_to_use[i], 'y')
        data_i[2] = cv_yaml.load(files_to_use[i], 'intensity')
        data_i = horizontal_split(data_i, HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, FEATURES)
        odometry_i = np.asarray(odometry_to_use[i].dof)
        
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
#                        print i, frame_i, bias, slot_i, len(odometry_i), ch_i, odometry_i[ch_i]
#                        print frame_i + bias, slot_i * len(odometry_i) + ch_i
#                sys.exit(0)
            
            bias += BATCH_SIZE
        
        if i % FILES_PER_HDF5 == 0 and i > 0:
            print i, "/", len(files_to_use)
    
    skip_prob += STEP_SKIP_PROB
    out_files.close()
