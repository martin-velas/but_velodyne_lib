#! /usr/bin/env python

import numpy as np
import sys
import math
import random
import h5py

if len(sys.argv) != 2:
    sys.stderr.write("ERROR, arguments: [output-file.hdf5]\n")
    sys.exit(1)

lines = sys.stdin.readlines()
data = []
for l in lines:
    tokens = l.strip().split()
    odometry = map(float, tokens[0:6])
    files = tokens[6:]
    data.append({"odometry":odometry, "data":files})

dato = np.load(data[0]["data"][0])
rings, width, channels = dato.shape
frames_per_odometry = len(data[0]["data"])

with h5py.File(sys.argv[1], 'w') as output_file:
    output_file.create_dataset('data', (len(data), channels*frames_per_odometry, rings, width), dtype='f4')
    output_file.create_dataset('odometry',    (len(data), 6), dtype='f4')

    for di, dato in enumerate(data):
        for fi, file in enumerate(dato["data"]):
            frame = np.load(file)
            frame = np.moveaxis(frame, [0, 1, 2], [1, 2, 0])
            output_file["data"][di, fi*channels:(fi+1)*channels, ...] = frame
        output_file["odometry"][di, :] = np.array(dato["odometry"])
