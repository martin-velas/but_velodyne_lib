#! /usr/bin/env python

import sys
import numpy as np
import h5py
import os

FILE_SIZE=96

filenames = []
fi = 1
while fi < len(sys.argv):
    if sys.argv[fi] == "-o":
        out_filename = sys.argv[fi+1]
        fi += 2
    else:
        filenames.append(sys.argv[fi])
        fi += 1

if len(out_filename) == 0 or len(filenames) == 0:
    sys.stderr.write("Usage: %s -o <part-output.hdf5> <input.hdf5>+\n", sys.argv[0])
    sys.exit(1)

for filename in filenames:
    with h5py.File(filename, "r") as hf:
        data = hf["data"]
        odometry = hf["odometry"]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count
        parts = odom_count/FILE_SIZE + (1 if odom_count%FILE_SIZE > 0 else 0)

        print filename, "will be split into", parts, "parts"

        odoms_copied = 0
        for s in range(parts):
            new_odoms_count = min(FILE_SIZE, odom_count-odoms_copied)
            new_filename = out_filename + ".%s"%(s)
            print new_filename
            output_file = h5py.File(new_filename, 'w')
            output_file.create_dataset('data', (new_odoms_count*hsize,) + np.shape(data)[1:], dtype='f4')
            output_file.create_dataset('odometry', (new_odoms_count,) + np.shape(odometry)[1:], dtype='f4')
            output_file['odometry'][...] = odometry[odoms_copied:odoms_copied+new_odoms_count]
            output_file['data'][...] = data[odoms_copied*hsize:(odoms_copied+new_odoms_count)*hsize]

            odoms_copied += new_odoms_count
