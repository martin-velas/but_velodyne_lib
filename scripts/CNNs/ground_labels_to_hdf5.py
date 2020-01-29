#! /usr/bin/env python

import sys
import h5py
import numpy
import cv

def load_from_yaml(yaml_filename, node_name):
    return numpy.asarray(cv.Load(yaml_filename, cv.CreateMemStorage(), node_name))

if len(sys.argv) != 3:
    sys.stderr.write("Invalid arguments. Usage: ground_labels_to_hdf5.py <input-YAML-fileliest> <output-HDF5-file>\n")
    sys.exit(1)

output_file = h5py.File(sys.argv[2], 'w')
data_files = open(sys.argv[1]).readlines()
output_file.create_dataset('data', (len(data_files), 3, 64, 360), dtype='f8')
output_file.create_dataset('labels', (len(data_files), 64, 360), dtype='f4')

i = 0
for data_file in data_files:
    data_file = data_file.strip()
    data = numpy.empty([3, 64, 360])
    labels = numpy.empty([64, 360])
    
    data[0] = load_from_yaml(data_file, 'range')
    data[1] = load_from_yaml(data_file, 'y')
    data[2] = load_from_yaml(data_file, 'intensity')
    labels = load_from_yaml(data_file, 'ground_labels')
    
    output_file['data'][i] = data
    output_file['labels'][i] = labels
    i += 1

    if i%1000 == 0:
        print i, "/", len(data_files)

output_file.close()
