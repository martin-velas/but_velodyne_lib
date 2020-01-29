#! /usr/bin/env python

import sys
import numpy
import cv
import cv2
from numpy import histogram

def load_from_yaml(yaml_filename, node_name):
    return numpy.asarray(cv.Load(yaml_filename, cv.CreateMemStorage(), node_name))


if len(sys.argv) != 3:
    sys.stderr.write("Expected arguments: <train-data-list> <test-data-list>\n")
    sys.exit(1)

train_files_list = open(sys.argv[1], "r").readlines()
histogram = numpy.zeros((64, 360), numpy.float32)
for train_file in train_files_list:
    train_file = train_file.strip()
    print train_file
    labels = load_from_yaml(train_file, "ground_labels")
    histogram += labels

histogram *= 1.0/len(train_files_list)

cv.Save("histogram.yaml", cv2.cv.fromarray(histogram))