#! /usr/bin/env python

import sys
import numpy as np
import math

import cv_yaml

for file in sys.argv[1:]:
    mat = cv_yaml.load(file, "range-y-intensity")
    print file, mat.shape
    np.save(file + ".npy", mat)
