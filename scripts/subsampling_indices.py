#! /usr/bin/env python

import sys

if len(sys.argv) != 3:
    sys.stderr.write("ERROR, expecting arguments [original.indices] [subsampled.indices]\n")
    sys.exit(1)

original_indices = map(int, open(sys.argv[1]).readlines())
subsampled_indices = map(int, open(sys.argv[2]).readlines())

max_i = max(original_indices)

inverted_original = [-1]*(max_i+1)
for original,idx in enumerate(original_indices):
    inverted_original[idx] = original

for si in subsampled_indices:
    print inverted_original[si]
