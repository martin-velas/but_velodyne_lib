#! /usr/bin/env python

import argparse, sys
import numpy as np


EPS = 0.0001


def isclose(a, b):
    return abs(a-b) <= EPS


def load_signal(filename):
    signal = {}
    with open(filename) as file:
        for line in file.readlines():
            tokens = map(float, line.split())
            signal[tokens[0]] = tokens[1:]
    return signal


def get_fps(signal):
    min_t = min(signal.keys())
    max_t = max(signal.keys())
    return (len(signal) - 1) / (max_t - min_t)


def get_dimension(signal):
    return len(signal[min(signal.keys())])


def signal_to_matrix(signal):
    M = np.zeros((len(signal), get_dimension(signal)))
    sorted_times = sorted(signal.keys())
    for i, k in enumerate(sorted_times):
        M[i, :] = signal[k]
    return sorted_times, M


def upsample_signal(dense_times, sparse_signal):
    dense_times_sorted = sorted(dense_times)
    sparse_times_sorted = sorted(sparse_signal.keys())
    sparse_fps = get_fps(sparse_signal)

    upsampled = {}
    for i, t in enumerate(dense_times_sorted):
        closest_sparse_times = filter(lambda x: abs(t-x) < (1/sparse_fps)-EPS, sparse_times_sorted)
        closest_sparse_times_cnt = len(closest_sparse_times)
        if closest_sparse_times_cnt > 0:
            if closest_sparse_times_cnt == 1:
                upsampled[t] = sparse_signal[closest_sparse_times[0]]
            elif closest_sparse_times_cnt == 2:
                t1, t2 = closest_sparse_times
                w1, w2 = (1/sparse_fps) - abs(t-t1), (1/sparse_fps) - abs(t-t2)
                sample1, sample2 = sparse_signal[t1], sparse_signal[t2]
                upsampled[t] = [(w1*s1 + w2*s2)/(w1 + w2) for s1, s2 in zip(sample1, sample2)]
            else:
                sys.stderr.write("Error, %s closest times found for %s (%s)\n" %
                                 (len(closest_sparse_times), t, closest_sparse_times))
                sys.exit(1)

    return upsampled


parser = argparse.ArgumentParser(description="Cross correlation of two signals")
parser.add_argument("-i,--signal_imu", dest="signal_imu", type=str, required=True)
parser.add_argument("-v,--signal_cls", dest="signal_cls", type=str, required=True)
parser.add_argument("-w", "--window_width", dest="window_width", type=float, required=False, default=2.0)
parser.add_argument("-t", "--max_time_shift", dest="max_time_shift", type=float, required=False, default=1.0)
args = parser.parse_args()

signal_imu = load_signal(args.signal_imu)
imu_fps = get_fps(signal_imu)
signal_cls = load_signal(args.signal_cls)

signal_cls_upsampled = upsample_signal(signal_imu.keys(), signal_cls)

times_imu, matrix_imu = signal_to_matrix(signal_imu)
times_cls, matrix_cls = signal_to_matrix(signal_cls_upsampled)
bias = int((times_cls[0] - times_imu[0]) * imu_fps)

window = int(imu_fps * args.window_width)
max_shift = int(imu_fps * args.max_time_shift)

frames = len(times_cls)
start_index = max([window/2, window/2 + max_shift - bias])
end_index = min([frames - window/2, frames - (window/2 + max_shift) - bias])

for i in range(start_index, end_index):
    time = times_cls[i]

    max_correlation = -float("inf")
    for shift in range(-max_shift, max_shift+1):

        matrix_imu_slice = matrix_imu[i+bias+shift-window/2:i+bias+shift+window/2+1, :]
        matrix_cls_slice = matrix_cls[i-window/2:i+window/2+1, :]

        correlation = np.sum(np.multiply(matrix_imu_slice, matrix_cls_slice))
        if correlation > max_correlation:
            best_shift = shift / imu_fps
            max_correlation = correlation

    print time, best_shift
