#! /usr/bin/env python

import argparse, sys


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


def upsample_signal(dense_times, sparse_signal):
    sparse_signal_keys_sorted = sorted(sparse_signal.keys())
    min_sparse_t = sparse_signal_keys_sorted[0]
    sparse_fps = get_fps(signal_cls)
    dimension = len(sparse_signal[min_sparse_t])
    upsampled = {}
    for t in dense_times:
        closest_sparse_times = filter(lambda x: abs(t-x) < (1/sparse_fps)-EPS, sparse_signal_keys_sorted)
        closest_sparse_times_cnt = len(closest_sparse_times)
        if closest_sparse_times_cnt == 0:
            upsampled[t] = [0] * dimension
            #print t, upsampled[t]
        elif closest_sparse_times_cnt == 1:
            upsampled[t] = sparse_signal[closest_sparse_times[0]]
            #print t, upsampled[t]
        elif closest_sparse_times_cnt == 2:
            t1, t2 = closest_sparse_times
            w1, w2 = (1/sparse_fps) - abs(t-t1), (1/sparse_fps) - abs(t-t2)
            sample1, sample2 = sparse_signal[t1], sparse_signal[t2]
            upsampled[t] = [(w1*s1 + w2*s2)/(w1 + w2) for s1, s2 in zip(sample1, sample2)]
            #print t, t1, t2, sample1, sample2, upsampled[t]
        else:
            sys.stderr.write("Error, %s closest times found for %s (%s)\n" % (len(closest_sparse_times), t, closest_sparse_times))
            sys.exit(1)
    return upsampled





parser = argparse.ArgumentParser(description="Cross correlation of two signals")
parser.add_argument("-i,--signal_imu", dest="signal_imu", type=str, required=True)
parser.add_argument("-v,--signal_cls", dest="signal_cls", type=str, required=True)
parser.add_argument("-w", "--window_width", dest="window_width", type=float, required=False, default=1.0)
parser.add_argument("-t", "--max_time_shift", dest="max_time_shift", type=float, required=False, default=1.0)
args = parser.parse_args()

signal_imu = load_signal(args.signal_imu)
signal_cls = load_signal(args.signal_cls)

signal_sparser_upsampled = upsample_signal(sorted(signal_imu.keys()), signal_cls)
assert(len(signal_sparser_upsampled) == len(signal_imu))

matrix_imu, matrix_cls = generate_matrices(signal_imu, signal_cls)
