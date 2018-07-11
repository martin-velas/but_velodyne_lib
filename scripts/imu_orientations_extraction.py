#! /usr/bin/env python

import sys
import numpy as np
from transformations_kittilike import quaternion_to_matrix
from odometry_cnn_data import Odometry
import json


VELO_TO_IMU = np.array([0,0,1,0,
                        1,0,0,0,
                        0,1,0,0,
                        0,0,0,1]).reshape((4,4))


def find_closest_value(dict, key):
    return dict[key] if key in dict else dict[min(dict.keys(), key=lambda k: abs(k - key))]


def get_imu_orientations(imu_json_data, frame_times):
    orientations_timed = {}
    for record in imu_json_data:
        if record["type"] == "quat_data":
            orientations_timed[float(record["gpsTimeOfWeek"])] = record["quaternion"]

    orientations = []
    for t in frame_times:
        quat = find_closest_value(orientations_timed, t)  # [x, y, z, w] in JSON
        M = quaternion_to_matrix(quat[3:] + quat[0:3])    # [w, x, y, z] expected
        M = np.matmul(np.matmul(np.linalg.inv(VELO_TO_IMU), M), VELO_TO_IMU)
        o = Odometry()
        o.M = M
        o.setDofFromM()
        orientations.append(o)
    return orientations


if __name__ == "__main__":

    if len(sys.argv) != 3:
        sys.stderr.write("ERROR, expecting arguments: [frame-times] [imu-data-log-gpstimed]\n")
        sys.exit(1)

    frame_times = map(float, open(sys.argv[1]).readlines())
    imu_json_data = json.load(open(sys.argv[2]))

    for o in get_imu_orientations(imu_json_data, frame_times):
        print o
