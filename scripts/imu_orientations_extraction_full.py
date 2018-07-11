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


EPS = 1e-4


def extract_imu_all_orientations(imu_json_data):
    for record in imu_json_data:
        if record["type"] == "quat_data":
            time = float(record["gpsTimeOfWeek"])
            quat = record["quaternion"]
            M = quaternion_to_matrix(quat[3:] + quat[0:3])  # [w, x, y, z] expected
            o = Odometry()
            o.M = np.matmul(np.matmul(np.linalg.inv(VELO_TO_IMU), M), VELO_TO_IMU)
            o.setDofFromM()
            print time, o


if __name__ == "__main__":

    if len(sys.argv) != 3:
        sys.stderr.write("ERROR, expecting arguments: [frame-times] [imu-data-log-gpstimed]\n")
        sys.exit(1)

    imu_json_data = json.load(open(sys.argv[2]))

    extract_imu_all_orientations(imu_json_data)
