//
// Created by ivelas on 06.12.19.
//

#ifndef BUT_VELODYNE_LIB_CALIBRATION_H
#define BUT_VELODYNE_LIB_CALIBRATION_H

#include <cstdlib>
#include <cstdio>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

namespace but_velodyne {
    typedef Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Vector3f::RowsAtCompileTime, Eigen::Dynamic> MatrixOfPoints;

    typedef struct {
        Eigen::Vector3f imu_normal;
        Eigen::Vector3f velodyne_normal;
    } NormalsPair;

    void load_normals(istream &in, vector<NormalsPair> &normal_pairs);

    void show_normals(but_velodyne::Visualizer3D &vis, const vector<NormalsPair> &normal_pairs);

    Eigen::Matrix4f compute_calibration(const vector<NormalsPair> &normal_pairs);
}

#endif //BUT_VELODYNE_LIB_CALIBRATION_H
