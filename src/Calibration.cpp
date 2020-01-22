//
// Created by ivelas on 06.12.19.
//

#include "but_velodyne/Calibration.h"

namespace but_velodyne {

    void load_normals(istream &in, vector<NormalsPair> &normal_pairs) {
      while (true) {
        NormalsPair pair;
        in >> pair.imu_normal.x() >> pair.imu_normal.y() >> pair.imu_normal.z();
        in >> pair.velodyne_normal.x() >> pair.velodyne_normal.y() >> pair.velodyne_normal.z();
        if (in.eof()) {
          break;
        }
        normal_pairs.push_back(pair);
        //cerr << normal_pairs.back().imu_normal << " " << normal_pairs.back().velodyne_normal << endl << "--" << endl;
      }
    }

    void show_normals(Visualizer3D &vis, const vector<NormalsPair> &normal_pairs) {
      vis.getViewer()->removeAllShapes();
      for (vector<NormalsPair>::const_iterator pair = normal_pairs.begin(); pair < normal_pairs.end(); pair++) {
        uchar r = vis.rngU();
        uchar g = vis.rngU();
        uchar b = vis.rngU();
        vis.setColor(r, g, b).addArrow(PointCloudLine(Eigen::Vector3f::Zero(), pair->imu_normal));
        vis.setColor(r, g, b).addArrow(PointCloudLine(Eigen::Vector3f::Zero(), pair->velodyne_normal));
      }
      vis.show();
    }

    Eigen::Matrix4f compute_calibration(const vector<NormalsPair> &normal_pairs) {
      MatrixOfPoints target_coresp_points(size_t(Eigen::Vector3f::RowsAtCompileTime), normal_pairs.size());
      MatrixOfPoints source_coresp_points(size_t(Eigen::Vector3f::RowsAtCompileTime), normal_pairs.size());

      for (int i = 0; i < normal_pairs.size(); i++) {
        target_coresp_points.col(i) = normal_pairs[i].velodyne_normal;
        source_coresp_points.col(i) = normal_pairs[i].imu_normal;
      }

      // Compute the Covariance matrix of these two pointclouds moved to the origin
      // This is not properly covariance matrix as there is missing the 1/N
      // 1/N is important for computing eigenvalues(scale), not the eigenvectors(directions) - as we are interested in eigenvectors
      Eigen::Matrix3f A = target_coresp_points * source_coresp_points.transpose();

      // Compute the SVD upon A = USV^t
      Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

      // Compute the determinant of V*U^t - to find out in what direction the rotation is
      float det = (svd.matrixV() * svd.matrixU().transpose()).determinant();

      // Fix the right hand/left hand rotation : assuming we would like the right hand rotation
      Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
      E(2, 2) = (det >= 0) ? 1.0f : -1.0f;

      // Compute the rotation as R = VEU^t
      // R is the rotation of point_0_translated to fit the source_coresp_points_translated
      Eigen::Matrix3f R = svd.matrixV() * E * (svd.matrixU().transpose());

      Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
      transformation.block(0, 0, 3, 3) = R;

      return transformation;
    }
}
