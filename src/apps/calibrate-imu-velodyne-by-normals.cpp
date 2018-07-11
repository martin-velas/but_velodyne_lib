/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <cstdio>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
typedef Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Vector3f::RowsAtCompileTime, Eigen::Dynamic> MatrixOfPoints;

typedef struct {
  Eigen::Vector3f imu_normal;
  Eigen::Vector3f velodyne_normal;
} NormalsPair;

void load_normals(istream &in, vector<NormalsPair> &normal_pairs) {
  while(true) {
    NormalsPair pair;
    in >> pair.imu_normal.x() >> pair.imu_normal.y() >> pair.imu_normal.z();
    in >> pair.velodyne_normal.x() >> pair.velodyne_normal.y() >> pair.velodyne_normal.z();
    if(in.eof()) {
      break;
    }
    normal_pairs.push_back(pair);
    //cerr << normal_pairs.back().imu_normal << " " << normal_pairs.back().velodyne_normal << endl << "--" << endl;
  }
}

void show_normals(Visualizer3D &vis, const vector<NormalsPair> &normal_pairs) {
  vis.getViewer()->removeAllShapes();
  for(vector<NormalsPair>::const_iterator pair = normal_pairs.begin(); pair < normal_pairs.end(); pair++) {
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

  for(int i = 0; i < normal_pairs.size(); i++) {
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

int main(int argc, char** argv) {

  vector<NormalsPair> imu_velodyne_normals;
  load_normals(cin, imu_velodyne_normals);

  Visualizer3D vis;
  show_normals(vis, imu_velodyne_normals);

  Eigen::Matrix4f calibration = compute_calibration(imu_velodyne_normals);

  cerr << calibration << endl;
  KittiUtils::printPose(cout, calibration);

  for(vector<NormalsPair>::iterator pair = imu_velodyne_normals.begin(); pair < imu_velodyne_normals.end(); pair++) {
    pair->velodyne_normal = calibration.block(0, 0, 3, 3) * pair->velodyne_normal;
  }

  show_normals(vis, imu_velodyne_normals);

  return EXIT_SUCCESS;
}
