/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/06/2015
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

#include <iostream>

#include <pcl/common/eigen.h>
#include <cxeigen.hpp>

#include <but_velodyne/PoseGraphEdge.h>

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

namespace but_velodyne
{

float PoseGraphEdge::CORRECTION = 0.01;

// EDGE3D sourceIdx targetIdx 6DOF(6 floats) upper-triange-of-cholesky-cov(21 floats)
std::ostream& operator<<(std::ostream &stream, const PoseGraphEdge &edge) {
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f affineTransf(edge.transformation);
  getTranslationAndEulerAngles(affineTransf, x, y, z, roll, pitch, yaw);

  stream << "EDGE3" << " " <<
      edge.sourceIdx << " " << edge.targetIdx << " " <<
      x << " " << y << " " << z << " " <<
      roll << " " << pitch << " " << yaw;

  Eigen::MatrixXf eigenCovariance;
  cv2eigen(edge.covariance, eigenCovariance);

  MatrixXf correction = MatrixXf::Identity(edge.covariance.rows, edge.covariance.cols) * PoseGraphEdge::CORRECTION;
  MatrixXf precision = (eigenCovariance + correction).inverse();

  for(int row = 0; row < edge.covariance.rows; row++) {
    for(int col = 0; col < edge.covariance.cols; col++) {
      if(row <= col) {
        stream << " " << precision(row, col);
      }
    }
  }
  return stream;
}

std::istream& operator>>(std::istream& is, PoseGraphEdge& edge) {
  string type;
  float x, y, z, roll, pitch, yaw;

  is >> type >> edge.sourceIdx >> edge.targetIdx >> x >> y >> z >> roll >> pitch >> yaw;

  Eigen::Affine3f t = getTransformation(x, y, z, roll, pitch, yaw);
  edge.transformation = t.matrix();

  edge.covariance.create(6, 6, CV_32FC1);
  for(int r = 0; r < edge.covariance.rows; r++) {
    for(int c = 0; c < edge.covariance.cols; c++) {
      if(r <= c) {
        float cov; is >> cov;
        edge.covariance.at<float>(r, c) = cov;
        edge.covariance.at<float>(c, r) = cov;
      }
    }
  }

  return is;
}

std::ostream& operator<<(std::ostream &stream, const PoseToLandmarkGraphEdge &edge) {
  stream << "EDGE_SE3_XYZ" << " " <<
      edge.sourceIdx << " " << edge.targetIdx << " " <<
      edge.dx << " " << edge.dy << " " << edge.dz;

  Eigen::MatrixXf eigenCovariance;
  cv2eigen(edge.covariance, eigenCovariance);

  Eigen::MatrixXf precision = eigenCovariance.inverse();

  for(int row = 0; row < edge.covariance.rows; row++) {
    for(int col = 0; col < edge.covariance.cols; col++) {
      if(row <= col) {
        stream << " " << precision(row, col);
      }
    }
  }
  return stream;
}

} /* namespace but_velodyne */
