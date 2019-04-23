/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 08/03/2019
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

#include <but_velodyne/InterpolationSE3.h>
#include <but_velodyne/KittiUtils.h>
#include <Eigen/Eigenvalues>

using namespace std;

namespace but_velodyne {

BSplineSE3::BSplineSE3(const std::vector<Eigen::Affine3f> &control_poses_) :
  control_poses(control_poses_) {
  C << 6,  0,  0,  0,
       5,  3, -3,  1,
       1,  3,  3, -2,
       0,  0,  0,  1;
  C *= 1.0/6.0;
  assert(control_poses.size() == DEGREE);
  Omega.push_back(Eigen::Matrix4f::Zero());
  for(int i = 1; i < DEGREE; i++) {
    Omega.push_back((control_poses[i-1].inverse() * control_poses[i]).matrix().log());
  }
}

Eigen::Affine3f BSplineSE3::estimate(const float t) const {
  Eigen::Vector4f B = C * Eigen::Vector4f(1, t, t*t, t*t*t);
  Eigen::Matrix4f result = control_poses[0].matrix();
  for(int i = 1; i < DEGREE; i++) {
    result = result * (B(i) * Omega[i]).exp();
  }
  Eigen::Affine3f result_t(result);
  return result_t;
}

Eigen::Affine3f LinearInterpolationSE3::estimate(const float portion) const {
  assert(0.0 <= portion && portion  <= 1.0);

  Eigen::Quaternionf rotation(delta.rotation());
  rotation = Eigen::Quaternionf::Identity().slerp(portion, rotation);
  Eigen::Translation3f translation(delta.translation() * portion);

  return init * (translation * rotation);
}

Eigen::Affine3f InterpolationBuffered::estimate(float t) {
  t = (t + phase_offset) * phase_scale;

  map_type::const_iterator it = interpolated_poses.find(t);
  if(it != interpolated_poses.end()) {
    return it->second;
  } else {
    interpolated_poses[t] = method->estimate(t);
    return interpolated_poses[t];
  }
}

Eigen::Affine3f average_transformations(const std::vector<Eigen::Affine3f> &transformations) {
  Eigen::Vector3f translation_sum(0, 0, 0);
  Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
  for(std::vector<Eigen::Affine3f>::const_iterator t = transformations.begin(); t < transformations.end(); t++) {

    translation_sum = translation_sum + t->translation();
    Eigen::Quaternionf Q(t->rotation());
    Eigen::Vector4f Q_vec(Q.w(), Q.x(), Q.y(), Q.z());
    A = A + Q_vec * Q_vec.transpose();
  }

  translation_sum *= 1.0 / transformations.size();

  A = A * (1.0/transformations.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver;
  solver.compute(A);
  Eigen::Vector4f evalues = solver.eigenvalues();
  int largest_vector;
  float largest_value = -INFINITY;
  for(int i = 0; i < 4; i++) {
    if(evalues(i) > largest_value) {
      largest_vector = i;
    }
  }

  Eigen::Vector4f Q_vec = solver.eigenvectors().col(largest_vector);
  Eigen::Quaternionf Q(Q_vec(0), Q_vec(1), Q_vec(2), Q_vec(3));

  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block(0, 0, 3, 3) = Q.matrix();
  matrix.block(0, 3, 3, 1) = translation_sum;

  return Eigen::Affine3f(matrix);
}

} /* namespace but_velodyne */
