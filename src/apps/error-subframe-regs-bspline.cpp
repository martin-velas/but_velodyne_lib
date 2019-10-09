/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 02/07/2015
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
#include <iostream>
#include <vector>

#include <but_velodyne/Visualizer3D.h>

using namespace std;



#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
};

Eigen::Vector6d pose2vec(const Eigen::Affine3f &pose) {
  Eigen::Vector6d vec;
  vec(0) = pose.translation()(0);
  vec(1) = pose.translation()(1);
  vec(2) = pose.translation()(2);

  Eigen::AngleAxisf R(pose.rotation());
  vec(3) = R.axis()(0) * R.angle();
  vec(4) = R.axis()(1) * R.angle();
  vec(5) = R.axis()(2) * R.angle();

  return vec;
}

Eigen::Affine3f vec2pose(const Eigen::Vector6d &vec) {

  Eigen::Translation3f t(vec(0), vec(1), vec(2));

  Eigen::Vector3d axisd = vec.tail(3);
  Eigen::Vector3f axis = axisd.cast<float>();
  float angle = axis.norm();
  if(angle < 1e-6) {
    axis.x() = 1;
    axis.y() = 0;
    axis.z() = 0;
  } else {
    axis.normalize();
  }
  Eigen::AngleAxisf R(angle, axis);

  return t*R;
}

class BSplineSE3 {
public:
  BSplineSE3(const std::vector<Eigen::Affine3f> &control_poses_) :
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

  Eigen::Affine3f estimate(const float t) const {
    Eigen::Vector4f B = C * Eigen::Vector4f(1, t, t*t, t*t*t);
    Eigen::Matrix4f result = control_poses[0].matrix();
    for(int i = 1; i < DEGREE; i++) {
      result = result * (B(i) * Omega[i]).exp();
    }
    Eigen::Affine3f result_t(result);
    return result_t;
  }

private:
  static const int DEGREE = 4;

  std::vector<Eigen::Affine3f> control_poses;
  Eigen::Matrix4f C;
  std::vector<Eigen::Matrix4f> Omega;
};

double bspline_error(const Eigen::Vector6d &V0, const Eigen::Vector6d &V1,
    const Eigen::Vector6d &V2, const Eigen::Vector6d &V3,
    const double t, const Eigen::Vector6d &ref) {

  vector<Eigen::Affine3f> control_points;
  control_points.push_back(vec2pose(V0));
  control_points.push_back(vec2pose(V1));
  control_points.push_back(vec2pose(V2));
  control_points.push_back(vec2pose(V3));

  Eigen::Affine3f ref_pose = vec2pose(ref);

  BSplineSE3 spline(control_points);

  Eigen::Affine3f estimation = spline.estimate(t);
  Eigen::Affine3f diff = ref_pose.inverse() * estimation;
  double diff_t = diff.translation().norm();
  double diff_R = fabs(Eigen::AngleAxisf(diff.rotation()).angle());

  but_velodyne::Visualizer3D vis;
  vis.getViewer()->removeAllCoordinateSystems();
  vis.getViewer()->removeAllShapes();
  vis.addPoses(control_points).show();

  control_points.push_back(ref_pose);
  cerr << "Ref:" << ref_pose.matrix() << endl;
  vis.getViewer()->removeAllCoordinateSystems();
  vis.addPoses(control_points).show();

  control_points.push_back(estimation);
  cerr << "Estimation:" << estimation.matrix() << endl;
  cerr << "Diff t: " << diff_t << " m; R: " << diff_R << endl;
  vis.getViewer()->removeAllCoordinateSystems();
  vis.addPoses(control_points).show();

  const double wt = 1.0;
  const double wR = 10.0;
  return diff_t * wt + diff_R * wR;
}



#include <but_velodyne/KittiUtils.h>

using namespace but_velodyne;

int main(int argc, char *argv[])
{
  if(argc != 2) {
    cerr << "ERROR, usage: " << argv[0] << " POSE_FILENAME" << endl;
    return EXIT_FAILURE;
  }

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[1]);

  cerr << "Poses: " << poses.size() << endl;

  Eigen::Vector6d V0 = pose2vec(poses[0]);
  Eigen::Vector6d V1 = pose2vec(poses[1]);
  Eigen::Vector6d V2 = pose2vec(poses[2]);
  Eigen::Vector6d V3 = pose2vec(poses[3]);
  Eigen::Vector6d ref = pose2vec(poses[4]);

  double error = bspline_error(V0, V1, V2, V3, 1.0/3.0, ref);

  cerr << vec2pose(V0) << endl;
  cerr << vec2pose(V1) << endl;
  cerr << vec2pose(V2) << endl;
  cerr << vec2pose(V3) << endl;

  cerr << "Error: " << error << endl;

  return EXIT_SUCCESS;
}
