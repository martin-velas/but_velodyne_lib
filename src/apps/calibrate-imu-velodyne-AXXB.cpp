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
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &A,
                     vector<Eigen::Affine3f> &B,
                     float &max_angle, int &samples) {
  string A_filename, B_filename;

  po::options_description desc("IMU Velodyne calibration with evolution strategy\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
  ;

  desc.add_options()
    ("A,a", po::value<string>(&A_filename)->required(), "A affine transformations.")
    ("B,b", po::value<string>(&B_filename)->required(), "B affine transformations.")
    ("max_angle,r", po::value<float>(&max_angle)->default_value(0), "Maximal rotation [deg].")
    ("samples,s", po::value<int>(&samples)->default_value(1000000), "Samples.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  try {
      po::notify(vm);
  }
  catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }

  A = KittiUtils::load_kitti_poses(A_filename);
  B = KittiUtils::load_kitti_poses(B_filename);

  if(A.size() != B.size()) {
    cerr << "Error: Sizes of A (" << A.size() << " and B (" << B.size() << " do not match!" << endl;
    return false;
  }

  return true;
}

void generate_sphere(const float r_rad, const int samples, PointCloud<PointXYZ> &sphere) {
  cv::RNG& rng = cv::theRNG();
  rng.state = cv::getTickCount();

  while(sphere.size() < samples) {
    PointXYZ pt;
    pt.x = rng.uniform(-r_rad, r_rad);
    pt.y = rng.uniform(-r_rad, r_rad);
    pt.z = rng.uniform(-r_rad, r_rad);

    if(pt.getVector3fMap().norm() <= r_rad)
      sphere.push_back(pt);
  }
}

// AX = XB
float eval_calibration(const Eigen::Affine3f &X, const vector<Eigen::Affine3f> &A_inv, const vector<Eigen::Affine3f> &B) {
  float error = 0.0;
  Eigen::Affine3f X_inv = X.inverse();
  for(int i = 0; i < A_inv.size(); i++) {
    const Eigen::Affine3f err_M = X_inv * A_inv[i] * X * B[i];
    const float err_angle = Eigen::AngleAxisf(err_M.rotation()).angle();
    const float err_translation = err_M.translation().norm();

    error += tan(err_angle)*30.0 + err_translation;
  }
  return error;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> A, B;
  float max_R;
  int samples;
  if(!parse_arguments(argc, argv, A, B, max_R, samples)) {
    return EXIT_FAILURE;
  }

  vector<Eigen::Affine3f> A_inv;
  for(vector<Eigen::Affine3f>::const_iterator a = A.begin(); a < A.end(); a++) {
    A_inv.push_back(a->inverse());
  }

  cerr << "A_inv transformations: " << A_inv.size() << endl;
  cerr << "B transformations: " << B.size() << endl;

  PointCloud<PointXYZ> sphere;
  generate_sphere(DEG2RAD(max_R), samples, sphere);
  // Visualizer3D().addPointCloud(sphere).show();

  float min_error = INFINITY;
  Eigen::Affine3f best_R;
  int iterations = 0;
  for(PointCloud<PointXYZ>::iterator pt = sphere.begin(); pt < sphere.end(); pt++) {
    Eigen::Vector3f axis = pt->getVector3fMap();
    const float angle = axis.norm();
    axis.normalize();
    Eigen::Affine3f R(Eigen::AngleAxisf(angle, axis));

    const float error = eval_calibration(R, A_inv, B);
    if(error < min_error) {
      cerr << error << endl << R << endl;
      min_error = error;
      best_R = R;
    }

    if(++iterations%10000 == 1) {
      cerr << "DONE " << iterations << "/" << samples << endl;
    }
  }

  cout << best_R << endl;

  return EXIT_SUCCESS;
}
