/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2019
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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(
    int argc, char **argv,
    vector<Eigen::Affine3f> &registrations) {

  string registrations_filename, sensors_pose_file;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&registrations_filename)->required(), "KITTI poses file.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }
  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  registrations = KittiUtils::load_kitti_poses(registrations_filename);

  if(registrations.size() % 3) {
    std::cerr << "Number of poses " << registrations.size() << " is not divisible by 3" <<
        " (only cubic B-splines are supported)." << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> registrations;
  if(!parse_arguments(argc, argv, registrations)) {
    return EXIT_FAILURE;
  }

  vector<Eigen::Affine3f> P(4, Eigen::Affine3f::Identity());

  for(vector<Eigen::Affine3f>::const_iterator t = registrations.begin(); t < registrations.end(); t++) {
    P[0] = P[1];
    P[1] = P[2];
    P[2] = P[3];
    P[3] = P[0]*(*t);

    const BSplineSE3 spline(P);
    P[1] = spline.estimate(0.0);
    P[2] = spline.estimate(1.0);

    cout << P[0] << endl;
  }

  cout << P[1] << endl;
  cout << P[2] << endl;
  cout << P[3] << endl;

  cout << P[3] * (P[2].inverse()*P[3]) << endl;

  return EXIT_SUCCESS;
}
