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

#include <cstdlib>
#include <cstdio>
#include <iostream>

#include <boost/program_options.hpp>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

using namespace std;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    Eigen::Affine3f &transformation, float &ratio) {

  string transformation_filename;

  po::options_description desc(
      "Interpolate transformation\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("transformation,t", po::value<string>(&transformation_filename)->required(), "KITTI poses file with transformation.")
    ("ratio,r", po::value<float>(&ratio)->default_value(0.5), "Ratio for interpolation (0.0; 1.0).")
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

  transformation = KittiUtils::load_kitti_poses(transformation_filename).front();

  return true;
}

int main(int argc, char** argv) {

  Eigen::Affine3f T;
  float ratio;
  if(!parse_arguments(argc, argv, T, ratio)) {
    return EXIT_FAILURE;
  }

  LinearInterpolationSE3 interpolation(Eigen::Affine3f::Identity(), T);
  cout << interpolation.estimate(ratio) << endl;

  return EXIT_SUCCESS;
}
