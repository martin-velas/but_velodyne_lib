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
#include <boost/program_options.hpp>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(
    int argc, char **argv,
    vector<Eigen::Affine3f> &poses,
    float &t) {

  string pose_filename;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "2 KITTI poses file.")
      ("t_param,t", po::value<float>(&t)->required(), "t parameter [0,1).")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);
  if(poses.size() != 2) {
    cerr << "Error - 2 poses required, " << poses.size() << " provided" << endl;
    return false;
  }

  return true;
}


int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  float t;
  if(!parse_arguments(argc, argv, poses, t)) {
    return EXIT_FAILURE;
  }

  LinearInterpolationSE3 interpolation(poses[0], poses[1]);
  cout << interpolation.estimate(t) << endl;

  return EXIT_SUCCESS;
}
