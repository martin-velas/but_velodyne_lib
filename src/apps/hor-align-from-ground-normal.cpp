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

#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

Eigen::Affine3f estimate_alignment(const Eigen::Vector3f& normal) {
  Eigen::Vector3f expected_y = normal;
  if(expected_y(1) > 0) {
    // preserve Y up/down orientation
    expected_y *= -1.0;
  }
  Eigen::Vector3f expected_z(0, expected_y(2), -expected_y(1));
  expected_z.normalize();
  Eigen::Affine3f plane_horizontal_corr;
  getTransformationFromTwoUnitVectorsAndOrigin(expected_y, expected_z, Eigen::Vector3f::Identity(), plane_horizontal_corr);

  return plane_horizontal_corr;
}

bool parse_arguments(int argc, char **argv, Eigen::Vector3f& normal) {

  po::options_description desc("Horizontal alignment of point cloud\n"
      "======================================\n"
      " * Allowed options");
  vector<float> normal_vector;
  desc.add_options()
    ("help,h", "produce help message")
    ("normal,n", po::value< vector<float> >(&normal_vector)->multitoken()->required(), "Ground normal.")
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
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }

  normal(0) = normal_vector[0];
  normal(1) = normal_vector[1];
  normal(2) = normal_vector[2];
  return true;
}

int main(int argc, char** argv) {

  Eigen::Vector3f normal;
  if(!parse_arguments(argc, argv, normal)) {
    return EXIT_FAILURE;
  }

  Eigen::Affine3f alignment = estimate_alignment(normal);
  KittiUtils::printPose(cout, alignment.matrix());

  return EXIT_SUCCESS;
}
