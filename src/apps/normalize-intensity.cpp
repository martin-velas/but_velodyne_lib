/*
 * Normalization of point cloud intensities.
 *
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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
  string &input_cloud, string &output_cloud,
  float &skip_ratio,float &min_val,float &max_val) {

  po::options_description desc("Horizontal alignment and defile of point cloud\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_cloud,i", po::value<string>(&input_cloud)->required(), "Input point cloud")
    ("output_cloud,o", po::value<string>(&output_cloud)->required(), "Output point cloud")
    ("skip_ratio,s", po::value<float>(&skip_ratio)->default_value(0.01), "Portion of min/max intensities to skip.")
    ("min", po::value<float>(&min_val)->default_value(0.0), "Min target intensity.")
    ("max", po::value<float>(&max_val)->default_value(1.0), "Max target intensity.")
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
  return true;
}

int main(int argc, char** argv) {

  string input_file, output_file;
  float skip_ratio, min_val, max_val;
  if(!parse_arguments(argc, argv, input_file, output_file, skip_ratio, min_val, max_val)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZI> cloud;
  pcl::io::loadPCDFile(input_file, cloud);
  Visualizer3D::normalizeMinMaxIntensity(cloud, cloud, skip_ratio, min_val, max_val);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud = Visualizer3D::colorizeCloud(cloud, true);

  io::savePCDFileBinary(output_file, cloud);
  //io::savePCDFileBinary(output_file + ".rgb.pcd", *rgb_cloud);

  return EXIT_SUCCESS;
}
