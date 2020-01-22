/*
 * Intensity normalization for Velodyne data.
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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/AdaptiveIntensitiesNormalization.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>

#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointWithSource PointType;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     string &sum_cloud_filename,
                     float &expected_mean, float &expected_std_dev, string &out_filename) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
          ("help,h", "produce help message")
          ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
          ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
          ("in_filename,i", po::value<string>(&sum_cloud_filename)->required(), "Input cloud PCD file.")
          ("expected_mean,m", po::value<float>(&expected_mean)->default_value(300.0), "Target mean of intensities.")
          ("expected_std_dev,d", po::value<float>(&expected_std_dev)->default_value(1.0), "Target standard deviation of intensities.")
          ("out_filename,o", po::value<string>(&out_filename)->required(), "Output normalized cloud filename.")
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
  } catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  string sum_cloud_filename, out_filename;
  SensorsCalibration calibration;
  float expected_mean, expected_std_dev;
  if(!parse_arguments(argc, argv,
      poses, calibration, sum_cloud_filename,
      expected_mean, expected_std_dev, out_filename)) {
    return EXIT_FAILURE;
  }

  cerr << "Loading points ..." << endl;
  PointCloud<PointType>::Ptr sum_cloud(new PointCloud<PointType>);
  io::loadPCDFile(sum_cloud_filename, *sum_cloud);

  AdaptiveIntensitiesNormalization normalization(expected_mean, expected_std_dev);
  PointCloud<PointType>::Ptr normalized_cloud(new PointCloud<PointType>);
  normalization.run(sum_cloud, calibration, poses, normalized_cloud);

  cerr << "Saving output ..." << endl;
  io::savePCDFileBinary(out_filename, *normalized_cloud);

  return EXIT_SUCCESS;
}
