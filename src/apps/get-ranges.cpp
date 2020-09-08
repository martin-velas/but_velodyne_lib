/*
 * Visualization of corrections for global optimization.
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
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/common.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     SensorsCalibration &calibration,
                     float &sampling_ratio,
                     vector<string> &clouds_to_process) {
  string sensor_poses_filename;

  po::options_description desc("Get ranges of points within the cloud\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("sensor_poses,c", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
      ("sampling_ratio,s", po::value<float>(&sampling_ratio)->default_value(0.1), "Reduce size with this ratio.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_process.size() < 1) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

int main(int argc, char** argv) {

  vector<string> filenames;
  float sampling_ratio;
  SensorsCalibration calibration;

  if(!parse_arguments(argc, argv,
      calibration, sampling_ratio, filenames)) {
    return EXIT_FAILURE;
  }

  VelodynePointCloud::Ptr cloud(new VelodynePointCloud);
  VelodyneFileSequence file_sequence(filenames, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.joinTo(*cloud);
    subsample_cloud<VelodynePoint>(cloud, sampling_ratio);

    for(VelodynePointCloud::iterator p = cloud->begin(); p < cloud->end(); p++) {
      cout << computeRange(*p) << endl;
    }
  }

  return EXIT_SUCCESS;
}
