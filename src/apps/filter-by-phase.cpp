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

#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/PhaseFilter.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    SensorsCalibration &calibration,
    vector<string> &clouds_to_process,
    float &start_phase, float &end_phase,
    string &out_dir) {

  string pose_filename, sensors_pose_file;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("start_phase,s", po::value<float>(&start_phase)->required(), "Beginning of phase range.")
    ("end_phase,e", po::value<float>(&end_phase)->required(), "End of phase range.")
    ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.")
    ("calibration,c", po::value<string>(&sensors_pose_file)->default_value(""), "Sensors poses (calibration).")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

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

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> slice_poses;
  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  float start_phase, end_phase;
  string out_dir;
  if(!parse_arguments(argc, argv, calibration, clouds_to_process, start_phase, end_phase, out_dir)) {
    return EXIT_FAILURE;
  }

  PhaseFilter filter(start_phase, end_phase);

  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      VelodynePointCloud cloud = *multiframe.clouds[sensor_i];
      VelodynePointCloud slice;
      filter.filter(cloud, slice);
      boost::filesystem::path cloud_fn(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + cloud_fn.filename().string(), slice);
    }
    cout << endl;
  }

  return EXIT_SUCCESS;
}
