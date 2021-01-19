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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/FrameCorrectionPerParts.h>

#include <pcl/common/eigen.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    vector<Eigen::Affine3f> &slice_poses,
    SensorsCalibration &calibration,
    vector<string> &clouds_to_process,
    int &slices_cnt,
    string &out_dir) {

  string pose_filename, sensors_pose_file;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("sub_poses,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.")
    ("calibration,c", po::value<string>(&sensors_pose_file)->default_value(""), "Sensors poses (calibration).")
    ("slices,s", po::value<int>(&slices_cnt)->required(), "Slices count.")
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

  slice_poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  if(clouds_to_process.size() == 0 ||
      clouds_to_process.size()*slices_cnt != (slice_poses.size()-1)*calibration.sensorsCount()) {
    cerr << "Invalid number of clouds: " << clouds_to_process.size() << " for " << slice_poses.size() << " half-poses."
        << endl << desc << endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> slice_poses;
  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  int slices_cnt;
  string out_dir;
  if(!parse_arguments(argc, argv, slice_poses, calibration, clouds_to_process,
      slices_cnt, out_dir)) {
    return EXIT_FAILURE;
  }

  FrameCorrectionPerParts correction(slice_poses, slices_cnt);

  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0; file_sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = file_sequence.getNext();

    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      VelodynePointCloud out_cloud;
      if(!correction.fixFrame(multiframe, frame_i, sensor_i, out_cloud)) {
        return EXIT_FAILURE;
      }

      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + first_cloud.filename().string(), out_cloud);
    }
  }

  return EXIT_SUCCESS;
}
