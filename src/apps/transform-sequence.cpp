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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/point_types.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointWithSource PointType;

const float EPS = 1e-6;
const float VELODYNE_FPS = 10.0;

bool parse_arguments(int argc, char **argv,
                     map<float, Eigen::Affine3f> &poses_timed,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_proces,
                     vector<float> &frame_borders,
                     int &poses_frequency,
                     string &output_dir) {
  string pose_filename, sensor_poses_filename, frame_borders_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_timed,p", po::value<string>(&pose_filename)->required(), "KITTI poses with times.")
      ("output_dir,o", po::value<string>(&output_dir)->required(), "Output directory for PCD files")
      ("calibration,c", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
      ("frame_borders,b", po::value<string>(&frame_borders_filename)->required(), "Frame borders.")
      ("poses_frequency,f", po::value<int>(&poses_frequency)->required(), "Frequency of poses per second.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_proces = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_proces.size() < 1) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  KittiUtils::load_kitti_poses_with_keys(pose_filename, poses_timed);

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  load_vector_from_file(frame_borders_filename, frame_borders);
  if(clouds_to_proces.size() != (frame_borders.size()-1)*calibration.sensorsCount()) {
    cerr << "Invalid number of frame borders (" << frame_borders.size() << ") for " << clouds_to_proces.size()
        << " point clouds. Expecting " << clouds_to_proces.size()/calibration.sensorsCount()+1 << " frame borders." << endl;
    return false;
  }

  return true;
}

class Fixer {

public:
  Fixer(const map<float, Eigen::Affine3f> &poses_timed, const int poses_frequency) :
    poses(poses_timed), poses_time_delta(1.0 / poses_frequency) {
  }

  void fix(VelodynePointCloud::Ptr cloud, const float start_time) const {
    for(VelodynePointCloud::iterator p = cloud->begin(); p < cloud->end(); p++) {
      const float time = start_time + p->phase*(1.0/VELODYNE_FPS);
      Eigen::Affine3f pose = getPose(time);
      *p = transformPoint(*p, pose);
    }
  }

protected:
  Eigen::Affine3f getPose(const float time) const {
    return poses.lower_bound(time-poses_time_delta/2.0)->second;
  }

private:
  map<float, Eigen::Affine3f> poses;
  const float poses_time_delta;
};

int main(int argc, char** argv) {

  vector<string> filenames;
  map<float, Eigen::Affine3f> poses_timed;
  SensorsCalibration calibration;
  vector<float> frame_borders;
  int poses_frequency;
  string output_dir;

  if(!parse_arguments(argc, argv,
      poses_timed, calibration, filenames, frame_borders, poses_frequency, output_dir)) {
    return EXIT_FAILURE;
  }

  Fixer fixer(poses_timed, poses_frequency);

  map<float, Eigen::Affine3f>::iterator first = poses_timed.begin();
  map<float, Eigen::Affine3f>::iterator second = first;
  second++;
  const float imu_dt = second->first - first->first;

  VelodyneFileSequence file_sequence(filenames, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    for(int ci = 0; ci < calibration.sensorsCount(); ci++) {
      transformPointCloud(*multiframe.clouds[ci], *multiframe.clouds[ci], calibration.ofSensor(ci));
      fixer.fix(multiframe.clouds[ci], frame_borders[frame_i]);
      transformPointCloud(*multiframe.clouds[ci], *multiframe.clouds[ci], calibration.ofSensor(ci).inverse());

      boost::filesystem::path cloud_file(multiframe.filenames[ci]);
      string basename = output_dir + "/" + cloud_file.filename().string();
      io::savePCDFileBinary(basename, *multiframe.clouds[ci]);
    }
  }

  return EXIT_SUCCESS;
}
