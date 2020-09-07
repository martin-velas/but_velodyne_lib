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
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(
    int argc, char **argv,
    vector<Eigen::Affine3f> &poses,
    vector<float> &timestamps,
    SensorsCalibration &calibration,
    vector<float> &frame_borders,
    vector<string> &clouds_to_process,
    string &out_dir) {

  string pose_filename, sensors_pose_file, timestamps_filename, frame_borders_filename;

  po::options_description desc(
      "Fine correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Reference(s): ???\n"
          " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("timestamps_file,t", po::value<string>(&timestamps_filename)->required(), "Timestamps file.")
      ("frame_borders_file,b", po::value<string>(&frame_borders_filename)->required(), "Frame borders file.")
      ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.")
      ("sensors_pose_file,s", po::value<string>(&sensors_pose_file)->default_value(""), "Sensors poses (calibration).")
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
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);
  load_vector_from_file(timestamps_filename, timestamps);
  assert(poses.size() == timestamps.size());

  load_vector_from_file(frame_borders_filename, frame_borders);

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  return true;
}

struct float_less {
  bool operator()(const float &a, const float &b) {
    const float EPS = 1e-4;
    return (a < b) && (b - a) > EPS;
  }
};

typedef map<float, VelodynePointCloud, float_less> SliceMap;

void getSlices(const VelodynePointCloud &in_cloud, SliceMap &slices) {
  for(VelodynePointCloud::const_iterator pt = in_cloud.begin(); pt < in_cloud.end(); pt++) {
    slices[pt->phase].push_back(*pt);
  }
}

Eigen::Affine3f get_interpolated_pose(const vector<Eigen::Affine3f> &poses,
    const vector<float> &timestamps, const float time) {
  if(time < timestamps.front() || time > timestamps.back()) {
    PCL_ERROR("No pose for time %f\n", time);
    return Eigen::Affine3f::Identity();
  }
  int before = 0;
  int after = poses.size()-1;
  while(before + 1 < after) {
    int middle = (before + after) / 2;
    if(timestamps[middle] <= time) {
      before = middle;
    } else {
      after = middle;
    }
  }
  float alpha = (time - timestamps[before]) / (timestamps[after] - timestamps[before]);
  return LinearInterpolationSE3(poses[before], poses[after]).estimate(alpha);
}

void fix_cloud(const VelodynePointCloud &in_cloud,
    const float start, const float end,
    const Eigen::Affine3f &sensor_pose,
    const vector<Eigen::Affine3f> &poses,
    const vector<float> &timestamps,
    VelodynePointCloud &out_cloud) {
  SliceMap slices;
  getSlices(in_cloud, slices);

  const Eigen::Affine3f start_pose = get_interpolated_pose(poses, timestamps, start);
  cout << start_pose << endl;
  const Eigen::Affine3f start_pose_inv = start_pose.inverse();

  const Eigen::Affine3f sensor_pose_inv = sensor_pose.inverse();
  for(SliceMap::iterator s = slices.begin(); s != slices.end(); s++) {
    if(s->second.empty()) {
      continue;
    }
    const float phase = s->second.front().phase;
    const float time = start + (end-start) * phase;
    const Eigen::Affine3f interpolated_pose = get_interpolated_pose(poses, timestamps, time);

    const Eigen::Affine3f fix_pose = sensor_pose_inv * start_pose_inv * interpolated_pose * sensor_pose;

    transformPointCloud(s->second, s->second, fix_pose);
    out_cloud += s->second;
  }
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  vector<float> timestamps, frame_borders;
  string out_dir;
  if(!parse_arguments(argc, argv, poses, timestamps, calibration, frame_borders,
      clouds_to_process, out_dir)) {
    return EXIT_FAILURE;
  }

  int output_count = MIN(clouds_to_process.size(), poses.size()) - 1;
  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0;
      file_sequence.hasNext() && frame_i+1 < poses.size() && frame_i+1 < frame_borders.size();
      frame_i++) {

    VelodyneMultiFrame multiframe = file_sequence.getNext();
    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      VelodynePointCloud out_cloud;
      Eigen::Affine3f sensor_pose = multiframe.calibration.ofSensor(sensor_i);
      fix_cloud(*multiframe.clouds[sensor_i], frame_borders[frame_i], frame_borders[frame_i+1],
          sensor_pose, poses, timestamps, out_cloud);

      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + first_cloud.filename().string(), out_cloud);
    }
  }

  return EXIT_SUCCESS;
}
