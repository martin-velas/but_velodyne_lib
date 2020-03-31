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
    SensorsCalibration &calibration,
    vector<string> &clouds_to_process,
    string &out_dir) {

  string pose_filename, sensors_pose_file;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
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
  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }
  return true;
}

void getSlices(const VelodynePointCloud &in_cloud, vector<VelodynePointCloud> &slices) {
  map<float, VelodynePointCloud> slices_map;
  for(VelodynePointCloud::const_iterator pt = in_cloud.begin(); pt < in_cloud.end(); pt++) {
    slices_map[pt->phase].push_back(*pt);
  }
  for(map<float, VelodynePointCloud>::const_iterator s = slices_map.begin(); s != slices_map.end(); s++) {
    slices.push_back(s->second);
  }
}

int phaseToInt(const float phase) {
  return MIN(phase*100, 99);
}

void fix_cloud(const int frame_idx, const VelodynePointCloud &in_cloud,
    const Eigen::Affine3f &sensor_pose,
    const Eigen::Affine3f &delta_pose,
    VelodynePointCloud &out_cloud) {
  VelodynePointCloud in_cloud_calibrated;
  transformPointCloud(in_cloud, in_cloud_calibrated, sensor_pose);

  vector<VelodynePointCloud> slices;
  getSlices(in_cloud_calibrated, slices);

  LinearInterpolationSE3 interpolation(Eigen::Affine3f::Identity(), delta_pose);
  for(int i = 0; i < slices.size(); i++) {
    Eigen::Affine3f t = interpolation.estimate(((float) i)/slices.size());
    transformPointCloud(slices[i], slices[i], t);
    out_cloud += slices[i];
    cout << frame_idx << " " << phaseToInt(slices[i].front().phase) << " " << t << endl;
  }

  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
  if(!in_cloud.getAxisCorrection().isIdentity()) {
    transformPointCloud(out_cloud, out_cloud, in_cloud.getAxisCorrection().inverse());
  }
}

void fix_cloud(const int frame_idx, const VelodynePointCloud &in_cloud,
    const BSplineSE3 &spline,
    const Eigen::Affine3f &sensor_pose,
    VelodynePointCloud &out_cloud) {
  transformPointCloud(in_cloud, out_cloud, sensor_pose);

  typedef map<float, Eigen::Affine3f> CB;
  CB code_book;
  const Eigen::Affine3f &T0_inv = spline.estimate(0.0).inverse();
  for(VelodynePointCloud::iterator p = out_cloud.begin(); p < out_cloud.end(); p++) {
    Eigen::Affine3f t;
    CB::iterator t_found = code_book.find(p->phase);
    if(t_found == code_book.end()) {
      t = T0_inv*spline.estimate(p->phase);
      code_book[p->phase] = t;
      cout << frame_idx << " " << phaseToInt(p->phase) << " " << t << endl;
    } else {
      t = t_found->second;
    }
    *p = transformPoint(*p, t);
  }

  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  string out_dir;
  if(!parse_arguments(argc, argv, poses, calibration, clouds_to_process, out_dir)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0; file_sequence.hasNext() && frame_i+1 < poses.size(); frame_i++) {

    VelodyneMultiFrame multiframe = file_sequence.getNext();

    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      VelodynePointCloud out_cloud;
      const Eigen::Affine3f &sensor_pose = multiframe.calibration.ofSensor(sensor_i);

      if(frame_i > 0 && frame_i+2 < poses.size()) {
        vector<Eigen::Affine3f> control_poses(poses.begin()+(frame_i-1), poses.begin()+(frame_i+3));
        const BSplineSE3 spline(control_poses);
        fix_cloud(frame_i, *multiframe.clouds[sensor_i], spline, sensor_pose, out_cloud);
      } else {
        const Eigen::Affine3f poses_delta = poses[frame_i].inverse()*poses[frame_i+1];
        fix_cloud(frame_i, *multiframe.clouds[sensor_i], sensor_pose, poses_delta, out_cloud);
      }

      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + first_cloud.filename().string(), out_cloud);
    }
  }

  return EXIT_SUCCESS;
}
