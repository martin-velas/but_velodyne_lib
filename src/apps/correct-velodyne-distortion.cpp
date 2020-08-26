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
#include <but_velodyne/VelodyneMultiFrameSequence.h>
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
  }

  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
  if(!in_cloud.getAxisCorrection().isIdentity()) {
    transformPointCloud(out_cloud, out_cloud, in_cloud.getAxisCorrection().inverse());
  }
}

void fix_cloud(const int frame_idx, const LineCloud &in_line_cloud,
               const Eigen::Affine3f &sensor_pose,
               const Eigen::Affine3f &delta_pose,
               LineCloud &out_line_cloud) {
  LineCloud in_line_cloud_calibrated;
  in_line_cloud.transform(sensor_pose.matrix(), out_line_cloud);

  LinearInterpolationSE3::Ptr interpolation_method(
          new LinearInterpolationSE3(Eigen::Affine3f::Identity(), delta_pose));
  InterpolationBuffered interpolation(interpolation_method, 0.0, 1.0);

  for(LineCloud::iterator l = out_line_cloud.begin(); l < out_line_cloud.end(); l++) {
    Eigen::Affine3f t = interpolation.estimate(l->phase);
    *l = l->transform(t);
  }

  out_line_cloud.transform(sensor_pose.inverse().matrix());
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
    } else {
      t = t_found->second;
    }
    *p = transformPoint(*p, t);
  }

  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
}

void fix_cloud(const int frame_idx, const LineCloud &in_line_cloud,
               const BSplineSE3 &spline,
               const Eigen::Affine3f &sensor_pose,
               LineCloud &out_line_cloud) {
  in_line_cloud.transform(sensor_pose.matrix(), out_line_cloud);

  typedef map<float, Eigen::Affine3f> CB;
  CB code_book;
  const Eigen::Affine3f &T0_inv = spline.estimate(0.0).inverse();
  for(LineCloud::iterator l = out_line_cloud.begin(); l < out_line_cloud.end(); l++) {
    Eigen::Affine3f t;
    CB::iterator t_found = code_book.find(l->phase);
    if(t_found == code_book.end()) {
      t = T0_inv*spline.estimate(l->phase);
      code_book[l->phase] = t;
    } else {
      t = t_found->second;
    }
    *l = l->transform(t);
  }

  out_line_cloud.transform(sensor_pose.inverse().matrix());
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
      LineCloud out_line_cloud;
      const Eigen::Affine3f &sensor_pose = multiframe.calibration.ofSensor(sensor_i);

      if(frame_i > 0 && frame_i+2 < poses.size()) {
        vector<Eigen::Affine3f> control_poses(poses.begin()+(frame_i-1), poses.begin()+(frame_i+3));
        const BSplineSE3 spline(control_poses);
        if(multiframe.hasPointClouds()) {
          fix_cloud(frame_i, *multiframe.clouds[sensor_i], spline, sensor_pose, out_cloud);
        } else {
          fix_cloud(frame_i, *multiframe.line_clouds[sensor_i], spline, sensor_pose, out_line_cloud);
        }
      } else {
        const Eigen::Affine3f poses_delta = poses[frame_i].inverse()*poses[frame_i+1];
        if(multiframe.hasPointClouds()) {
          fix_cloud(frame_i, *multiframe.clouds[sensor_i], sensor_pose, poses_delta, out_cloud);
        } else {
          fix_cloud(frame_i, *multiframe.line_clouds[sensor_i], sensor_pose, poses_delta, out_line_cloud);
        }
      }

      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      string out_filename = out_dir + "/" + first_cloud.filename().string();
      if(multiframe.hasPointClouds()) {
        io::savePCDFileBinary(out_filename, out_cloud);
      } else {
        ofstream out_line_cloud_file(out_filename.c_str());
        out_line_cloud_file << out_line_cloud;
      }
    }
  }

  return EXIT_SUCCESS;
}
