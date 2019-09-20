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
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

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

void fix_cloud(const VelodynePointCloud &in_cloud,
    InterpolationBuffered &interpolation,
    const Eigen::Affine3f &sensor_pose,
    VelodynePointCloud &out_cloud) {
  transformPointCloud(in_cloud, out_cloud, sensor_pose);
  for(VelodynePointCloud::iterator p = out_cloud.begin(); p < out_cloud.end(); p++) {
    *p = transformPoint(*p, interpolation.estimate(p->phase));
  }
  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
}

void get_control_points(const vector<Eigen::Affine3f> &slice_poses, const int t1_idx,
    const int slices_cnt, vector<Eigen::Affine3f> &control_points) {
  const Eigen::Affine3f relative_pose_inv = slice_poses[(t1_idx/slices_cnt)*slices_cnt].inverse();
  for(int i = t1_idx-1; i <= t1_idx+2; i++) {
    control_points.push_back(relative_pose_inv*slice_poses[i]);
  }
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

  float slice_fraction = 1.0/slices_cnt;

  vector<PhaseFilter> filters;
  float range_start = 0.0;
  for(int i = 0; i < slices_cnt; i++) {
    float range_end = range_start+slice_fraction;
    filters.push_back(PhaseFilter(range_start, range_end));
    range_start = range_end;
  }

  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0; file_sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = file_sequence.getNext();
    const Eigen::Affine3f &first_inv = slice_poses[frame_i*slices_cnt].inverse();

    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {

      const Eigen::Affine3f &sensor_pose = multiframe.calibration.ofSensor(sensor_i);

      VelodynePointCloud out_cloud;
      for(int slice_i = 0; slice_i < slices_cnt; slice_i++) {
        VelodynePointCloud in_cloud_part = *multiframe.clouds[sensor_i];
        const PhaseFilter &filter = filters[slice_i];
        filter.filter(in_cloud_part);

        int pose_i = frame_i*slices_cnt + slice_i;
        InterpolationSE3::Ptr interpolation;

        if(pose_i == 0 || pose_i == slice_poses.size()-2) {
          Eigen::Affine3f current = first_inv * slice_poses[pose_i];
          Eigen::Affine3f next = first_inv * slice_poses[pose_i+1];
          interpolation.reset(new LinearInterpolationSE3(current, next));

        } else if(pose_i >= slice_poses.size()-1) {
          PCL_ERROR("ERROR, this should not happen (pose_i >= slice_poses.size()-1)");
          return EXIT_FAILURE;

        } else {
          vector<Eigen::Affine3f> control_points;
          get_control_points(slice_poses, pose_i, slices_cnt, control_points);
          interpolation.reset(new BSplineSE3(control_points));
        }

        float phase_scale = 1.0 / (filter.getMaxPhase() - filter.getMinPhase());
        InterpolationBuffered interpolation_buffered(interpolation, -filter.getMinPhase(), phase_scale);

        VelodynePointCloud out_cloud_part;
        fix_cloud(in_cloud_part, interpolation_buffered, sensor_pose, out_cloud_part);
        out_cloud += out_cloud_part;
      }

      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + first_cloud.filename().string(), out_cloud);
    }
  }

  return EXIT_SUCCESS;
}
