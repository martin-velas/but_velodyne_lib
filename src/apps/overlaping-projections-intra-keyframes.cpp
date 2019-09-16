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
#include <but_velodyne/Overlap.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     vector<int> &keyframe_indices,
                     float &depth_quantile,
                     float &depth_relative_tolerance, float &depth_absolute_tolerance,
                     bool &average_with_zbuffer_occupancy,
                     bool &visualize) {
  string pose_filename, sensor_poses_filename, keyframe_indices_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("keyframes,k", po::value<string>(&keyframe_indices_filename)->required(),
        "File with keyframe indices.")
    ("depth_quantile", po::value<float>(&depth_quantile)->default_value(0.0),
        "Quantile of depth within the bin (0.5 for median, 0.0 for minimum, 1.0 for maximum.")
    ("depth_relative_tolerance", po::value<float>(&depth_relative_tolerance)->default_value(0.1),
        "Relative tolerance of the depth in Z-buffer.")
    ("depth_absolute_tolerance", po::value<float>(&depth_absolute_tolerance)->default_value(0.3),
        "Absolute tolerance of the depth in Z-buffer.")
    ("visualize", po::bool_switch(&visualize), "Run visualization.")
    ("average_with_zbuffer_occupancy", po::bool_switch(&average_with_zbuffer_occupancy),
        "Average with nuber of occupied cells in the Z-buffer.")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensor_poses_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensor_poses_filename);
  }

  load_vector_from_file(keyframe_indices_filename, keyframe_indices);

  return true;
}

float harmonic_avg(const float a, const float b) {
  if(-0.0001 < a+b && a+b < 0.0001) {
    return 0;
  } else {
    return 2.0*a*b/(a+b);
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  vector<int> keyframe_indices;
  float depth_quantile, depth_relative_tolerance, depth_absolute_tolerance;
  bool visualize, average_with_zbuffer_occupancy;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, keyframe_indices,
      depth_quantile, depth_relative_tolerance, depth_absolute_tolerance,
      average_with_zbuffer_occupancy, visualize)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  Visualizer3D::Ptr vis;

  if(keyframe_indices.back() != (sequence.size()-1)) {
    keyframe_indices.push_back(sequence.size()-1);
  }

  for(vector<int>::const_iterator ki = keyframe_indices.begin(); ki+1 < keyframe_indices.end(); ki++) {
    for(int src_i = *ki; src_i < *(ki+1); src_i++) {

      VelodyneMultiFrame src_frame = sequence[src_i];
      PointCloud<VelodynePoint> src_cloud;
      src_frame.joinTo(src_cloud);
      transformPointCloud(src_cloud, src_cloud, Eigen::Affine3f(poses[src_i].rotation()));
      SphericalZbuffer src_zbuffer(src_cloud, 180, 90, depth_quantile);

      int fibonacci_prev = 1;
      int fibonacci_curr = 2;
      while(src_i+fibonacci_curr <= *(ki+1)) {
        int trg_i = src_i+fibonacci_curr;

        VelodyneMultiFrame trg_frame = sequence[trg_i];
        PointCloud<VelodynePoint> trg_cloud;
        trg_frame.joinTo(trg_cloud);
        transformPointCloud(trg_cloud, trg_cloud, Eigen::Affine3f(poses[trg_i].rotation()));
        Eigen::Vector3f translation = poses[trg_i].translation() - poses[src_i].translation();
        transformPointCloud(trg_cloud, trg_cloud, translation, Eigen::Quaternionf::Identity());

        size_t points_within;
        if(visualize) {
          PointCloud<VelodynePoint> within, rest;
          points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance,
                                                     within, rest);
          if(!vis) {
            vis.reset(new Visualizer3D);
          }
          vis->keepOnlyClouds(0)
              .setColor(0, 0, 0).addPointCloud(src_cloud)
              .setColor(0, 200, 0).addPointCloud(within)
              .setColor(255, 0, 0).addPointCloud(rest)
              .setColor(255, 0, 255);
          src_zbuffer.addToVisualizer(*vis, src_cloud, depth_relative_tolerance, depth_absolute_tolerance);
        } else {
          points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance);
        }

        float overlap;
        float points_within_portion = (float (points_within)) / trg_cloud.size();
        if(average_with_zbuffer_occupancy) {
          float visited_portion = src_zbuffer.visitedPortion();
          overlap = harmonic_avg(visited_portion, points_within_portion);
        } else {
          overlap = points_within_portion;
        }

        cout << src_i << " " << trg_i << " " << overlap << endl;

        if(visualize) {
          vis->show();
        }

        int fibonacci_next = fibonacci_prev+fibonacci_curr;
        fibonacci_prev = fibonacci_curr;
        fibonacci_curr = fibonacci_next;
      }
    }
  }

  return EXIT_SUCCESS;
}
