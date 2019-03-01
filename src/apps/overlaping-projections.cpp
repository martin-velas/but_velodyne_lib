/*
 * Overlaps estimation among point clouds for CLS registration.
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
                     int &source_index,
                     int &frames_dist, int &max_frames_dist, bool &circular,
                     float &depth_quantile,
                     float &depth_relative_tolerance, float &depth_absolute_tolerance,
                     bool &average_with_zbuffer_occupancy,
                     bool &visualize) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("src_idx", po::value<int>(&source_index)->default_value(-1), "Source index, if not set, all clouds are considered as source")
    ("frames_dist", po::value<int>(&frames_dist)->default_value(1), "Frames distance (minimal of max is set) to compute overlap")
    ("max_frames_dist", po::value<int>(&max_frames_dist)->default_value(-1), "Maximal frames distance. If not set, frames_dist is used.")
    ("circular", po::bool_switch(&circular), "The trajectory is considered to be circular.")
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

  if(max_frames_dist < 0) {
    max_frames_dist = frames_dist;
  }

  return true;
}

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular) {
  if(!circular) {
    return abs(i-j);
  } else {
    return MIN(abs(i-j), frames_count-abs(i-j));
  }
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
  int source_index;
  int expected_frames_dist, max_frames_dist;
  bool circular;
  float depth_quantile, depth_relative_tolerance, depth_absolute_tolerance;
  bool visualize, average_with_zbuffer_occupancy;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, source_index, expected_frames_dist, max_frames_dist, circular,
      depth_quantile, depth_relative_tolerance, depth_absolute_tolerance,
      average_with_zbuffer_occupancy, visualize)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  Visualizer3D::Ptr vis;

  for(int i = 0; i < sequence.size(); i++) {
    if(source_index >= 0) {
      i = source_index;
    }
    VelodyneMultiFrame src_frame = sequence[i];
    PointCloud<VelodynePoint> src_cloud;
    src_frame.joinTo(src_cloud);
    transformPointCloud(src_cloud, src_cloud, Eigen::Affine3f(poses[i].rotation()));
    SphericalZbuffer src_zbuffer(src_cloud, 180, 90, depth_quantile);

    for(int j = i+1; j < sequence.size(); j++) {
      int frames_distace = get_frames_distance(i, j, sequence.size(), circular);
      if(expected_frames_dist <= frames_distace && frames_distace <= max_frames_dist) {

        VelodyneMultiFrame trg_frame = sequence[j];
        PointCloud<VelodynePoint> trg_cloud;
        trg_frame.joinTo(trg_cloud);
        transformPointCloud(trg_cloud, trg_cloud, Eigen::Affine3f(poses[j].rotation()));
        Eigen::Vector3f translation = poses[j].translation() - poses[i].translation();
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

        cout << i << " " << j << " " << overlap << endl;

        if(visualize) {
          vis->show();
        }
      }
    }
    if(source_index >= 0) {
      break;
    }
  }

  return EXIT_SUCCESS;
}
