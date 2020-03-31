/*
 * Overlaps estimation among point cloud subsequences for CLS registration.
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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                      vector<Eigen::Affine3f> &src_poses,
                      vector<Eigen::Affine3f> &trg_poses,
                      vector<string> &src_clouds,
                      vector<string> &trg_clouds,
                      SensorsCalibration &calibration,
                      float &dist_threshold, float &sampling) {
  string src_poses_fn, trg_poses_fn, src_clouds_fn, trg_clouds_fn, calibration_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("src_poses", po::value<string>(&src_poses_fn)->required(), "KITTI poses file for source clouds.")
      ("trg_poses", po::value<string>(&trg_poses_fn)->required(), "KITTI poses file for target clouds.")
      ("src_clouds", po::value<string>(&src_clouds_fn)->required(), "List of source clouds.")
      ("trg_clouds", po::value<string>(&trg_clouds_fn)->required(), "List of target clouds.")
      ("calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Sensors calibration file.")
      ("distance,d", po::value<float>(&dist_threshold)->default_value(0.1), "Sensors calibration file.")
      ("sampling,s", po::value<float>(&sampling)->default_value(0.1), "Point cloud sampling.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if(src_poses.size() != src_clouds.size() || trg_poses.size() != trg_clouds.size()) {
    cerr << "ERROR: Number of poses (" << src_poses.size() << ", " << trg_poses.size() << ") differs from number of point clouds (" <<
        src_clouds.size() << ", " << trg_clouds.size() << ")." << endl;
    return false;
  }

  src_poses = KittiUtils::load_kitti_poses(src_poses_fn);
  trg_poses = KittiUtils::load_kitti_poses(trg_poses_fn);

  load_vector_from_file(src_clouds_fn, src_clouds);
  load_vector_from_file(trg_clouds_fn, trg_clouds);

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  return true;
}

void build_sum_pcd(VelodyneFileSequence &sequence, const vector<Eigen::Affine3f> &poses,
    PointCloud<PointXYZ>::Ptr sum_cloud, const float sampling) {
  for(int i = 0; i < poses.size() && sequence.hasNext(); i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PointCloud<PointXYZ>::Ptr part_cloud(new PointCloud<PointXYZ>);
    multiframe.joinTo(*part_cloud);
    subsample_cloud<PointXYZ>(part_cloud, sampling);
    transformPointCloud(*part_cloud, *part_cloud, poses[i]);
    *sum_cloud += *part_cloud;
  }
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> src_poses, trg_poses;
  vector<string> src_clouds_fn, trg_clouds_fn;
  SensorsCalibration calibration;
  float dist_threshold, sampling;

  if(!parse_arguments(argc, argv,
      src_poses, trg_poses, src_clouds_fn, trg_clouds_fn, calibration, dist_threshold, sampling)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence src_sequence(src_clouds_fn, calibration);
  VelodyneFileSequence trg_sequence(trg_clouds_fn, calibration);

  PointCloud<PointXYZ>::Ptr src_sum_cloud(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr trg_sum_cloud(new PointCloud<PointXYZ>);
  build_sum_pcd(src_sequence, src_poses, src_sum_cloud, sampling);
  build_sum_pcd(trg_sequence, trg_poses, trg_sum_cloud, sampling);

  KdTreeFLANN<PointXYZ> src_tree;
  src_tree.setInputCloud(src_sum_cloud);
  float dist_threshold_sq = dist_threshold*dist_threshold;
  int overlaping_pts = 0;
  for(PointCloud<PointXYZ>::const_iterator trg_pt = trg_sum_cloud->begin(); trg_pt < trg_sum_cloud->end(); trg_pt++) {
    vector<int> indices(1);
    vector<float> distances(1);
    src_tree.nearestKSearch(*trg_pt, 1, indices, distances);
    if(distances.front() < dist_threshold_sq) {
      overlaping_pts++;
    }
  }

  cout << (float (overlaping_pts))/trg_sum_cloud->size() << endl;

  return EXIT_SUCCESS;
}
