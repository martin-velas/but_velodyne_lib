/*
 * Visualization of KITTI poses file.
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
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointWithSource PointType;

bool parse_arguments(int argc, char **argv,
                     float &sampling_ratio,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     string &output_file,
                     vector<bool> &mask,
                     float &range_threshold) {
  string pose_filename, skip_filename, sensor_poses_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sampling_ratio,s", po::value<float>(&sampling_ratio)->default_value(0.1), "Reduce size with this ratio.")
      ("output_file,o", po::value<string>(&output_file)->required(), "Output PCD file")
      ("skip_file,k", po::value<string>(&skip_filename)->default_value(""), "File with indices to skip")
      ("sensor_poses", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
      ("range_threshold,r", po::value<float>(&range_threshold)->default_value(1e10), "Maximal range of the point.")
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

  mask.resize(poses.size(), true);
  if(!skip_filename.empty()) {
    ifstream skip_file(skip_filename.c_str());
    string line;
    while(getline(skip_file, line)) {
      mask[atoi(line.c_str())] = false;
    }
  }

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

void subsample_cloud2(PointCloud<PointXYZI> &cloud, float leaf_size, int upsampling_ratio) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
  rgb_cloud = Visualizer3D::colorizeCloud(cloud, true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_downsample(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i = 0; i < upsampling_ratio; i++) {
    float offset = leaf_size / upsampling_ratio * i;
    Eigen::Affine3f t = getTransformation(offset, offset, offset, 0, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZRGB>);
    transformPointCloud(*rgb_cloud, *cloud_grid, t);

    pcl::VoxelGrid<PointXYZRGB> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud_grid);
    grid.filter(*cloud_grid);

    transformPointCloud(*cloud_grid, *cloud_grid, t.inverse());
    *cloud_to_downsample += *cloud_grid;
  }

  cloud.clear();
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = cloud_to_downsample->begin(); pt < cloud_to_downsample->end(); pt++) {
    PointXYZI pt_grey;
    copyXYZ(*pt, pt_grey);
    pt_grey.intensity = pt->r / 255.0;
    cloud.push_back(pt_grey);
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  string output_pcd_file;
  float sampling_ratio, range_threshold;
  vector<bool> mask;

  if(!parse_arguments(argc, argv,
      sampling_ratio,
      poses, calibration, filenames,
      output_pcd_file,
      mask,
      range_threshold)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointType> sum_cloud;
  PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
  VelodyneFileSequence file_sequence(filenames, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    if(frame_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << frame_i << std::endl << std::flush;
      break;
    }

    VelodyneMultiFrame multiframe = file_sequence.getNext();
    if(mask[frame_i]) {
      multiframe.joinTo(*cloud);
      filter_by_range(*cloud, *cloud, range_threshold);
      subsample_cloud<PointType>(cloud, sampling_ratio);
      transformPointCloud(*cloud, *cloud, poses[frame_i]);
      for(PointCloud<PointType>::iterator pt = cloud->begin(); pt < cloud->end(); pt++) {
        pt->source = pt->source*poses.size() + frame_i;
      }
      sum_cloud += *cloud;
      cloud->clear();
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
  rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);

  io::savePCDFileBinary(output_pcd_file, sum_cloud);
  io::savePCDFileBinary(output_pcd_file + ".rgb.pcd", *rgb_cloud);

  PointCloud<PointXYZ> poses_cloud;
  for(vector<Eigen::Affine3f>::const_iterator p = poses.begin(); p < poses.end(); p++) {
    poses_cloud.push_back(KittiUtils::positionFromPose(*p));
  }
  io::savePCDFileBinary(output_pcd_file + ".poses.pcd", poses_cloud);

  return EXIT_SUCCESS;
}
