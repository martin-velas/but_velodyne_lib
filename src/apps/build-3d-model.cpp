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

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointWithSource PointType;

class RoNI {

public:

    RoNI(const float x_, const float y_, const float z_, const float radius_) :
            center(x_, y_, z_), radius(radius_) {
    }

    static bool load(const string filename, vector<RoNI> &ronis) {
      std::ifstream ronis_file(filename.c_str());
      if(!ronis_file.is_open()) {
        std::perror((std::string("Unable to open file: ") + filename).c_str());
        return false;
      }

      while(true) {
        float x, y, z, radius;
        ronis_file >> x >> y >> z >> radius;
        if(ronis_file.eof()) {
          break;
        } else {
          ronis.push_back(RoNI(x, y, z, radius));
        }
      }
      return true;
    }

    static void filter(const PointCloud<PointType> &input, const vector<RoNI> &ronis, PointCloud<PointType> &output) {
      if(input.size() != output.size()) {
        output.resize(input.size());
      }
      int out_i = 0;
      for(PointCloud<PointType>::const_iterator in_pt = input.begin(); in_pt < input.end(); in_pt++) {
        if(!RoNI::isWithin(*in_pt, ronis)) {
          output[out_i++] = *in_pt;
        }
      }
      output.erase(output.begin()+out_i, output.end());
    }

    static bool isWithin(const PointType &pt, const vector<RoNI> &ronis) {
      for(vector<RoNI>::const_iterator r = ronis.begin(); r < ronis.end(); r++) {
        if(r->isWithin(pt)) {
          return true;
        }
      }
      return false;
    }

    bool isWithin(const PointType &pt) const {
      return (pt.getVector3fMap() - center).norm() < radius;
    }

private:

    Eigen::Vector3f center;
    float radius;
};

bool parse_arguments(int argc, char **argv,
                     float &sampling_ratio,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     string &output_file,
                     vector<bool> &mask,
                     float &range_threshold,
                     float &range_seg_leaf_size, float &range_seg_relative_diff,
                     vector<RoNI> &regions_of_no_interest) {
  string pose_filename, skip_filename, sensor_poses_filename, regions_of_no_interest_file;

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
      ("range_seg_leaf_size", po::value<float>(&range_seg_leaf_size)->default_value(-1.0), "Leaf size for range segmentation (negative = disabled).")
      ("range_seg_relative_diff", po::value<float>(&range_seg_relative_diff)->default_value(2.0), "Relative difference for range segmentation.")
      ("ronis", po::value<string>(&regions_of_no_interest_file)->default_value(""), "Regions of no interest to be erased. Line: \"x y z radius\".")
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

  if(!regions_of_no_interest_file.empty()) {
    if(!RoNI::load(regions_of_no_interest_file, regions_of_no_interest)) {
      return false;
    }
  }

  return true;
}

void subsampleCloud(PointCloud<PointXYZ>::Ptr cloud, float voxel_resolution) {
  pcl::VoxelGrid<PointXYZ> grid;
  grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
  grid.setInputCloud(cloud);
  grid.filter(*cloud);
}

void range_segmentation(PointCloud<PointType>::ConstPtr input_cloud,
    const vector<Eigen::Affine3f> &poses,
    const float leaf_size, const float relative_range_diff,
    PointCloud<PointType>::Ptr output_cloud, PointCloud<PointType>::Ptr removed_points) {

  PointCloud<PointXYZ>::Ptr voxels(new PointCloud<PointXYZ>);
  copyPointCloud(*input_cloud, *voxels);
  subsampleCloud(voxels, leaf_size);

  vector< PointCloud<PointType> > clusters(voxels->size());
  vector< vector<float> > cluster_ranges(voxels->size());
  vector<float> cluster_min_range(voxels->size(), INFINITY);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(voxels);
  int K = 1;
  std::vector<int> pointIdx(K);
  std::vector<float> pointDist(K);

  for(PointCloud<PointType>::const_iterator pt = input_cloud->begin(); pt < input_cloud->end(); pt++) {
    pcl::PointXYZ searchPoint(pt->x, pt->y, pt->z);
    kdtree.nearestKSearch(searchPoint, K, pointIdx, pointDist);
    const int idx = pointIdx[0];
    clusters[idx].push_back(*pt);
    float range = (pt->getVector3fMap() - poses[pt->source % poses.size()].translation()).norm();
    cluster_ranges[idx].push_back(range);
    cluster_min_range[idx] = MIN(range, cluster_min_range[idx]);
  }

  output_cloud->clear();
  removed_points->clear();
  for(int ci = 0; ci < clusters.size(); ci++) {
    const float min_range = cluster_min_range[ci];
    for(int pi = 0; pi < clusters[ci].size(); pi++) {
      const float range = cluster_ranges[ci][pi];
      const PointType &pt = clusters[ci][pi];
      if(range < relative_range_diff*min_range) {
        output_cloud->push_back(pt);
      } else {
        removed_points->push_back(pt);
      }
    }
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  string output_pcd_file;
  float sampling_ratio, range_threshold;
  float range_seg_leaf_size, range_seg_relative_diff;
  vector<bool> mask;
  vector<RoNI> ronis;

  if(!parse_arguments(argc, argv,
      sampling_ratio,
      poses, calibration, filenames,
      output_pcd_file,
      mask,
      range_threshold,
      range_seg_leaf_size, range_seg_relative_diff,
      ronis)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointType>::Ptr sum_cloud(new PointCloud<PointType>);
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
      RoNI::filter(*cloud, ronis, *cloud);
      filter_by_range(*cloud, *cloud, range_threshold);
      subsample_cloud<PointType>(cloud, sampling_ratio);
      transformPointCloud(*cloud, *cloud, poses[frame_i]);
      for(PointCloud<PointType>::iterator pt = cloud->begin(); pt < cloud->end(); pt++) {
        pt->source = pt->source*poses.size() + frame_i;
      }
      *sum_cloud += *cloud;
      cloud->clear();
    }
  }

  if(range_seg_leaf_size > 0.0) {
    PointCloud<PointType>::Ptr removed_points(new PointCloud<PointType>);
    range_segmentation(sum_cloud, poses, range_seg_leaf_size, range_seg_relative_diff, sum_cloud, removed_points);
    io::savePCDFileBinary(output_pcd_file + ".removed.pcd", *removed_points);
  }

  io::savePCDFileBinary(output_pcd_file, *sum_cloud);

  return EXIT_SUCCESS;
}
