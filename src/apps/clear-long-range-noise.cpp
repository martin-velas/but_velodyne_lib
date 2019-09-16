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
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

struct PointXYZIRSRange
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t ring;                      ///< laser ring number
  uint16_t source;                    ///< source index (pose, sensor, etc.)
  float range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRSRange,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (uint16_t, ring, ring)
                                (uint16_t, source, source)
                                (float, range, range));

typedef PointXYZIRSRange PointType;

bool parse_arguments(int argc, char **argv,
                     PointCloud<PointType> &input, string &output_filename,
                     float &range_seg_leaf_size, float &range_seg_relative_diff) {
  vector<Eigen::Affine3f> poses;
  string pose_filename, input_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("input,i", po::value<string>(&input_filename)->required(), "Input cloud.")
      ("output,o", po::value<string>(&output_filename)->required(), "Output cloud.")
      ("leaf_size,s", po::value<float>(&range_seg_leaf_size)->default_value(1.0), "Leaf size for range segmentation (negative = disabled).")
      ("relative_diff,d", po::value<float>(&range_seg_relative_diff)->default_value(2.0), "Relative difference for range segmentation.")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);
  io::loadPCDFile(input_filename, input);
  for(PointCloud<PointType>::iterator p = input.begin(); p < input.end(); p++) {
    p->range = (p->getVector3fMap() - poses[p->source % poses.size()].translation()).norm();
  }

  return true;
}

Eigen::Vector3i getIndices3D(PointType pt, const PointType &offset, const float leaf_size) {
  pt.getVector3fMap() -= offset.getVector3fMap();
  pt.getVector3fMap() *= 1.0 / leaf_size;
  int x = floor(pt.x);
  int y = floor(pt.y);
  int z = floor(pt.z);
  return Eigen::Vector3i(x, y, z);
}

int getIndex(PointType pt, const PointType &offset, const float leaf_size,
    const Eigen::Vector3i &grid_size) {
  Eigen::Vector3i indices = getIndices3D(pt, offset, leaf_size);
  return indices.x() + grid_size.x()*( indices.y() + grid_size.y()*indices.z() );
}

void range_segmentation(PointCloud<PointType> input_cloud,
    const float leaf_size, const float relative_range_diff,
    PointCloud<PointType>::Ptr output_cloud) {

  cerr << "Computing voxels ... ";
  PointType min, max;
  getMinMax3D(input_cloud, min, max);

  Eigen::Vector3i max_indices = getIndices3D(max, min, leaf_size);
  Eigen::Vector3i grid_size = max_indices + Eigen::Vector3i(1, 1, 1);

  int max_index = getIndex(max, min, leaf_size, grid_size);

  vector< PointCloud<PointType> > voxels(max_index+1);
  vector<float> cluster_min_range(voxels.size(), INFINITY);
  for(PointCloud<PointType>::const_iterator pt = input_cloud.begin(); pt < input_cloud.end(); pt++) {
    int idx = getIndex(*pt, min, leaf_size, grid_size);
    voxels[idx].push_back(*pt);
    cluster_min_range[idx] = MIN(pt->range, cluster_min_range[idx]);
  }
  cerr << "[DONE]" << endl;

  //Visualizer3D vis;
  cerr << "Filtering ... ";
  for(int ci = 0; ci < voxels.size(); ci++) {
    if(!voxels[ci].empty()) {
      //vis.addPointCloud(voxels[ci]).show();
    }
    const float min_range = cluster_min_range[ci];
    for(int pi = 0; pi < voxels[ci].size(); pi++) {
      const PointType &pt = voxels[ci][pi];
      if(pt.range < relative_range_diff*min_range) {
        output_cloud->push_back(pt);
      }
    }
  }
  cerr << "[DONE]" << endl;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  PointCloud<PointType> input;
  string output_filename;
  float leaf_size, range_relative_diff;

  if(!parse_arguments(argc, argv, input, output_filename, leaf_size, range_relative_diff)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointType>::Ptr output(new PointCloud<PointType>);
  range_segmentation(input, leaf_size, range_relative_diff, output);
  io::savePCDFileBinary(output_filename, *output);

  return EXIT_SUCCESS;
}
