/*
 * Overlap estimation of dense point clouds.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
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

#include <pcl/common/eigen.h>
#include <boost/program_options.hpp>
#include <cv.h>

#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/Overlap.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

typedef PointXYZ PointT;


class DenseCloudOverlap {

public:

    class Parameters {
    public:
        Parameters(
                bool visualization_ = false,
                float leaf_size_ = 0.05,
                float max_match_distance_ = 0.1) :
          visualization(visualization_),
          leaf_size(leaf_size_),
          max_match_distance(max_match_distance_) {
        }
        bool visualization;
        float leaf_size;
        float max_match_distance;

        void loadFrom(po::options_description &desc) {
          desc.add_options()
              ("visualization,v", po::bool_switch(&visualization),
                  "Show visualization.")
              ("leaf_size,l", po::value<float>(&leaf_size)->default_value(leaf_size),
                  "Kd-tree leaf size for downsampling.")
              ("max_match_distance,m", po::value<float>(&max_match_distance)->default_value(max_match_distance),
                  "Correspondence distance threshold.")
              ;
        }
    } param;

    DenseCloudOverlap(Parameters parameters_) :
        param(parameters_) {
    }

    float compute(PointCloud<PointT>::ConstPtr src_cloud_,
                  PointCloud<PointT>::ConstPtr trg_cloud_) const {
      PointCloud<PointT>::Ptr src_cloud(new PointCloud<PointT>);
      PointCloud<PointT>::Ptr trg_cloud(new PointCloud<PointT>);

      pcl::VoxelGrid<PointT> grid;
      grid.setLeafSize(param.leaf_size, param.leaf_size, param.leaf_size);
      grid.setInputCloud(src_cloud_);
      grid.filter(*src_cloud);
      grid.setInputCloud (trg_cloud_);
      grid.filter(*trg_cloud);

      KdTreeFLANN<PointT> tree;
      tree.setInputCloud(src_cloud);

      PointCloud<PointT> trg_overlap, trg_rest;
      for(PointCloud<PointT>::const_iterator pt = trg_cloud->begin(); pt < trg_cloud->end(); pt++) {
        vector<int> indices;
        vector<float> distances;
        tree.radiusSearch(*pt, param.max_match_distance, indices, distances);
        if(indices.empty()) {
          trg_rest.push_back(*pt);
        } else {
          trg_overlap.push_back(*pt);
        }
      }

      float overlap = trg_overlap.size() / float(trg_cloud->size());

      if(param.visualization) {
        cerr << "Overlap: " << overlap << " (src points: " << src_cloud->size() << ", trg_points: " << trg_cloud->size()
          << ", trg overlapping points: " << trg_overlap.size() << ", trg rest: " << trg_rest.size() << ")" << endl;
        io::savePCDFileBinary("src_cloud.pcd", *src_cloud);
        io::savePCDFileBinary("trg_overlap.pcd", trg_overlap);
        io::savePCDFileBinary("trg_rest.pcd", trg_rest);
        Visualizer3D::getCommonVisualizer()->keepOnlyClouds(0)
            .setColor(0, 0, 255).addPointCloud(*src_cloud)
            .setColor(0, 255, 0).addPointCloud(trg_overlap)
            .setColor(255, 0, 0).addPointCloud(trg_rest)
            .show();
      }

      return overlap;
    }
};

bool parse_arguments(int argc, char **argv,
                     vector<string> &clouds, vector<Eigen::Affine3f> &poses, size_t &frame_distance,
                     DenseCloudOverlap::Parameters &parameters) {
  string poses_file;

  po::options_description desc("Dense point cloud overlaps.\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("poses,p", po::value<string>(&poses_file)->required(), "Poses of the PCDs.")
      ("frame_distance,d", po::value<size_t>(&frame_distance)->default_value(frame_distance),
           "Acceptable frame distance (0 means all distances).")
      ;

  parameters.loadFrom(desc);

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds = po::collect_unrecognized(parsed.options, po::include_positional);

  try {
    po::notify(vm);
  }
  catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }

  poses = KittiUtils::load_kitti_poses(poses_file);

  if(poses.size() != clouds.size()) {
    cerr << "ERROR! Number of poses (" << poses.size() << ") is different from the number of point clouds ("
         << clouds.size() << ")!" << endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {

  vector<string> clouds_filenames;
  vector<Eigen::Affine3f> poses;
  size_t expected_frame_distance;
  DenseCloudOverlap::Parameters parameters;

  if (!parse_arguments(argc, argv, clouds_filenames, poses, expected_frame_distance, parameters)) {
    return EXIT_FAILURE;
  }

  DenseCloudOverlap overlap_estimator(parameters);
  for(int i = 0; i < poses.size(); i++) {
    for(int j = i+1; j < poses.size(); j++) {

      size_t frame_dist = get_frames_distance(i, j, poses.size(), true);
      if(expected_frame_distance == 0 || expected_frame_distance == frame_dist) {

        PointCloud<PointT>::Ptr src_cloud(new PointCloud<PointT>);
        io::loadPCDFile(clouds_filenames[i], *src_cloud);
        transformPointCloud(*src_cloud, *src_cloud, poses[i]);

        PointCloud<PointT>::Ptr trg_cloud(new PointCloud<PointT>);
        io::loadPCDFile(clouds_filenames[j], *trg_cloud);
        transformPointCloud(*trg_cloud, *trg_cloud, poses[j]);

        float overlap = harmonic_avg(
                overlap_estimator.compute(src_cloud, trg_cloud),
                overlap_estimator.compute(trg_cloud, src_cloud));
        cout << i << " " << j << " " << overlap << endl;
      }
    }
  }

  return EXIT_SUCCESS;
}
