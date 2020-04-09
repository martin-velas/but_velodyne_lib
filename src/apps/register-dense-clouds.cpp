/*
 * Registration of dense point clouds.
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

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

typedef PointXYZ PointT;
typedef PointNormal PointNormalT;


class DenseCloudRegistration {

private:
    PointCloud<PointNormalT>::Ptr src_cloud;
    PointCloud<PointNormalT>::Ptr trg_cloud;

public:

    class Parameters {
    public:
        Parameters(
                bool visualization_ = false,
                float leaf_size_ = 0.05,
                float epsilon_ = 1e-6,
                float max_match_distance_ = 0.1,
                size_t max_iterations_ = 100,
                size_t neighbours_for_normal_ = 30) :
          visualization(visualization_),
          leaf_size(leaf_size_),
          epsilon(epsilon_),
          max_match_distance(max_match_distance_),
          max_iterations(max_iterations_),
          neighbours_for_normal(neighbours_for_normal_) {
        }
        bool visualization;
        float leaf_size;
        float epsilon;
        float max_match_distance;
        size_t max_iterations;
        size_t neighbours_for_normal;

        void loadFrom(po::options_description &desc) {
          desc.add_options()
              ("visualization,v", po::bool_switch(&visualization),
                    "Show visualization.")
              ("leaf_size,l", po::value<float>(&leaf_size)->default_value(leaf_size),
                    "Kd-tree leaf size for downsampling.")
              ("epsilon,e", po::value<float>(&epsilon)->default_value(epsilon),
                   "Termination parameter - transformation change threshold.")
              ("max_iterations,i", po::value<size_t>(&max_iterations)->default_value(max_iterations),
                   "Termination parameter - maximum number of iterations.")
              ("max_match_distance,d", po::value<float>(&max_match_distance)->default_value(max_match_distance),
                   "Correspondence distance threshold.")
              ("neighbours_for_normal,n", po::value<size_t>(&neighbours_for_normal)->default_value(neighbours_for_normal),
                   "How many neighbours are taken in account when the normal is estimated.")
              ;
        }
    } param;

    DenseCloudRegistration(PointCloud<PointT>::ConstPtr src_cloud_,
            PointCloud<PointT>::ConstPtr trg_cloud_, Parameters parameters_) :
        src_cloud(new PointCloud<PointNormalT>),
        trg_cloud(new PointCloud<PointNormalT>),
        param(parameters_) {

      pcl::VoxelGrid<PointT> grid;
      grid.setLeafSize(param.leaf_size, param.leaf_size, param.leaf_size);
      grid.setInputCloud(src_cloud_);
      PointCloud<PointT>::Ptr src_sampled(new PointCloud<PointT>);
      grid.filter(*src_sampled);
      grid.setInputCloud (trg_cloud_);
      PointCloud<PointT>::Ptr trg_sampled(new PointCloud<PointT>);
      grid.filter(*trg_sampled);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<PointT>);
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (param.neighbours_for_normal);

      norm_est.setInputCloud (src_sampled);
      norm_est.compute(*src_cloud);
      pcl::copyPointCloud(*src_sampled, *src_cloud);

      norm_est.setInputCloud(trg_sampled);
      norm_est.compute (*trg_cloud);
      pcl::copyPointCloud (*trg_sampled, *trg_cloud);
    }

    Eigen::Affine3f run(void) {

      // pcl::IterativeClosestPoint<PointT, PointT> reg;
      pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;

      reg.setTransformationEpsilon(param.epsilon);
      reg.setMaxCorrespondenceDistance(param.max_match_distance);
      reg.setMaximumIterations(param.max_iterations);

      reg.setInputSource(src_cloud);
      reg.setInputTarget(trg_cloud);

      if(param.visualization) {
        Visualizer3D::getCommonVisualizer()->keepOnlyClouds(0)
                .setColor(255, 0, 0).addPointCloud(*src_cloud)
                .setColor(0, 0, 255).addPointCloud(*trg_cloud)
                .show();
      }

      // Estimate
      PointCloud<PointNormalT> src_cloud_aligned;
      reg.align(src_cloud_aligned);

      if(param.visualization) {
        Visualizer3D::getCommonVisualizer()
                ->setColor(0, 255, 0).addPointCloud(src_cloud_aligned)
                .show();
      }

      // Get the transformation from target to source
      return Eigen::Affine3f(reg.getFinalTransformation().inverse());
    }
};

bool parse_arguments(int argc, char **argv,
                     string &source_pcd_fn, string &target_pcd_fn, Eigen::Affine3f &init_transformation,
                     DenseCloudRegistration::Parameters &parameters) {
  string init_poses_file;

  po::options_description desc("Dense point cloud registration.\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("source,s", po::value<string>(&source_pcd_fn)->required(), "Source pcd file.")
      ("target,t", po::value<string>(&target_pcd_fn)->required(), "Target pcd file.")
      ("init_poses,p", po::value<string>(&init_poses_file)->default_value(""), "Initial poses (2) of the PCDs.")
      ;

  parameters.loadFrom(desc);

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_to_process;

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

  if(init_poses_file.empty()) {
    init_transformation = Eigen::Affine3f::Identity();
  } else {
    vector<Eigen::Affine3f> init_poses = KittiUtils::load_kitti_poses(init_poses_file);
    if(init_poses.size() != 2) {
      cerr << "ERROR, expected exactly two initial poses." << endl;
      return false;
    } else {
      init_transformation = init_poses[0].inverse() * init_poses[1];
    }
  }

  return true;
}

int main(int argc, char** argv) {

  string source_cloud_fn, target_cloud_fn;
  Eigen::Affine3f init_transform;
  DenseCloudRegistration::Parameters parameters;

  if (!parse_arguments(argc, argv, source_cloud_fn, target_cloud_fn,
                       init_transform, parameters)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointT>::Ptr src_cloud(new PointCloud<PointT>);
  io::loadPCDFile(source_cloud_fn, *src_cloud);
  PointCloud<PointT>::Ptr trg_cloud(new PointCloud<PointT>);
  io::loadPCDFile(target_cloud_fn, *trg_cloud);
  transformPointCloud(*trg_cloud, *trg_cloud, init_transform);

  DenseCloudRegistration registration(src_cloud, trg_cloud, parameters);
  Eigen::Affine3f t = registration.run();

  cout << t*init_transform << endl;

  return EXIT_SUCCESS;
}
