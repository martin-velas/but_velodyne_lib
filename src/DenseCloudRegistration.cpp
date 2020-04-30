/*
 * Registration of dense point clouds.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 29/04/2020
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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/DenseCloudRegistration.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

namespace po = boost::program_options;

namespace but_velodyne {

void DenseCloudRegistration::Parameters::loadFrom(po::options_description &desc) {
  desc.add_options()
          ("visualization,v", po::bool_switch(&visualization),
           "Show visualization.")
          ("leaf_size,l", po::value<float>(&leaf_size)->default_value(leaf_size),
           "Kd-tree leaf size for downsampling. If (size <= 0) then no resampling.")
          ("epsilon,e", po::value<float>(&epsilon)->default_value(epsilon),
           "Termination parameter - transformation change threshold.")
          ("max_iterations,i", po::value<size_t>(&max_iterations)->default_value(max_iterations),
           "Termination parameter - maximum number of iterations.")
          ("max_match_distance,d", po::value<float>(&max_match_distance)->default_value(max_match_distance),
           "Correspondence distance threshold.")
          ("neighbours_for_normal,n",
           po::value<size_t>(&neighbours_for_normal)->default_value(neighbours_for_normal),
           "How many neighbours are taken in account when the normal is estimated.");
}

DenseCloudRegistration::DenseCloudRegistration(PointCloud<PointT>::ConstPtr src_cloud_,
                       PointCloud<PointT>::ConstPtr trg_cloud_, Parameters parameters_) :
    src_cloud(new PointCloud <PointNormalT>),
    trg_cloud(new PointCloud <PointNormalT>),
    param(parameters_) {

  PointCloud<PointT>::Ptr src_sampled(new PointCloud <PointT>);
  PointCloud<PointT>::Ptr trg_sampled(new PointCloud <PointT>);

  if(param.leaf_size > 0.0) {
    subsample_by_voxel_grid(src_cloud_, *src_sampled, param.leaf_size);
    subsample_by_voxel_grid(trg_cloud_, *trg_sampled, param.leaf_size);
  } else {
    *src_sampled += *src_cloud_;
    *trg_sampled += *trg_cloud_;
  }

  pcl::NormalEstimation <PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <PointT>);
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(param.neighbours_for_normal);

  norm_est.setInputCloud(src_sampled);
  norm_est.compute(*src_cloud);
  pcl::copyPointCloud(*src_sampled, *src_cloud);

  norm_est.setInputCloud(trg_sampled);
  norm_est.compute(*trg_cloud);
  pcl::copyPointCloud(*trg_sampled, *trg_cloud);
}

Eigen::Affine3f DenseCloudRegistration::run(void) {

  // pcl::IterativeClosestPoint<PointT, PointT> reg;
  pcl::IterativeClosestPointWithNormals <PointNormalT, PointNormalT> reg;

  reg.setTransformationEpsilon(param.epsilon);
  reg.setMaxCorrespondenceDistance(param.max_match_distance);
  reg.setMaximumIterations(param.max_iterations);

  reg.setInputSource(src_cloud);
  reg.setInputTarget(trg_cloud);

  if (param.visualization) {
    Visualizer3D::getCommonVisualizer()->keepOnlyClouds(0)
            .setColor(255, 0, 0).addPointCloud(*src_cloud)
            .setColor(0, 0, 255).addPointCloud(*trg_cloud)
            .show();
  }

  // Estimate
  PointCloud <PointNormalT> src_cloud_aligned;
  reg.align(src_cloud_aligned);

  if (param.visualization) {
    Visualizer3D::getCommonVisualizer()
            ->setColor(0, 255, 0).addPointCloud(src_cloud_aligned)
            .show();
  }

  // Get the transformation from target to source
  return Eigen::Affine3f(reg.getFinalTransformation().inverse());
}

}
