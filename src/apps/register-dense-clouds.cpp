/*
 * Registration of dense point clouds.
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

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/DenseCloudRegistration.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

namespace po = boost::program_options;

typedef PointXYZ PointT;


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

  cerr << "Dense registration:" << endl << " -> " << source_cloud_fn << endl << " -> " << target_cloud_fn << endl;

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
