/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 08/03/2019
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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <Eigen/Geometry>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    SensorsCalibration &calibration,
    vector<string> &clouds_to_process,
    float &fov_angle,
    string &out_dir,
    bool &visualize) {

  string sensors_pose_file;

  po::options_description desc(
      "Extraction of the overlap within the multiframe\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.")
    ("fov_angle,a", po::value<float>(&fov_angle)->required(), "Horizontal FOV [deg].")
    ("calibration,c", po::value<string>(&sensors_pose_file)->required(), "Sensors poses (calibration).")
    ("visualize,v", po::bool_switch(&visualize), "Show visualization.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }
  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  calibration = SensorsCalibration(sensors_pose_file);

  return true;
}

void cut(const vector<Eigen::Vector3f> &slicer_normals,
        const Eigen::Affine3f &slicer_pose,
        const Eigen::Affine3f &victim_pose,
        const VelodynePointCloud &victim_cloud,
        VelodynePointCloud &output) {
  assert(slicer_normals.size() == 2);

  Eigen::Vector3f N1 = slicer_pose.rotation() * slicer_normals[0];
  Eigen::Vector3f N2 = slicer_pose.rotation() * slicer_normals[1];

  Eigen::Vector3f slicer_position = slicer_pose.translation();
  Eigen::Hyperplane<float, 3> P1(N1, slicer_position);
  Eigen::Hyperplane<float, 3> P2(N2, slicer_position);

  VelodynePointCloud victim_cloud_transformed;
  transformPointCloud(victim_cloud, victim_cloud_transformed, victim_pose);
  ;
  for(VelodynePointCloud::const_iterator p = victim_cloud_transformed.begin(); p < victim_cloud_transformed.end(); p++) {
    const float d1 = P1.signedDistance(p->getVector3fMap());
    const float d2 = P2.signedDistance(p->getVector3fMap());
    if(d1 > 0 && d2 > 0) {
      output.push_back(*p);
    }
  }
  transformPointCloud(output, output, victim_pose.inverse());
}

void get_slicing_normals(const float fov_angle,
        vector<Eigen::Vector3f> &front_normals0, vector<Eigen::Vector3f> &back_normals0) {
  Eigen::Vector3f y_p(0.0, +1.0, 0.0);
  Eigen::Vector3f y_m(0.0, -1.0, 0.0);
  Eigen::Affine3f Rz_p = getTransformation(0.0, 0.0, 0.0, 0.0, 0.0, +degToRad(fov_angle/2.0));
  Eigen::Affine3f Rz_m = getTransformation(0.0, 0.0, 0.0, 0.0, 0.0, -degToRad(fov_angle/2.0));

  front_normals0.push_back(Rz_p * y_m);
  front_normals0.push_back(Rz_m * y_p);
  back_normals0.push_back(Rz_p * y_p);
  back_normals0.push_back(Rz_m * y_m);
}

int main(int argc, char** argv) {

  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  float fov_angle;
  string out_dir;
  bool visualize;
  if(!parse_arguments(argc, argv, calibration, clouds_to_process, fov_angle, out_dir, visualize)) {
    return EXIT_FAILURE;
  }

  vector<Eigen::Vector3f> front_normals0, back_normals0;
  get_slicing_normals(fov_angle, front_normals0, back_normals0);
  const vector<Eigen::Vector3f> &front_normals1 = back_normals0;
  const vector<Eigen::Vector3f> &back_normals1 = front_normals0;

  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  while(file_sequence.hasNext()) {
    VelodyneMultiFrame multiframe = file_sequence.getNext();

    VelodynePointCloud front0, back0, front1, back1;
    cut(front_normals1, calibration.ofSensor(1), calibration.ofSensor(0), *multiframe.clouds[0], front0);
    cut(back_normals1, calibration.ofSensor(1), calibration.ofSensor(0), *multiframe.clouds[0], back0);
    cut(front_normals0, calibration.ofSensor(0), calibration.ofSensor(1), *multiframe.clouds[1], front1);
    cut(back_normals0, calibration.ofSensor(0), calibration.ofSensor(1), *multiframe.clouds[1], back1);

    if (visualize) {
      static Visualizer3D vis;
      vis.keepOnlyClouds(0)
              .addPointCloud(front0, calibration.ofSensor(0).matrix())
              .addPointCloud(front1, calibration.ofSensor(1).matrix())
              .addPointCloud(back0, calibration.ofSensor(0).matrix())
              .addPointCloud(back1, calibration.ofSensor(1).matrix())
              .show();
    }

    size_t kitti_i = KittiUtils::kittiNameToIndex(multiframe.filenames.front());

    io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(kitti_i, ".front.pcd", 1), front0);
    io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(kitti_i, ".back.pcd", 1), back0);
    io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(kitti_i, ".front.pcd", 2), front1);
    io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(kitti_i, ".back.pcd", 2), back1);
  }

  return EXIT_SUCCESS;
}
