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

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     Eigen::Affine3f &halign,
                     vector<string> &clouds_to_process) {
  string pose_filename, sensor_poses_filename, halign_pose_filename;

  po::options_description desc("Showroom demo of mapping process\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("horizontal_align", po::value<string>(&halign_pose_filename)->required(), "Horizontal alignment pose.")
      ("sensor_poses", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
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
  halign = KittiUtils::load_kitti_poses(halign_pose_filename).front();

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

void getSlice(PointCloud<PointXYZRGB>::Ptr sum_cloud, PointCloud<PointXYZRGB>::Ptr slice,
    float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
  slice->clear();
  for(PointCloud<PointXYZRGB>::iterator p = sum_cloud->begin(); p < sum_cloud->end(); p++) {
    if(p->x > min_x && p->x < max_x && p->y > min_y && p->y < max_y && p->z > min_z && p->z < max_z) {
      slice->push_back(*p);
    }
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  Eigen::Affine3f halign;

  if(!parse_arguments(argc, argv,
      poses, calibration, halign, filenames)) {
    return EXIT_FAILURE;
  }

  halign.translation() = Eigen::Vector3f::Identity();

  PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
  PointCloud<PointXYZRGB>::Ptr rgb_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr sum_cloud(new PointCloud<PointXYZRGB>);
  VelodyneFileSequence file_sequence(filenames, calibration);
  Visualizer3D vis;
  vis.getViewer()->setBackgroundColor(0, 0, 0);
  vis.getViewer()->removeCoordinateSystem();
  vis.getViewer()->removeAllShapes();
  vis.setPointSize(2);
  float min_intensity, max_intensity;
  stringstream fn;
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.joinTo(*cloud);
    transformPointCloud(*cloud, *cloud, halign*poses[frame_i]);
    subsample_cloud<PointXYZI>(cloud, 0.5);

    if(frame_i == 0) {
      Visualizer3D::getMinMaxIntensities(*cloud, min_intensity, max_intensity, 0.001);
    }
    Visualizer3D::normalizeMinMaxIntensity(*cloud, *cloud, min_intensity, max_intensity, 0, 1);
    rgb_cloud = Visualizer3D::colorizeCloud(*cloud, true);
    vis.keepOnlyClouds(0).addColorPointCloud(rgb_cloud);
    fn.str("");
    fn << "frame_" << frame_i << ".png";
    vis.getViewer()->addCoordinateSystem(0.1, halign*poses[frame_i], fn.str());
    if(frame_i == 0) {
      vis.show();
    }
    vis.saveSnapshot(fn.str());

    *sum_cloud += *rgb_cloud;
  }

  vis.getViewer()->removeAllCoordinateSystems();

  vis.setPointSize(1);
  vis.keepOnlyClouds(0).addColorPointCloud(sum_cloud).saveSnapshot("sum_cloud.png");

  PointCloud<PointXYZRGB>::Ptr slice(new PointCloud<PointXYZRGB>);

  float TOP = 2;

  for(int i = 0; TOP-i*0.05 > -3; i++) {
    float min_y = TOP-i*0.05;
    float max_y = min_y + 0.1;
    cerr << "slice: " << min_y << " - " << max_y << endl;
    getSlice(sum_cloud, slice,
        -20, 20,
        min_y, max_y,
        -20, 20);
    fn.str("");
    fn << "slice_" << i << ".png";
    vis.keepOnlyClouds(0).addColorPointCloud(slice).saveSnapshot(fn.str());
  }

  getSlice(sum_cloud, slice,
      -20, 20,
      -20, 1.3,
      -20, 20);
  vis.keepOnlyClouds(0).addColorPointCloud(slice).saveSnapshot("notop.png");

  PointXYZ centroid;
  computeCentroid(*slice, centroid);
  Eigen::Affine3f to_center = getTransformation(-centroid.x, -centroid.y, -centroid.z, 0, 0, 0);
  PointCloud<PointXYZRGB>::Ptr slice_rot(new PointCloud<PointXYZRGB>);
  for(float a = 0.0; a < 360.0; a += 4.0) {
    Eigen::Affine3f t = to_center.inverse() * getTransformation(0, 0, 0, 0, a/180*3.14159, 0) * to_center;
    transformPointCloud(*slice, *slice_rot, t);
    fn.str("");
    fn << "defile_" << a << ".png";
    vis.keepOnlyClouds(0).addColorPointCloud(slice_rot).saveSnapshot(fn.str());
    slice_rot->clear();
  }

  return EXIT_SUCCESS;
}
