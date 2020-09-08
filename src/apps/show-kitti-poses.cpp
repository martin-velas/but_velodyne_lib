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

#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

void toColor(uchar i, uchar &r, uchar &g, uchar &b) {
  if(i < 128) {
    b = 2*i;
    r = g = 0;
  } else {
    b = 255;
    r = g = (i-128)*2;
  }
}

void addVelodynePcl(Visualizer3D &vis, const VelodynePointCloud &cloud) {
  PointCloud<PointXYZRGB>::Ptr rgb_cloud(new PointCloud<PointXYZRGB>());

  float min = cloud.getMinValuePt().intensity;
  float max = cloud.getMaxValuePt().intensity;

  for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    uchar r, g, b;
    float normalized = (pt->intensity - min) / (max - min) * 255.0;
    toColor(MIN(normalized*2, 255), r, g, b);
    PointXYZRGB rgb_pt;
    rgb_pt.x = pt->x;
    rgb_pt.y = pt->y;
    rgb_pt.z = pt->z;
    rgb_pt.r = r;
    rgb_pt.g = g;
    rgb_pt.b = b;
    rgb_cloud->push_back(rgb_pt);
  }

  vis.addColorPointCloud(rgb_cloud);
}

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     vector<string> &clouds_to_process,
                     SensorsCalibration &calibration,
                     bool &index_by_cloud_name,
                     bool &color_by_phase,
                     bool &show_indices, bool &single) {
  string pose_filename, calibration_filename;

  po::options_description desc("Poses and point clouds visualization\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Velodynes calibration file.")
    ("index_by_cloud_name,i", po::bool_switch(&index_by_cloud_name), "Get cloud index from filename.")
    ("color_by_phase", po::bool_switch(&color_by_phase), "Color clouds by rotor phase.")
    ("show_indices", po::bool_switch(&show_indices), "Show indices over poses.")
    ("single,s", po::bool_switch(&single), "Show each frame separately.")
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
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  return true;
}

void add_pose_index(const Eigen::Affine3f &pose, const int idx, Visualizer3D &visualizer) {
  stringstream ss;
  ss << idx;
  PointXYZ pt;
  pt.getVector3fMap() = pose.translation();
  pt.y -= visualizer.rngF() * 0.05 + 0.01;
  visualizer.getViewer()->addText3D(ss.str(), pt, 0.02, 0.8, 0.0, 0.8, "idx_" + ss.str());
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<string> clouds_fnames;
  SensorsCalibration calibration;
  bool index_by_cloud_name, color_by_phase, show_indices, single_view;
  if(!parse_arguments(argc, argv, poses, clouds_fnames, calibration,
      index_by_cloud_name, color_by_phase, show_indices, single_view)) {
    return EXIT_FAILURE;
  }

  Visualizer3D visualizer;

  visualizer.setColor(255, 0, 0);
  visualizer.setColor(0, 0, 255);
  visualizer.setColor(0, 200, 0);

  VelodyneFileSequence sequence(clouds_fnames, calibration);
  for (int frame_i = 0; sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    VelodynePointCloud cloud;
    multiframe.joinTo(cloud);
    int pose_i;
    if(index_by_cloud_name) {
      pose_i = KittiUtils::kittiNameToIndex(multiframe.filenames.front());
    } else {
      pose_i = frame_i;
    }
    if(color_by_phase) {
      visualizer.addColorPointCloud(Visualizer3D::colorizeCloudByPhase(cloud), poses[pose_i].matrix());
    } else {
      visualizer.addPointCloud(cloud, poses[pose_i].matrix());
    }
    if(single_view) {
      visualizer.keepOnlyClouds(1).show();
    }
  }

  if(show_indices) {
    for(int i = 0; i < poses.size(); i++) {
      add_pose_index(poses[i], i, visualizer);
    }
  }

  visualizer.addPoses(poses, 0.2).show();

  return EXIT_SUCCESS;
}
