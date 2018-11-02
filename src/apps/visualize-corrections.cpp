/*
 * Visualization of corrections for global optimization.
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

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class PairRegistration {
public:
  PairRegistration(const int src_, const int trg_, const Eigen::Affine3f &t_) :
    src(src_), trg(trg_), t(t_) {
  }

  int src, trg;
  Eigen::Affine3f t;
};

std::ostream& operator<< (std::ostream &os, const PairRegistration &reg) {
  os << "[" << reg.src << " -> " << reg.trg << "]: ";
  KittiUtils::printPose(os, reg.t.matrix());
  return os;
}

bool parse_arguments(int argc, char **argv,
                     float &sampling_ratio,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     vector<PairRegistration> &corrections) {
  string pose_filename, sensor_poses_filename, corrections_poses_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sampling_ratio,s", po::value<float>(&sampling_ratio)->default_value(0.1), "Reduce size with this ratio.")
      ("sensor_poses", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
      ("corrections", po::value<string>(&corrections_poses_filename)->required(), "Corrections (srcidx trgidx pose)")
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

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  ifstream corrections_file(corrections_poses_filename.c_str());
  while(true) {
    int src_i, trg_i;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    corrections_file >> src_i >> trg_i >> pose;

    if(corrections_file.eof()) {
      break;
    } else {
      corrections.push_back(PairRegistration(src_i, trg_i, pose));
    }
  }

  return true;
}

typedef VelodynePoint PointType;

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  string output_pcd_file;
  float sampling_ratio;
  vector<PairRegistration> corrections;

  if(!parse_arguments(argc, argv,
      sampling_ratio,
      poses, calibration, filenames,
      corrections)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence file_sequence(filenames, calibration);
  PointCloud<PointType> sum_cloud;
  vector< PointCloud<PointType>::Ptr > clouds;
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    if(frame_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << frame_i << std::endl << std::flush;
      break;
    }
    clouds.push_back(PointCloud<PointType>::Ptr(new PointCloud<PointType>));
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.joinTo(*clouds.back());
    subsample_cloud<PointType>(clouds.back(), sampling_ratio);
    transformPointCloud(*clouds.back(), *clouds.back(), poses[frame_i]);
    sum_cloud += *clouds.back();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
  rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);
  subsample_cloud<pcl::PointXYZRGB>(rgb_cloud, sampling_ratio);

  Visualizer3D vis;
  for(vector<PairRegistration>::iterator pair = corrections.begin(); pair < corrections.end(); pair++) {
    cerr << *pair << endl;
    vis.keepOnlyClouds(0);
    vis.addColorPointCloud(rgb_cloud);
    vis.setColor(255, 0, 0);
    vis.addPointCloud(*(clouds[pair->src]));
    vis.setColor(0, 0, 255);
    vis.addPointCloud(*(clouds[pair->trg]), pair->t.matrix());
    vis.show();
  }

  return EXIT_SUCCESS;
}
