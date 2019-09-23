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
#include <but_velodyne/VelodynePointCloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process) {
  string pose_filename, skip_filename, sensor_poses_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("calibration,c", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
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

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  if(clouds_to_process.size() != 2*calibration.sensorsCount()) {
    std::cerr << "ERROR: expecting 2 Velodyne frames, " <<
        (float(clouds_to_process.size())) / calibration.sensorsCount() << " given" << std::endl;
    return false;
  }

  return true;
}

typedef struct RGB_struct {
  RGB_struct(unsigned char _r, unsigned char _g, unsigned char _b) :
    r(_r), g(_g), b(_b) {
  }
  unsigned char r, g, b;
} RGB_t;

int main(int argc, char** argv) {

  vector<string> filenames;
  SensorsCalibration calibration;

  vector< vector<Eigen::Affine3f> > poses(2);

  if(!parse_arguments(argc, argv,
      poses[1], calibration, filenames)) {
    return EXIT_FAILURE;
  }

  const int slices_cnt = poses[1].size();
  poses[0] = vector<Eigen::Affine3f>(slices_cnt, Eigen::Affine3f::Identity());

  const float slice_fraction = 1.0/slices_cnt;
  cerr << "Slices: " << slices_cnt << ", fraction: " << slice_fraction << endl;
  vector<PhaseFilter> filters;
  filters.push_back(PhaseFilter(0.0, 0.2));
  filters.push_back(PhaseFilter(0.13, 0.53));
  filters.push_back(PhaseFilter(0.46, 0.86));

  vector< vector<RGB_t> > colors(2);
  vector<PointCloud<PointXYZRGB>::Ptr> slices;
  for(int i = 0; i < slices_cnt; i++) {
    colors[0].push_back(RGB_t((150/slices_cnt+1)*i+105, 100, 100));
    colors[1].push_back(RGB_t(100, 100, (150/slices_cnt+1)*i+105));
    slices.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>));
  }

  VelodyneFileSequence file_sequence(filenames, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    const VelodyneMultiFrame &frame = file_sequence.getNext();
    for(int slice_i = 0; slice_i < slices_cnt; slice_i++) {
      VelodynePointCloud cloud;
      frame.joinTo(cloud);
      filters[slice_i].filter(cloud);
      cerr << "[" << filters[slice_i].getMinPhase() << ", " << filters[slice_i].getMaxPhase() << "]" << endl;
      const RGB_t &color = colors[frame_i][slice_i];
      transformPointCloud(cloud, cloud, poses[frame_i][slice_i]);
      cerr << poses[frame_i][slice_i].matrix() << endl;
      *slices[slice_i] += *Visualizer3D::colorizeCloud(cloud, color.r, color.g, color.b);
    }
  }

  Visualizer3D vis;
  for(int slice_i = 0; slice_i < slices_cnt; slice_i++) {
    cerr << "Slice: " << slice_i << endl;
    vis.keepOnlyClouds(0);
    vis.addColorPointCloud(slices[slice_i]);
    vis.show();
  }

  return EXIT_SUCCESS;
}
