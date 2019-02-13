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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

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
                     vector<bool> &mask,
                     vector<double> &times,
                     bool &show_axis) {
  string pose_filename, skip_filename, times_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("skip_file,s", po::value<string>(&skip_filename)->default_value(""), "File with indidces to skip.")
      ("times_file,t", po::value<string>(&times_filename)->default_value(""), "File with timestamps for poses.")
      ("axis,a", po::value<bool>(&show_axis)->default_value(true), "Show frame axis.")
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

  mask.resize(poses.size(), true);
  ifstream skip_file(skip_filename.c_str());
  string line;
  while(getline(skip_file, line)) {
    mask[atoi(line.c_str())] = false;
  }

  load_vector_from_file(times_filename, times);

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<bool> mask;
  vector<double> times;
  vector<string> clouds_fnames;
  bool show_axis;
  if(!parse_arguments(argc, argv, poses, clouds_fnames, mask, times, show_axis)) {
    return EXIT_FAILURE;
  }

  Visualizer3D visualizer;
  VelodynePointCloud cloud;
  VelodynePointCloud sum_cloud;

  PointXYZ senzor(0,0,0);
  for(int i = 0; i < clouds_fnames.size(); i++) {
    if(mask[i]) {
      /*if(i%10 != 0) {
        continue;
      }*/
      VelodynePointCloud::fromFile(clouds_fnames[i], cloud, false);
      transformPointCloud(cloud, cloud, poses[0].inverse()*poses[i]);
      sum_cloud += cloud;
      visualizer.keepOnlyClouds(0);
      addVelodynePcl(visualizer, sum_cloud);
    }
  }

  vector<Eigen::Affine3f> poses_to_vis;
  vector<Eigen::Affine3f> poses_to_skip;
  for(int i = 0; i < poses.size(); i++) {
    if(mask[i]) {
      poses_to_vis.push_back(poses[i]);
    } else {
      poses_to_skip.push_back(poses[i]);
    }
  }

  if(!show_axis) {
    visualizer.getViewer()->removeAllCoordinateSystems();
  }

  visualizer
    .addPoses(poses_to_vis, 0.3)
    .setColor(200, 50, 50).addPosesDots(poses_to_vis)
    .setColor(10, 10, 255).addPosesDots(poses_to_skip);

  for(int i = 0; i < times.size(); i++) {
    stringstream stamp;
    stamp << times[i] - times[0];
    PointXYZ position;
    position.getVector3fMap() = poses[i].translation();
    position.z -= 0.2;
    //visualizer.getViewer()->addText3D(stamp.str(), position, 0.03, 200, 0, 200, visualizer.getId("text"));
  }

  visualizer.show();

  return EXIT_SUCCESS;
}
