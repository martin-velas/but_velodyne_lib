/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 29/04/2019
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

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointXYZRGB PointType;

bool parse_arguments(int argc, char **argv,
    string &output_filename,
    vector<string> &clouds_to_process) {

  string input_cloud_fn, ref_cloud_fn, slices_fn;

  po::options_description desc("Evaluation of 4RECON backpack\n"
      "======================================\n"
      " * Reference(s): ? Velas et al, Sensors, 2019 ?\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("output,o", po::value<string>(&output_filename)->required(), "Screenshot output file.png")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_process.empty()) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  return true;
}

int main(int argc, char** argv) {

  string output_filename;
  vector<string> cloud_filenames;
  if(!parse_arguments(argc, argv, output_filename, cloud_filenames)) {
    return EXIT_FAILURE;
  }

  Visualizer3D vis;
  vis.setPointSize(2);

  vector<int> viewports(4);
  vis.getViewer()->createViewPort(0.0, 0.5, 0.5, 1.0, viewports[0]);
  vis.getViewer()->createViewPort(0.5, 0.5, 1.0, 1.0, viewports[1]);
  vis.getViewer()->createViewPort(0.0, 0.0, 0.5, 0.5, viewports[2]);
  vis.getViewer()->createViewPort(0.5, 0.0, 1.0, 0.5, viewports[3]);
  for(int i = 0; i < 4; i++) {
    vis.getViewer()->setBackgroundColor(1.0, 1.0, 1.0, viewports[i]);
  }

  for(int i = 0; i < cloud_filenames.size(); i++) {
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    io::loadPCDFile(cloud_filenames[i], *cloud);
    vis.addColorPointCloud(cloud, Eigen::Matrix4f::Identity(), viewports[i]);
  }
  vis.show();

  vis.getViewer()->saveScreenshot(output_filename);
  cerr << output_filename << " saved." << endl;

  return EXIT_SUCCESS;
}
