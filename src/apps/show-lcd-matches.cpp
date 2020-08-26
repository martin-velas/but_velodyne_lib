/*
 * Visualization of matches between LineClouds.
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
#include <but_velodyne/VelodyneMultiFrameSequence.h>
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


bool parse_arguments(int argc, char **argv,
                     string &matches_filename,
                     vector<string> &line_clouds,
                     SensorsCalibration &calibration) {
  string calibration_filename;

  po::options_description desc("CLS matches visualization\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("matches,m", po::value<string>(&matches_filename)->required(), "Matches file.")
    ("calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Velodynes calibration file.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  line_clouds = po::collect_unrecognized(parsed.options, po::include_positional);

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

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  if(line_clouds.size() != 2 * calibration.sensorsCount()) {
    cerr << "ERROR, expecting " << 2 * calibration.sensorsCount() << " line clouds, found " <<
      line_clouds.size() << endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {

  string matches_filename;
  vector<string> clouds_fnames;
  SensorsCalibration calibration;
  if(!parse_arguments(argc, argv, matches_filename, clouds_fnames, calibration)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZ> src_cloud, trg_cloud;
  for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
    ifstream src_file(clouds_fnames[sensor_i]);
    ifstream trg_file(clouds_fnames[sensor_i + calibration.sensorsCount()]);

    LineCloud src_lines, trg_lines;
    src_file >> src_lines;
    src_lines.transform(calibration.ofSensor(sensor_i).matrix());
    trg_file >> trg_lines;
    trg_lines.transform(calibration.ofSensor(sensor_i).matrix());

    src_cloud += *src_lines.getMiddles();
    trg_cloud += *trg_lines.getMiddles();
  }



  vector<cv::DMatch> matches;
  ifstream matches_file(matches_filename.c_str());
  while(!matches_file.eof()) {
    cv::DMatch m;
    matches_file >> m.trainIdx >> m.queryIdx;
    if(m.trainIdx < src_cloud.size() && m.queryIdx < trg_cloud.size()) {
      matches.push_back(m);
    }
  }

  Visualizer3D vis_matches;

  vis_matches
    .addPointCloud(src_cloud)
    .addPointCloud(trg_cloud)
    .show();

  vis_matches
    .addMatches(matches, src_cloud, src_cloud)
    .show();

  return EXIT_SUCCESS;
}
