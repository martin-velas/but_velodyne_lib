/*
 * Transformation of the PCD to the LineCloud.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
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
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


bool parse_arguments(int argc, char **argv,
                     string &pcd_filename, string &output_filename,
                     int &generated_lines, int &preserved_lines,
                     int &polar_bins, int &polar_bins_subdivision,
                     bool &visualize) {

  po::options_description desc("PCD to the LineCloud transformation\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("input_pcd,i", po::value<string>(&pcd_filename)->required(), "Input PCD file")
      ("output_line_cloud,o", po::value<string>(&output_filename)->required(), "Output LineCloud file")
      ("lines_per_cell_generated,g", po::value<int>(&generated_lines)->default_value(20), "Lines per cell generated")
      ("lines_per_cell_preserved,p", po::value<int>(&preserved_lines)->default_value(5), "Lines per cell preserved")
      ("polar_bins", po::value<int>(&polar_bins)->default_value(36), "Polar bins count")
      ("polar_bins_subdivision", po::value<int>(&polar_bins_subdivision)->default_value(1), "Polar bins subdivision")
      ("visualize,v", po::bool_switch(&visualize), "Run visualization")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

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

  return true;
}

void pcd_filename_to_frame_sensor_id(const string &filename, int &frame_id, int &sensor_id) {
  boost::filesystem::path cloud_fn(filename);
  string fn = cloud_fn.filename().string();
  string delim = ".";

  size_t pos = 0;
  std::string token;
  int idx = 0;
  while ((pos = fn.find(delim)) != std::string::npos) {
    token = fn.substr(0, pos);

    switch(idx) {
      case 0:
        frame_id = atoi(token.c_str());
        break;
      case 1:
        sensor_id = atoi(token.c_str());
        break;
      default:
        frame_id = -1;
        sensor_id = 0;
        break;
    }

    fn.erase(0, pos + delim.length());
    idx++;
  }
}

int main(int argc, char** argv) {

  string input_pcd_filename, output_line_cloud_filename;
  int lines_generated, lines_preserved, polar_bins, polar_bin_subdivision;
  bool visualize;

  if (!parse_arguments(argc, argv, input_pcd_filename, output_line_cloud_filename,
                       lines_generated, lines_preserved, polar_bins, polar_bin_subdivision,
                       visualize)) {
    return EXIT_FAILURE;
  }

  VelodynePointCloud point_cloud;
  VelodynePointCloud::fromFile(input_pcd_filename, point_cloud);

  PolarGridOfClouds grid(point_cloud, polar_bins, polar_bin_subdivision);

  CollarLinesFilter registration_filter(lines_preserved);
  LineCloud line_cloud(grid, lines_generated, registration_filter);

  // default values:
  int frame_id = -1;
  int sensor_id = 0;
  pcd_filename_to_frame_sensor_id(input_pcd_filename, frame_id, sensor_id);
  for(LineCloud::iterator l = line_cloud.begin(); l < line_cloud.end();) {
    if(isnan(l->phase)) {
      l = line_cloud.erase(l);
    } else {
      l->frame_id = frame_id;
      l->sensor_id = sensor_id - 1;
      l++;
    }
  }

  ofstream out_line_cloud_file(output_line_cloud_filename.c_str());
  out_line_cloud_file << line_cloud;
  out_line_cloud_file.close();

  cerr << "Generated " << line_cloud.size() << " lines of the frame " << frame_id << " and the sensor " <<
    sensor_id << " into the file: " << output_line_cloud_filename << endl;

  if(visualize) {
    Visualizer3D vis;
    vis.addPointCloud(point_cloud)
       .addLines(line_cloud)
       .show();
  }

  return EXIT_SUCCESS;
}
