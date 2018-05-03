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
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &cloud_times_file, string &frame_borders_file, int &velodyne_idx,
                     string &output_dir,
                     vector<string> &clouds_to_process) {

  po::options_description desc("Synchronization of Velodynes in cloud meassurements\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("frame_borders,b", po::value<string>(&frame_borders_file)->required(), "File with expected timestamps last rows is time of termination.")
      ("cloud_times,t", po::value<string>(&cloud_times_file)->required(), "File with timestamps of clouds.")
      ("velodyne_idx,i", po::value<int>(&velodyne_idx)->default_value(-1), "Index of the Velodyne sensor.")
      ("output_dir,o", po::value<string>(&output_dir)->required(), "Output dir for PCD files.")
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

  return true;
}

bool get_slice(const double start, const double end,
    const vector<string> &cloud_filenames, const vector<double> &cloud_times,
    VelodynePointCloud &slice) {
  cerr << fixed << start << " " << end << " " << cloud_times.front() << " "  << cloud_times.back() << endl;
  if(start < cloud_times.front() || end > cloud_times.back()) {
    return false;
  }

  //static Visualizer3D vis;
  //vis.keepOnlyClouds(0);
  int i;
  for(i = 0; cloud_times[i+1] < start; i++)
    ;
  double expected_start_phase = 0.0;
  float min_phase = INFINITY;
  float max_phase = -INFINITY;
  for(; cloud_times[i] < end; i++) {
    double start2 = MAX(start, cloud_times[i]);
    double end2   = MIN(end, cloud_times[i+1]);
    double phase_span = cloud_times[i+1] - cloud_times[i];
    double start_phase = (start2 - cloud_times[i]) / phase_span;
    double end_phase = 1.0 - (cloud_times[i+1] - end2) / phase_span;

    cerr << fixed << start << " " << end << " " << start2 << " "  << end2 << " "  << start_phase << " " << end_phase << endl;

    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(cloud_filenames[i], cloud);
    VelodynePointCloud subslice;
    for(VelodynePointCloud::iterator p = cloud.begin(); p < cloud.end(); p++) {
      if(start_phase-0.0001 < p->phase && p->phase < end_phase+0.0001) {
        p->phase = p->phase - start_phase + expected_start_phase;
        min_phase = MIN(min_phase, p->phase);
        max_phase = MAX(max_phase, p->phase);
        subslice.push_back(*p);
      }
    }
    slice += subslice;
    expected_start_phase += end_phase - start_phase;
  }

  for(VelodynePointCloud::iterator p = slice.begin(); p < slice.end(); p++) {
    p->phase = (p->phase - min_phase) / (max_phase - min_phase + 0.0001);
  }

  //PointCloud<PointXYZRGB>::Ptr colored_by_phase = Visualizer3D::colorizeCloudByPhase(slice);
  //vis.addColorPointCloud(colored_by_phase).show();
  return true;
}

int main(int argc, char** argv) {

  vector<string> cloud_filenames;
  string cloud_times_filename, frame_borders_filename, output_dir;
  int velodyne_idx;

  if(!parse_arguments(argc, argv,
      cloud_times_filename, frame_borders_filename, velodyne_idx,
      output_dir,
      cloud_filenames)) {
    return EXIT_FAILURE;
  }

  vector<double> frame_borders, cloud_timestamps;
  load_vector_from_file(cloud_times_filename, cloud_timestamps);
  load_vector_from_file(frame_borders_filename, frame_borders);
  for(int i = 0; i+1 < frame_borders.size(); i++) {
    VelodynePointCloud slice;
    bool slice_found = get_slice(frame_borders[i], frame_borders[i+1], cloud_filenames, cloud_timestamps, slice);
    if(slice_found) {
      string fn = output_dir + "/" + KittiUtils::getKittiFrameName(i, ".pcd", velodyne_idx);
      io::savePCDFileBinary(fn, slice);
    }
  }

  return EXIT_SUCCESS;
}
