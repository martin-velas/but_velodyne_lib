/*
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

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/PhaseFilter.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

namespace po = boost::program_options;


bool parse_arguments(int argc, char **argv,
                     float &phase_shift,
                     vector<string> &clouds_to_process,
                     string &output_dir,
                     string &suffix) {
  po::options_description desc("Rephasing of the Velodyne point clouds.\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("phase_shift,p", po::value<float>(&phase_shift)->required(), "Phase shift.")
      ("output_dir,o", po::value<string>(&output_dir)->required(), "Output directory.")
      ("suffix,s", po::value<string>(&suffix)->required(), "Output files suffix (e.g. \".N.pcd\").")
      ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  try {
    po::notify(vm);
  }
  catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }
  if (vm.count("help") || clouds_to_process.size() < 2) {
    std::cerr << desc << std::endl;
    return false;
  }

  return true;
}

void shift_phase(VelodynePointCloud &cloud, const float phase_shift) {
  for(VelodynePointCloud::iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    pt->phase += phase_shift;
  }
}

void rephase(const VelodynePointCloud &this_cloud, const VelodynePointCloud &next_cloud, const float phase_shift,
             VelodynePointCloud &output_cloud) {
  PhaseFilter this_filter(phase_shift, 1.0);
  PhaseFilter next_filter(0.0, phase_shift);

  VelodynePointCloud slice;
  this_filter.filter(this_cloud, slice);
  shift_phase(slice, -phase_shift);
  output_cloud += slice;

  slice.clear();
  next_filter.filter(next_cloud, slice);
  shift_phase(slice, 1.0-phase_shift);
  output_cloud += slice;
}

int main(int argc, char** argv) {

  float phase_shift;
  vector<string> clouds_to_process;
  string output_dir;
  string suffix;

  if (!parse_arguments(argc, argv, phase_shift, clouds_to_process,
                       output_dir, suffix)) {
    return EXIT_FAILURE;
  }

  VelodynePointCloud this_cloud;
  VelodynePointCloud::fromFile(clouds_to_process[0], this_cloud);

  for(int i = 0; i+1 < clouds_to_process.size(); i++) {
    VelodynePointCloud next_cloud;
    VelodynePointCloud::fromFile(clouds_to_process[i+1], next_cloud);

    VelodynePointCloud output_cloud;
    rephase(this_cloud, next_cloud, phase_shift, output_cloud);

    string output_fn = output_dir + "/" + KittiUtils::getKittiFrameName(i, suffix);
    io::savePCDFileBinary(output_fn, output_cloud);

    this_cloud = next_cloud;
  }

  return EXIT_SUCCESS;
}
