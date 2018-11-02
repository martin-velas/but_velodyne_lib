/*
 * Odometry estimation by collar line segments of Velodyne scan.
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
#include <cstdio>
#include <libgen.h>

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/SubseqRegistration.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

void read_lines(const string &fn, vector<string> &lines) {
  ifstream file(fn.c_str());
  string line;
  while (getline(file, line)) {
    lines.push_back(line);
  }
}

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    Eigen::Affine3f &init_transform,
    vector<Eigen::Affine3f> &src_poses, vector<string> &src_clouds_filenames,
    vector<Eigen::Affine3f> &trg_poses, vector<string> &trg_clouds_filenames,
    SensorsCalibration &calibration,
    bool &manual, bool &fail_when_not_converged) {
  string source_poses_file, target_poses_file;
  string source_clouds_list, target_clouds_list;
  string init_transform_filename, calibration_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
  ;

  registration_parameters.prepareForLoading(desc);

  pipeline_parameters.prepareForLoading(desc);

  desc.add_options()
    ("source_clouds_list", po::value<string>(&source_clouds_list)->required(),
      "List of source point clouds")
    ("target_clouds_list", po::value<string>(&target_clouds_list)->required(),
      "List of target point clouds")
    ("source_poses_file", po::value<string>(&source_poses_file)->required(),
      "File with source poses")
    ("target_poses_file", po::value<string>(&target_poses_file)->required(),
      "File with poses of target clouds for initialisation")
    ("init_transform,i", po::value<string>(&init_transform_filename)->default_value(""),
      "Transform for initialisation (pose file)")
    ("manual", po::bool_switch(&manual),
      "Enable manual verification and correction.")
    ("calibration,c", po::value<string>(&calibration_filename)->default_value(""),
        "Calibration file (sensors poses).")
    ("fail_when_not_converged", po::bool_switch(&fail_when_not_converged),
        "Fail when the registration did not converged.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }

  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }

  src_poses = KittiUtils::load_kitti_poses(source_poses_file);
  trg_poses = KittiUtils::load_kitti_poses(target_poses_file);
  read_lines(source_clouds_list, src_clouds_filenames);
  read_lines(target_clouds_list, trg_clouds_filenames);

  if(!init_transform_filename.empty()) {
    init_transform = KittiUtils::load_kitti_poses(init_transform_filename).front();
  } else {
    init_transform = Eigen::Affine3f::Identity();
  }

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  return true;
}

void buildLineCloud(const vector<string> &cloud_files,
    const vector<Eigen::Affine3f> &poses,
    const SensorsCalibration &calibration,
    int lines_per_cell_gen, int lines_per_cell_preserve,
    LineCloud &out_lines) {
  CollarLinesFilter filter(lines_per_cell_preserve);
  VelodyneFileSequence sequence(cloud_files, calibration);
  for(int i = 0; sequence.hasNext(); i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PolarGridOfClouds polar_grid(multiframe.clouds, multiframe.calibration);
    LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
    line_cloud.transform(poses[i].matrix());
    out_lines += line_cloud;
  }
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  Eigen::Affine3f init_transform;
  vector<Eigen::Affine3f> src_poses, trg_poses;
  vector<string> src_clouds_filenames, trg_clouds_filenames;
  SensorsCalibration calibration;
  bool manual, fail_when_not_converged;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters,
      init_transform,
      src_poses, src_clouds_filenames, trg_poses, trg_clouds_filenames,
      calibration, manual, fail_when_not_converged)) {
    return EXIT_FAILURE;
  }

  LineCloud src_lines, trg_lines;
  buildLineCloud(src_clouds_filenames, src_poses,
      calibration,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      src_lines);
  buildLineCloud(trg_clouds_filenames, trg_poses,
      calibration,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      trg_lines);

  Eigen::Affine3f t;
  Termination::Reason term_reason;
  if(manual) {
    ManualSubseqRegistration registration(src_lines, trg_lines,
        init_transform,
        pipeline_parameters,
        registration_parameters);
    t = registration.run();
    term_reason = registration.getTerminationReason();
  } else {
    SubseqRegistration registration(src_lines, trg_lines,
        init_transform,
        pipeline_parameters,
        registration_parameters);
    t = registration.run();
    term_reason = registration.getTerminationReason();
  }
  KittiUtils::printPose(std::cout, t.matrix());

  cerr << Termination::reasonToString(term_reason) << endl;
  if(fail_when_not_converged && term_reason != Termination::ERR_DEVIATION && term_reason != Termination::ERROR) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
