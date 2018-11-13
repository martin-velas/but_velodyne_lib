/*
 * Registration of the Velodyne point clouds by Collar Line Segments.
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

#include <pcl/common/eigen.h>
#include <boost/program_options.hpp>
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     SensorsCalibration &calibration, Eigen::Affine3f &init_transform,
                     PolarGridOfClouds::Ptr &src_grid, PolarGridOfClouds::Ptr &trg_grid) {
  string sensors_pose_file, init_poses_file;

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
    ("sensors_pose_file", po::value<string>(&sensors_pose_file)->default_value(""),
        "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
    ("init_poses", po::value<string>(&init_poses_file)->default_value(""),
        "Initial poses of the frames.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_to_process;
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  try {
      po::notify(vm);
  }
  catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  vector<Eigen::Affine3f> init_poses = KittiUtils::load_kitti_poses(init_poses_file);
  if(init_poses.size() != 2) {
    cerr << "ERROR, expected exactly two initial poses." << endl;
    return false;
  }
  init_transform = init_poses[0].inverse() * init_poses[1];

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }

  if (clouds_to_process.size() != 2*calibration.sensorsCount()) {
    cerr << "ERROR, expected " << 2*calibration.sensorsCount() << " point clouds" << endl;
    return false;
  }
  vector<string> src_clouds(clouds_to_process.begin(), clouds_to_process.begin()+calibration.sensorsCount());
  VelodyneMultiFrame src_frame(src_clouds, calibration);
  src_grid.reset(new PolarGridOfClouds(src_frame.clouds, calibration));
  vector<string> trg_clouds(clouds_to_process.begin()+calibration.sensorsCount(), clouds_to_process.end());
  VelodyneMultiFrame trg_frame(trg_clouds, calibration);
  trg_grid.reset(new PolarGridOfClouds(trg_frame.clouds, calibration));

  return true;
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  SensorsCalibration calibration;
  Eigen::Affine3f init_transform;
  PolarGridOfClouds::Ptr src_grid, trg_grid;

  if (!parse_arguments(argc, argv, registration_parameters, pipeline_parameters,
      calibration, init_transform, src_grid, trg_grid)) {
    return EXIT_FAILURE;
  }

  LinearMoveEstimator null_estimator(0);
  ofstream null_file("/dev/null");
  CollarLinesRegistrationPipeline registration(null_estimator, null_file,
      pipeline_parameters, registration_parameters);

  RegistrationOutcome result;
  registration.registerTwoGrids(*src_grid, *trg_grid, init_transform.matrix(), result);
  cout << result << endl;

  return EXIT_SUCCESS;
}
