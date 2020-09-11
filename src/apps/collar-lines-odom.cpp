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

#include <pcl/common/eigen.h>
#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
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
                     boost::shared_ptr<MoveEstimator> &estimator,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     bool &visualize);

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  vector<string> clouds_to_process;
  boost::shared_ptr<MoveEstimator> estimator;
  SensorsCalibration calibration;
  bool visualize;

  if (!parse_arguments(argc, argv, registration_parameters, pipeline_parameters,
      estimator, calibration, clouds_to_process, visualize)) {
    return EXIT_FAILURE;
  }

  CollarLinesRegistrationPipeline registration(*estimator, pipeline_parameters, registration_parameters);

  int sensors = calibration.sensorsCount();
  VelodyneFileSequence sequence(clouds_to_process, calibration);

  Visualizer3D::Ptr vis;
  PointCloud<PointXYZ> src_vis_cloud, trg_vis_cloud;
  if(visualize) {
    vis = Visualizer3D::Ptr(new Visualizer3D);
  }

  for (int frame_i = 0; sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();

    if(visualize) {
      multiframe.joinTo(trg_vis_cloud);
      vis->keepOnlyClouds(0)
              .setColor(0, 0, 255).addPointCloud(src_vis_cloud)
              .setColor(255, 0, 0).addPointCloud(trg_vis_cloud)
              .show();
    }

    Eigen::Matrix4f t = registration.runRegistration(multiframe.clouds, calibration);
    registration.output(t);

    if(visualize) {
      vis->setColor(0, 255, 0).addPointCloud(trg_vis_cloud, t).show();
      src_vis_cloud = trg_vis_cloud;
    }
  }

  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     boost::shared_ptr<MoveEstimator> &estimator,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     bool &visualize) {
  bool use_kalman;
  int linear_estimator;
  string init_poses;
  string sensors_pose_file;

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
      ("linear_estimator", po::value<int>(&linear_estimator)->default_value(0),
          "Use last N frames for linear odometry prediction - can not be combined with kalman_estimator switch or init_poses_estimator")
      ("init_poses_estimator", po::value<string>(&init_poses)->default_value(""),
          "Use precomputed poses as a prediction - can not be combined with kalman_estimator switch or linear_estimator")
      ("kalman_estimator", po::bool_switch(&use_kalman),
          "Use Kalman filter instead of linear predictor or precomputed poses for estimation of odometry")
      ("sensors_pose_file", po::value<string>(&sensors_pose_file)->default_value(""),
          "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
      ("visualize", po::bool_switch(&visualize),
           "Run visualization before/after the registration.")
  ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1)
    {
        std::cerr << desc << std::endl;
        return false;
    }

    try
    {
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if(use_kalman?1:0 + (linear_estimator > 0)?1:0 + init_poses.empty()?0:1 > 1) {
        std::cerr << "Unable to use multiple predictors at the same time!" << std::endl;
        std::cerr << desc << std::endl;
        return false;
    } else if(use_kalman) {
      estimator.reset(new KalmanMoveEstimator(1e-5, 1e-4, 1.0));
    } else if(!init_poses.empty()) {
      estimator.reset(new PosesToInitEstimator(init_poses));
    } else {
      estimator.reset(new LinearMoveEstimator(linear_estimator > 0 ? linear_estimator : 3));
    }

    if(sensors_pose_file.empty()) {
      calibration = SensorsCalibration();
    } else {
      calibration = SensorsCalibration(sensors_pose_file);
    }

    return true;
}
