/*
 * Overlaps estimation among point clouds for CLS registration.
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
#include <but_velodyne/Overlap.h>
#include <but_velodyne/common.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     float &overlap_thresholds, float &max_reg_error,
                     float &depth_quantile,
                     float &depth_relative_tolerance, float &depth_absolute_tolerance,
                     bool &average_with_zbuffer_occupancy,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     bool &visualize) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("poses", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("overlap_thresholds,t", po::value<float>(&overlap_thresholds)->required(), "Overlap threshold")
    ("max_reg_error", po::value<float>(&max_reg_error)->required(), "Maximal inverse registration error.")
    ("depth_quantile", po::value<float>(&depth_quantile)->default_value(0.0),
        "Quantile of depth within the bin (0.5 for median, 0.0 for minimum, 1.0 for maximum.")
    ("depth_relative_tolerance", po::value<float>(&depth_relative_tolerance)->default_value(0.1),
        "Relative tolerance of the depth in Z-buffer.")
    ("depth_absolute_tolerance", po::value<float>(&depth_absolute_tolerance)->default_value(0.3),
        "Absolute tolerance of the depth in Z-buffer.")
    ("visualize", po::bool_switch(&visualize), "Run visualization.")
    ("average_with_zbuffer_occupancy", po::bool_switch(&average_with_zbuffer_occupancy),
        "Average with nuber of occupied cells in the Z-buffer.")
  ;
  registration_parameters.prepareForLoading(desc);
  pipeline_parameters.prepareForLoading(desc);
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

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensor_poses_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensor_poses_filename);
  }

  return true;
}

float computeOverlap(SphericalZbuffer &src_zbuffer, const PointCloud<VelodynePoint> &src_cloud,
    const PointCloud<VelodynePoint> &trg_cloud,
    const float depth_relative_tolerance, const float depth_absolute_tolerance,
    const bool average_with_zbuffer_occupancy, const bool visualize) {
  Visualizer3D::Ptr vis;

  size_t points_within;
  if(visualize) {
    PointCloud<VelodynePoint> within, rest;
    points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance,
                                               within, rest);
    vis = Visualizer3D::getCommonVisualizer();
    vis->keepOnlyClouds(0)
        .setColor(0, 0, 0).addPointCloud(src_cloud)
        .setColor(0, 200, 0).addPointCloud(within)
        .setColor(255, 0, 0).addPointCloud(rest)
        .setColor(255, 0, 255);
    src_zbuffer.addToVisualizer(*vis, src_cloud, depth_relative_tolerance, depth_absolute_tolerance);
  } else {
    points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance);
  }

  float overlap;
  float points_within_portion = (float (points_within)) / trg_cloud.size();
  if(average_with_zbuffer_occupancy) {
    float visited_portion = src_zbuffer.visitedPortion();
    overlap = harmonicalAverage(visited_portion, points_within_portion);
  } else {
    overlap = points_within_portion;
  }

  if(visualize) {
    vis->show();
  }

  return overlap;
}

float getReverseRegistrationError(const PolarGridOfClouds &src_grid,
                                  const PolarGridOfClouds &trg_grid,
                                  const Eigen::Affine3f &init_transform,
                                  const float overlap,
                                  const CollarLinesRegistration::Parameters &registration_parameters,
                                  const CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                                  Eigen::Affine3f &registration_pose) {
  LinearMoveEstimator null_estimator(0);
  CollarLinesRegistration::Parameters reg_parameters_modified = registration_parameters;
  reg_parameters_modified.distance_threshold = CollarLinesRegistration::PORTION_VALUE_THRESHOLD;
  reg_parameters_modified.distance_threshold_value = overlap;
  CollarLinesRegistrationPipeline registration(null_estimator, pipeline_parameters, reg_parameters_modified);
  RegistrationOutcome result;
  registration.registerTwoGrids(src_grid, trg_grid, init_transform.matrix(), result);

  RegistrationOutcome inv_result;
  registration.registerTwoGrids(trg_grid, src_grid, init_transform.inverse().matrix(), inv_result);

  registration_pose = result.transformation;
  return tdiff(result.transformation, inv_result.transformation.inverse(), 10.0);
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  float threshold;
  float depth_quantile, depth_relative_tolerance, depth_absolute_tolerance;
  bool visualize, average_with_zbuffer_occupancy;
  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  float max_reg_error;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, threshold, max_reg_error,
      depth_quantile, depth_relative_tolerance, depth_absolute_tolerance,
      average_with_zbuffer_occupancy, registration_parameters, pipeline_parameters,
      visualize)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);

  for(int src_i = 0; src_i+1 < sequence.size();) {
    VelodyneMultiFrame::Ptr src_frame = sequence[src_i];
    PolarGridOfClouds src_grid(src_frame->clouds, calibration);
    PointCloud<VelodynePoint> src_cloud;
    src_frame->joinTo(src_cloud);
    transformPointCloud(src_cloud, src_cloud, Eigen::Affine3f(poses[src_i].rotation()));
    SphericalZbuffer src_zbuffer(src_cloud, 180, 90, depth_quantile);

    bool backward = false;
    for(int trg_i = src_i+1; trg_i < sequence.size();) {
      VelodyneMultiFrame::Ptr trg_frame = sequence[trg_i];
      PointCloud<VelodynePoint> trg_cloud;
      trg_frame->joinTo(trg_cloud);
      transformPointCloud(trg_cloud, trg_cloud, Eigen::Affine3f(poses[trg_i].rotation()));
      const Eigen::Vector3f translation = poses[trg_i].translation() - poses[src_i].translation();
      transformPointCloud(trg_cloud, trg_cloud, translation, Eigen::Quaternionf::Identity());
      const float overlap = computeOverlap(src_zbuffer, src_cloud, trg_cloud,
                                           depth_relative_tolerance, depth_absolute_tolerance,
                                           average_with_zbuffer_occupancy, visualize);
      if(overlap < threshold) {
        backward = true;
      }

      if(!backward && (trg_i+1 < sequence.size())) {
        // all OK - go forward
        trg_i++;

      } else {
        // some event - can not go forward

        const Eigen::Affine3f init_t = poses[src_i].inverse() * poses[trg_i];
        const PolarGridOfClouds trg_grid(trg_frame->clouds, calibration);
        Eigen::Affine3f registration;
        const float t_error = getReverseRegistrationError(src_grid, trg_grid, init_t, overlap, registration_parameters,
                                                    pipeline_parameters, registration);
        if(t_error < max_reg_error) {
          backward = false;
        }

        if(backward && (trg_i-1 > src_i)) {
          // can go backward
          trg_i--;
          cerr << "BACKOFF [" << src_i << ", " << trg_i << "], overlap: " << overlap << "; inv_error: " << t_error << endl;

        } else {
          // can not go forward nor backward
          cout << src_i << " " << trg_i << " " << overlap << " " << t_error << " " << registration << endl;
          src_i = trg_i + 1;
          break;
        }
      }

    }
  }

  return EXIT_SUCCESS;
}
