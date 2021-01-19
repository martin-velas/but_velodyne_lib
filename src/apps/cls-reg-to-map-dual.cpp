/*
 * Velodnye odometry estimation by dual map of Collar Line Segments.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 08/01/2021
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
#include <algorithm>

#include <boost/program_options.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/CollarLinesMap.h>
#include <but_velodyne/InterpolationSE3.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/PhaseFilter.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


class CollarLinesRegistrationToDualMap {
public:
    CollarLinesRegistrationToDualMap(
            const CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
            const CollarLinesRegistration::Parameters &registration_parameters,
            const float prune_ratio,
            const bool visualization) :
        cls_map_reg(pipeline_parameters, registration_parameters, prune_ratio, visualization),
        filter(pipeline_parameters.linesPerCellPreserved),
        phase_filter1(0.0, 0.5),
        phase_filter2(0.5, 1.0) {
    }

    void addToMap(const std::vector<VelodynePointCloud::Ptr> &point_clouds,
                  const SensorsCalibration &calibration, const Eigen::Affine3f &pose, const int frame_id) {
      cls_map_reg.addToMap(point_clouds, calibration, pose, frame_id);
    }

    bool runMapping(const VelodyneMultiFrame &multiframe,
                     const SensorsCalibration &calibration,
                     const Eigen::Affine3f &init_pose, const int frame_id,
                     Eigen::Affine3f &refined_pose,
                     vector <CLSMatch> &matches, Termination::Reason &reason,
                     float &diff_error) {
      PointCloud<PointXYZ> target_cloud;
      multiframe.joinTo(target_cloud);

      PolarGridOfClouds target_polar_grid(multiframe.clouds, calibration);
      LineCloud target_line_cloud_whole(target_polar_grid, cls_map_reg.params.linesPerCellGenerated, filter);

      LineCloud target_line_cloud1, target_line_cloud2;
      phase_filter1.filter(target_line_cloud_whole, target_line_cloud1);
      phase_filter2.filter(target_line_cloud_whole, target_line_cloud2);

      const Eigen::Affine3f reg_pose1 = cls_map_reg.registerLineCloud(target_line_cloud1, init_pose, matches, reason);
      const Eigen::Affine3f reg_pose2 = cls_map_reg.registerLineCloud(target_line_cloud2, init_pose, matches, reason);

      refined_pose = LinearInterpolationSE3(reg_pose1, reg_pose2).estimate(0.5);

      diff_error = tdiff(reg_pose1, reg_pose2, target_cloud);

      if(isDiffOk(diff_error)) {
        cls_map_reg.addToMap(target_line_cloud_whole, refined_pose, frame_id);
        cls_map_reg.pruneIfNeeded(target_line_cloud_whole.size());
        return true;
      } else {
        return false;
      }
    }

    bool isDiffOk(const float diff) {
      differences.push_back(diff);
      sort(differences.begin(), differences.end());

      if(differences.size() < 10) {
        return true;
      } else {
        return diff <= differences[differences.size()*0.8];
      }
    }

private:
    CollarLinesRegistrationToMap cls_map_reg;
    CollarLinesFilterRangeCheck filter;
    vector<float> differences;
    PhaseFilter phase_filter1, phase_filter2;
};

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    vector<Eigen::Affine3f> &init_poses, SensorsCalibration &calibration,
    vector<string> &clouds_to_process, float &prune_ratio, bool &visualization) {
  string pose_file, sensors_pose_file;

  po::options_description desc("Collar Lines Registration of Velodyne scans against the dual map\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  registration_parameters.prepareForLoading(desc);
  pipeline_parameters.prepareForLoading(desc);
  desc.add_options()
    ("help,h", "produce help message")
    ("poses", po::value<string>(&pose_file)->required(),
      "File with initial poses")
    ("sensors_pose_file", po::value<string>(&sensors_pose_file)->default_value(""),
        "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
    ("prune_ratio", po::value<float>(&prune_ratio)->default_value(0.1),
        "How many lines (portion) should be discarted from the map.")
    ("visualize", po::bool_switch(&visualization),
        "Run visualization")
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
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }

  init_poses = KittiUtils::load_kitti_poses(pose_file);

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  return true;
}

int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  vector<Eigen::Affine3f> init_poses;
  vector<string> clouds_filenames;
  SensorsCalibration calibration;
  float prune_ratio;
  bool visualization;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters, init_poses, calibration,
      clouds_filenames, prune_ratio, visualization)) {
    return EXIT_FAILURE;
  }

  CollarLinesRegistrationToDualMap registration(
      pipeline_parameters, registration_parameters, prune_ratio, visualization);

  VelodyneFileSequence sequence(clouds_filenames, calibration);
  vector<Eigen::Affine3f> refined_poses;
  bool used;
  float diff_error;
  for (int frame_i = 0; sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = sequence.getNext();

    Eigen::Affine3f pose;
    if(frame_i == 0) {
      used = true;
      diff_error = 0.0;
      pose = init_poses.front().matrix();
      registration.addToMap(multiframe.clouds, calibration, pose, frame_i);

    } else {
      Eigen::Affine3f init_pose = refined_poses.back() * (init_poses[frame_i-1].inverse() * init_poses[frame_i]);

      vector<CLSMatch> matches;
      Termination::Reason reason;
      used = registration.runMapping(multiframe, calibration, init_pose, frame_i, pose, matches, reason, diff_error);
    }

    refined_poses.push_back(pose);
    std::cout << used << " " << diff_error << " " << pose << endl;
  }

  return EXIT_SUCCESS;
}
