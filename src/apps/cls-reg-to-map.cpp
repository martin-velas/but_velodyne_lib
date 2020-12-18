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
#include <algorithm>

#include <boost/program_options.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/CollarLinesMap.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    vector<Eigen::Affine3f> &init_poses, SensorsCalibration &calibration,
    vector<string> &clouds_to_process, float &prune_ratio, bool &visualization,
    string &matches_output_file, float &matches_portion, int &reset_after_nframes) {
  string pose_file, sensors_pose_file;

  po::options_description desc("Collar Lines Registration of Velodyne scans against the map\n"
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
    ("matches_output", po::value<string>(&matches_output_file)->default_value(""),
        "Output file for the matches used in final registration.")
    ("matches_portion", po::value<float>(&matches_portion)->default_value(0.9),
        "Keep this portion of best matches (by the distance)")
    ("reset_after_nframes", po::value<int>(&reset_after_nframes)->default_value(-1),
         "Reset of the map after each N frames")
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
  string matches_output_filename;
  float matches_portion;
  int reset_after_nframes;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters, init_poses, calibration,
      clouds_filenames, prune_ratio, visualization, matches_output_filename, matches_portion,
      reset_after_nframes)) {
    return EXIT_FAILURE;
  }

  CollarLinesRegistrationToMap registration(
      pipeline_parameters, registration_parameters, prune_ratio, visualization);

  ofstream matches_output;
  if(!matches_output_filename.empty()) {
    matches_output.open(matches_output_filename.c_str());
  }
  CLSMatchByCoeffComparator clsMatchByCoeffComparator;

  VelodyneFileSequence sequence(clouds_filenames, calibration);
  vector<Eigen::Affine3f> refined_poses;
  for (int frame_i = 0; sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = sequence.getNext();

    Eigen::Affine3f pose;
    if(frame_i == 0) {
      pose = init_poses.front().matrix();
      registration.addToMap(multiframe.clouds, calibration, pose, frame_i);

    } else {
      Eigen::Affine3f init_pose = refined_poses.back() * (init_poses[frame_i-1].inverse() * init_poses[frame_i]);
      if(reset_after_nframes > 0 && (frame_i % reset_after_nframes) == 0) {
        registration.reset();
        pose = init_pose;
        registration.addToMap(multiframe.clouds, calibration, pose, frame_i);

      } else {
        vector<CLSMatch> matches;
        Termination::Reason reason;
        pose = registration.runMapping(multiframe, calibration, init_pose, frame_i, matches, reason);
        cerr << "Matches (unfiltered): " << matches.size() << endl;

        sort(matches.begin(), matches.end(), clsMatchByCoeffComparator);
        matches.erase(matches.begin() + matches_portion*matches.size(), matches.end());
        cerr << "Matches (filtered " << matches_portion*100 << "%): " << matches.size() << endl;

        if(matches_output.is_open()) {
          for(vector<CLSMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
            const int src_frame_id = m->getSourceLine().frame_id;
            const PointXYZ &trg_pt = m->getTrgPt();
            PointXYZ real_src_point = transformPoint(m->getSrcPt(), refined_poses[src_frame_id].inverse());
            matches_output
                    << src_frame_id << " "
                    << real_src_point.x << " " << real_src_point.y << " " << real_src_point.z << " "
                    << frame_i << " "
                    << trg_pt.x << " " << trg_pt.y << " " << trg_pt.z << endl;
          }
        }
      }
    }
    refined_poses.push_back(pose);
    std::cout << pose << endl;
  }

  return EXIT_SUCCESS;
}
