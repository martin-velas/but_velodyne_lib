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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/Overlap.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>

#include <pcl/common/eigen.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     int &source_index,
                     int &frames_dist, int &max_frames_dist, bool &circular,
                     int &lines_generated, int &lines_preserved,
                     ClsOverlapEstimator::Parameters &params, bool &visualize) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("src_idx", po::value<int>(&source_index)->default_value(-1), "Source index, if not set, all clouds are considered as source")
    ("frames_dist", po::value<int>(&frames_dist)->default_value(1), "Frames distance (minimal of max is set) to compute overlap")
    ("max_frames_dist", po::value<int>(&max_frames_dist)->default_value(-1), "Maximal frames distance. If not set, frames_dist is used.")
    ("circular", po::bool_switch(&circular), "The trajectory is considered to be circular.")
    ("visualize", po::bool_switch(&visualize), "Run visualization.")
    ("lines_generated", po::value<int>(&lines_generated)->default_value(20), "Lines generated.")
    ("lines_preserved", po::value<int>(&lines_preserved)->default_value(5), "Lines preserved.")
  ;
  params.prepareForLoading(desc);

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

  if(max_frames_dist < 0) {
    max_frames_dist = frames_dist;
  }

  return true;
}

LineCloud::Ptr generate_lines(const VelodyneMultiFrame::Ptr frame,
                              const SensorsCalibration &calibration,
                              const int lines_generated, const int lines_preserved) {
  CollarLinesFilter cls_filter(lines_preserved);
  PolarGridOfClouds grid(frame->clouds, calibration);
  LineCloud::Ptr lines(new LineCloud(grid, lines_generated, cls_filter));
  return lines;
}


int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int source_index, expected_frames_dist, max_frames_dist;
  bool circular, visualize;
  int lines_generated, lines_preserved;
  ClsOverlapEstimator::Parameters params;

  if(!parse_arguments(argc, argv,
    poses, calibration, filenames, source_index, expected_frames_dist, max_frames_dist, circular,
    lines_generated, lines_preserved, params, visualize)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  Visualizer3D::Ptr vis;

  for(int i = 0; i < sequence.size(); i++) {
    if(source_index >= 0) {
      i = source_index;
    }
    LineCloud::Ptr src_lines = generate_lines(sequence[i], calibration, lines_generated, lines_preserved);

    ClsOverlapEstimator overlap_estimator(src_lines, params);

    for(int j = i+1; j < sequence.size(); j++) {
      int frames_distace = get_frames_distance(i, j, sequence.size(), circular);
      if(expected_frames_dist <= frames_distace && frames_distace <= max_frames_dist) {

        LineCloud::Ptr trg_lines = generate_lines(sequence[j], calibration, lines_generated, lines_preserved);
        Eigen::Affine3f T_delta = poses[i].inverse() * poses[j];

        float overlap;
        if(visualize) {
          LineCloud within, rest;
          overlap = overlap_estimator.overlapWith(*trg_lines, T_delta, within, rest);
          if(!vis) {
            vis.reset(new Visualizer3D);
          }
          vis->keepOnlyClouds(0)
              .setColor(0, 0, 0).addPointCloud(*(src_lines->getMiddles()))
              .setColor(0, 200, 0).addPointCloud(*(within.getMiddles()), T_delta.matrix())
              .setColor(255, 0, 0).addPointCloud(*(rest.getMiddles()), T_delta.matrix());
        } else {
          overlap = overlap_estimator.overlapWith(*trg_lines, T_delta);
        }

        cout << i << " " << j << " " << overlap << endl;

        if(visualize) {
          vis->show();
        }
      }
    }
    if(source_index >= 0) {
      break;
    }
  }

  return EXIT_SUCCESS;
}
