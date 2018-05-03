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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class SeqViewer {

public:

  SeqViewer(VelodyneFileSequence &sequence_, const vector<Eigen::Affine3f> &poses_,
      int seq_len_, int step_) :
    sequence(sequence_), poses(poses_), frames(seq_len_), step(step_) {

    vis.getViewer()->registerKeyboardCallback(&SeqViewer::keyCallback, *this);
    vis.getViewer()->setBackgroundColor(0, 0, 0);

    for(int i = 0; i < seq_len_ && sequence.hasNext(); i++) {
      frames[i].reset(new PointCloud<PointXYZI>);
      sequence.getNext().joinTo(*frames[i]);
      transformPointCloud(*frames[i], *frames[i], poses[sequence.getIndex()]);
    }

    setDataToVis();
    vis.show();
  }

protected:

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "a") {
        for(int i = 0; i < step && sequence.hasPrev(); i++) {
          PointCloud<PointXYZI>::Ptr new_frame(new PointCloud<PointXYZI>);
          sequence.getPrev().joinTo(*new_frame);
          transformPointCloud(*new_frame, *new_frame, poses[sequence.getIndex()]);
          cerr << "pose " << sequence.getIndex() << " of " << poses.size() << " " << poses[sequence.getIndex()].matrix() << endl;
          frames.push_front(new_frame);
          frames.pop_back();
        }
      } else if(event.getKeySym() == "d") {
        for(int i = 0; i < step && sequence.hasNext(); i++) {
          PointCloud<PointXYZI>::Ptr new_frame(new PointCloud<PointXYZI>);
          sequence.getNext().joinTo(*new_frame);
          transformPointCloud(*new_frame, *new_frame, poses[sequence.getIndex()]);
          cerr << "pose " << sequence.getIndex() << " of " << poses.size() << " " << poses[sequence.getIndex()].matrix() << endl;
          frames.push_back(new_frame);
          frames.pop_front();
        }
      }
    }
    setDataToVis();
  }

  void setDataToVis(void) {
    PointCloud<PointXYZI> sum_cloud;
    cerr << frames.size() << " frames" << endl;
    for(deque<PointCloud<PointXYZI>::Ptr>::iterator cloud = frames.begin(); cloud != frames.end(); cloud++) {
      sum_cloud += **cloud;
    }
    PointCloud<PointXYZRGB>::Ptr rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);
    vis.keepOnlyClouds(0).addColorPointCloud(rgb_cloud);
  }

private:
  int step;
  VelodyneFileSequence &sequence;
  Visualizer3D vis;
  deque<PointCloud<PointXYZI>::Ptr> frames;
  const vector<Eigen::Affine3f> poses;
};

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     int &seq_length, int &step_size) {
  string pose_filename, calibration_filename;

  po::options_description desc("Subsequence viewer\n"
      "======================================\n"
      " * Reference(s): Velas et al, ???\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Sensor poses.")
      ("seq_length,l", po::value<int>(&seq_length)->default_value(100), "How many frames show at once")
      ("step,s", po::value<int>(&step_size)->default_value(30), "Step size")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  return true;
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int seq_len, step_size;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, seq_len, step_size)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  SeqViewer viewer(sequence, poses, seq_len, step_size);

  return EXIT_SUCCESS;
}
