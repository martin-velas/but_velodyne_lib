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
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/SubseqRegistration.h>

#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class PairPicker {

public:

  PairPicker(VelodyneFileSequence &sequence_, const vector<Eigen::Affine3f> &poses_) :
    frames(sequence_.size()), poses(poses_), src_i(0), trg_i(sequence_.size()/2),
    sum_cloud(new PointCloud<PointXYZRGB>) {
    vis.getViewer()->registerKeyboardCallback(&PairPicker::keyCallback, *this);

    for (int frame_i = 0; sequence_.hasNext(); frame_i++) {
      PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
      VelodyneMultiFrame multiframe = sequence_.getNext();
      multiframe.joinTo(*cloud);
      transformPointCloud(*cloud, *cloud, poses[frame_i]);
      subsample_cloud<PointXYZI>(cloud, 0.2);
      frames[frame_i] += *cloud;
      subsample_cloud<PointXYZI>(cloud, 0.01);
      *sum_cloud += *Visualizer3D::colorizeCloud(*cloud, true);
    }
  }

  void run(int &src_i_, int &trg_i_) {
    setDataToVis();
    vis.show();
    src_i_ = src_i;
    trg_i_ = trg_i;
  }

protected:

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "a") {
        src_i--;
      } else if(event.getKeySym() == "s") {
        src_i++;
      } else if(event.getKeySym() == "k") {
        trg_i--;
      } else if(event.getKeySym() == "l") {
        trg_i++;
      }
    }
    src_i = MIN(MAX(src_i, 0), frames.size()-1);
    trg_i = MIN(MAX(trg_i, 0), frames.size()-1);
    setDataToVis();
  }

  void setDataToVis(void) {
    vis.keepOnlyClouds(0).addColorPointCloud(sum_cloud);
    vis.setColor(255, 0, 0).addPointCloud(frames[src_i]);
    vis.setColor(0, 0, 255).addPointCloud(frames[trg_i]);
  }

private:
  vector < PointCloud<PointXYZI> > frames;
  Visualizer3D vis;
  const vector<Eigen::Affine3f> poses;
  int src_i, trg_i;
  PointCloud<PointXYZRGB>::Ptr sum_cloud;
};

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     CollarLinesRegistration::Parameters &reg_params,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_params) {
  string pose_filename, calibration_filename;

  po::options_description desc("Subsequence viewer\n"
      "======================================\n"
      " * Reference(s): Velas et al, ???\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Sensor poses.")
   ;
  reg_params.prepareForLoading(desc);
  pipeline_params.prepareForLoading(desc);

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

void buildLineCloud(const VelodyneMultiFrame &multiframe,
    const Eigen::Affine3f &pose,
    const SensorsCalibration &calibration,
    int lines_per_cell_gen, int lines_per_cell_preserve,
    LineCloud &out_lines) {
  CollarLinesFilter filter(lines_per_cell_preserve);
  PolarGridOfClouds polar_grid(multiframe.clouds, multiframe.calibration);
  LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
  line_cloud.transform(pose.matrix());
  out_lines += line_cloud;
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  CollarLinesRegistration::Parameters reg_params;
  CollarLinesRegistrationPipeline::Parameters pipeline_params;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, reg_params, pipeline_params)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  PairPicker picker(sequence, poses);
  int src_i, trg_i;
  picker.run(src_i, trg_i);

  LineCloud src_lines, trg_lines;
  buildLineCloud(sequence[src_i], poses[src_i], calibration,
      pipeline_params.linesPerCellGenerated, pipeline_params.linesPerCellPreserved,
      src_lines);
  buildLineCloud(sequence[trg_i], poses[trg_i], calibration,
      pipeline_params.linesPerCellGenerated, pipeline_params.linesPerCellPreserved,
      trg_lines);
  ManualSubseqRegistration registration(src_lines, trg_lines,
      Eigen::Affine3f::Identity(),
      pipeline_params,
      reg_params);
  Eigen::Affine3f t = registration.run();

  cout << src_i << " " << trg_i << " " << endl;
  KittiUtils::printPose(std::cout, t.matrix());

  return EXIT_SUCCESS;
}
