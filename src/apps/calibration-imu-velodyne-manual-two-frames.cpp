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

#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     Eigen::Affine3f &src_pose, Eigen::Affine3f &trg_pose,
                     VelodynePointCloud &src_cloud, VelodynePointCloud &trg_cloud,
                     SensorsCalibration &velodynes_calibration,
                     Eigen::Affine3f &imu_to_velodyne) {
  string pose_filename, calibration_filename, init_filename;

  po::options_description desc("Poses and point clouds visualization\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("velodyne_calibration,c", po::value<string>(&calibration_filename)->default_value(""), "Velodynes calibration file.")
    ("imu_velo_calibration,i", po::value<string>(&init_filename)->default_value(""), "IMU-Velodyne calibration file.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(pose_filename);

  if(calibration_filename.empty()) {
    velodynes_calibration = SensorsCalibration();
  } else {
    velodynes_calibration = SensorsCalibration(calibration_filename);
  }

  if(init_filename.empty()) {
    imu_to_velodyne = Eigen::Affine3f::Identity();
  } else {
    imu_to_velodyne = KittiUtils::load_kitti_poses(init_filename).front();
  }

  if(clouds_to_process.size() != 2*velodynes_calibration.sensorsCount()) {
    cerr << "Expected 2 frames (" << 2*velodynes_calibration.sensorsCount() << " clouds for "
         << velodynes_calibration.sensorsCount() << " sensors). Found: " << clouds_to_process.size() << endl;
    return false;
  }

  VelodyneFileSequence sequence(clouds_to_process, velodynes_calibration);
  sequence[0]->joinTo(src_cloud);
  sequence[1]->joinTo(trg_cloud);

  src_pose = poses[KittiUtils::kittiNameToIndex(sequence[0]->filenames.front())];
  trg_pose = poses[KittiUtils::kittiNameToIndex(sequence[1]->filenames.front())];

  return true;
}

class ManualCalibration {

public:

    ManualCalibration(const VelodynePointCloud &src_cloud_, const VelodynePointCloud &trg_cloud_,
                      const Eigen::Affine3f &src_pose_, const Eigen::Affine3f &trg_pose_,
                      const Eigen::Affine3f &init_calibration_) :
            src_cloud(src_cloud_), trg_cloud(trg_cloud_), src_pose(src_pose_), trg_pose(trg_pose_), delta(0.1) {
      trg_pose = src_pose.inverse() * trg_pose * init_calibration_;
      src_pose = init_calibration_;
      vis.getViewer()->registerKeyboardCallback(&ManualCalibration::keyCallback, *this);
    }

    Eigen::Affine3f run() {
      setDataToVisualizer();
    }

protected:

    void setDataToVisualizer(void) {
      cout << src_pose << endl;
      vis.keepOnlyClouds(0)
         .setColor(255, 0, 0).addPointCloud(src_cloud, src_pose.matrix())
         .setColor(0, 0, 255).addPointCloud(trg_cloud, trg_pose.matrix())
         .show();
    }

    static float rad(const float degrees) {
      return degrees / 180.0 * 3.14159;
    }

    void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
      if(event.keyDown()) {
        float rx, ry, rz;
        rx = ry = rz = 0.0;
        if(event.getKeySym() == "KP_Multiply") {
          delta *= 2;
        } else if(event.getKeySym() == "KP_Divide") {
          delta /= 2;
        } else if(event.getKeySym() == "KP_1") {
          rx = -delta;
        } else if(event.getKeySym() == "KP_2") {
          rx = +delta;
        } else if(event.getKeySym() == "KP_4") {
          ry = -delta;
        } else if(event.getKeySym() == "KP_5") {
          ry = +delta;
        } else if(event.getKeySym() == "KP_7") {
          rz = -delta;
        } else if(event.getKeySym() == "KP_8") {
          rz = +delta;
        }
        Eigen::Affine3f delta_T;
        getTransformation(0.0, 0.0, 0.0, rad(rx), rad(ry), rad(rz), delta_T);
        src_pose = src_pose * delta_T;
        trg_pose = trg_pose * delta_T;
        setDataToVisualizer();
      }
    }

private:
    Visualizer3D vis;
    float delta;
    const VelodynePointCloud &src_cloud, &trg_cloud;
    Eigen::Affine3f src_pose, trg_pose;
};

int main(int argc, char** argv) {

  Eigen::Affine3f src_pose, trg_pose;
  VelodynePointCloud src_cloud, trg_cloud;
  SensorsCalibration velodynes_calibration;
  Eigen::Affine3f imu_to_velodyne;
  if(!parse_arguments(argc, argv, src_pose, trg_pose, src_cloud, trg_cloud,
                      velodynes_calibration, imu_to_velodyne)) {
    return EXIT_FAILURE;
  }

  ManualCalibration calibration(src_cloud, trg_cloud, src_pose, trg_pose, imu_to_velodyne);
  calibration.run();

  return EXIT_SUCCESS;
}
