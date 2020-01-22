/*
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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/AdaptiveIntensitiesNormalization.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef map<size_t, PointCloud<PointXYZRGB>::Ptr> CloudsIndexed;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &imu_poses,
                     Eigen::Affine3f &init_calibration,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process) {
  string clouds_indices_fn, imu_poses_fn, initial_calibration_fn, sensors_pose_fn;

  po::options_description desc("IMU Velodyne calibration using wall normals\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
  ;

  desc.add_options()
    ("imu_poses,p", po::value<string>(&imu_poses_fn)->required(), "IMU poses.")
    ("init_calibration,c", po::value<string>(&initial_calibration_fn)->required(),
            "Initial IMU-Velodyne calibration.")
    ("sensors_pose_file", po::value<string>(&sensors_pose_fn)->default_value(""),
            "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  try {
      po::notify(vm);
  }
  catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }

  imu_poses = KittiUtils::load_kitti_poses(imu_poses_fn);
  init_calibration = KittiUtils::load_kitti_poses(initial_calibration_fn).front();

  if(sensors_pose_fn.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_fn);
  }

  return true;
}

float deg(const float radians) {
  return radians * 180.0 / 3.14159;
}

float rad(const float degrees) {
  return degrees / 180.0 * 3.14159;
}

class ManualCalibration {

public:

    ManualCalibration(const CloudsIndexed &clouds_, const Eigen::Translation3f &move_origin_,
                      const vector<Eigen::Affine3f> &imu_poses_, const Eigen::Affine3f &init_calibration) :
                      clouds(clouds_), move_origin(move_origin_), imu_poses(imu_poses_), delta(0.1),
                      calibration(init_calibration) {
      vis.getViewer()->setBackgroundColor(0.2, 0.2, 0.8);
      vis.getViewer()->registerKeyboardCallback(&ManualCalibration::keyCallback, *this);
    }

    Eigen::Affine3f run() {
      setDataToVisualizer();
    }

protected:

    void setDataToVisualizer(void) {
      vis.keepOnlyClouds(0);
      vis.getViewer()->removeAllShapes();
      PointCloud<PointWithSource>::Ptr sum_cloud(new PointCloud<PointWithSource>);
      for(CloudsIndexed::const_iterator cloud_it = clouds.begin(); cloud_it != clouds.end(); cloud_it++) {
        const size_t frame_i = cloud_it->first;
        const Eigen::Affine3f pose = move_origin * imu_poses[frame_i] * calibration;

        vis.addColorPointCloud(cloud_it->second, pose.matrix());
      }
      float rx, ry, rz;
      getEulerAngles(calibration, rx, ry, rz);
      stringstream str_rx, str_ry, str_rz, str_delta;
      str_delta << setprecision(6) << delta;
      vis.getViewer()->addText("delta: " + str_delta.str(), 20, 110, 20, 1.0, 0.8, 0,  "rx_delta");
      str_rx << setprecision(6) << deg(rx);
      vis.getViewer()->addText("Rx: " + str_rx.str(), 20, 20, 20, 1.0, 1.0, 0,  "rx_text");
      str_ry << setprecision(6) << deg(ry);
      vis.getViewer()->addText("Ry: " + str_ry.str(), 20, 50, 20, 1.0, 1.0, 0,  "ry_text");
      str_rz << setprecision(6) << deg(rz);
      vis.getViewer()->addText("Rz: " + str_rz.str(), 20, 80, 20, 1.0, 1.0, 0,  "rz_text");
      vis.show();
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
        calibration = calibration * delta_T;
        cerr << "calibration: " << calibration << endl;
        setDataToVisualizer();
      }
    }

private:
    const CloudsIndexed clouds;
    const Eigen::Translation3f move_origin;
    const vector<Eigen::Affine3f> imu_poses;
    Visualizer3D vis;
    float delta;
    Eigen::Affine3f calibration;
};


int main(int argc, char** argv) {

  vector<Eigen::Affine3f> imu_poses;
  Eigen::Affine3f imu_velodyne_calibration;
  SensorsCalibration velodynes_calibration;
  vector<string> clouds_fnames;
  if(!parse_arguments(argc, argv, imu_poses, imu_velodyne_calibration, velodynes_calibration, clouds_fnames)) {
    return EXIT_FAILURE;
  }

  CloudsIndexed clouds;
  VelodyneFileSequence sequence(clouds_fnames, velodynes_calibration);
  Eigen::Vector3f centroid = Eigen::Vector3f::Identity();
  for (sequence.reset(); sequence.hasNext(); ) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PointCloud<PointXYZI> cloud;
    multiframe.joinTo(cloud);
    const size_t frame_i = KittiUtils::kittiNameToIndex(multiframe.filenames.front());
    Visualizer3D::normalizeMinMaxIntensity(cloud, cloud, 0.05, 0.0, 1.0);
    clouds[frame_i] = Visualizer3D::colorizeCloud(cloud, true);

    centroid += imu_poses[frame_i].translation();
  }
  centroid /= sequence.size();
  Eigen::Translation3f move_origin(-centroid);

  ManualCalibration calibration(clouds, move_origin, imu_poses, imu_velodyne_calibration);
  cout << calibration.run() << endl;

  return EXIT_SUCCESS;
}
