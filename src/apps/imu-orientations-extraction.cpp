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
#include <iostream>

#include <pcl/common/eigen.h>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <boost/program_options.hpp>

#include <but_velodyne/KittiUtils.h>

using namespace std;
using namespace but_velodyne;
using namespace rapidjson;
namespace po = boost::program_options;


bool parse_arguments(int argc, char **argv,
                     string &quat_label, Eigen::Affine3f &imu_to_velodyne) {

  string imu_to_velodyne_filename;

  po::options_description desc("IMU Orientations extraction\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("quat_label,l", po::value<string>(&quat_label)->default_value("quaternion_full"),
          "Labels of quaternions.")
      ("imu_to_velodyne_calibration,c", po::value<string>(&imu_to_velodyne_filename)->required(),
          "IMU to Velodyne calibration.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

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

  imu_to_velodyne = KittiUtils::load_kitti_poses(imu_to_velodyne_filename).front();

  return true;
}


int main(int argc, char** argv) {

  string label;
  Eigen::Affine3f C;
  if(!parse_arguments(argc, argv, label, C)) {
    return EXIT_FAILURE;
  }

  Eigen::Matrix4f VELO_TO_IMU_MATRIX;
  VELO_TO_IMU_MATRIX << 0,0,1,0,
                        1,0,0,0,
                        0,1,0,0,
                        0,0,0,1;
  const Eigen::Affine3f VELO_TO_IMU(VELO_TO_IMU_MATRIX);
  const Eigen::Affine3f VELO_TO_IMU_INV = VELO_TO_IMU.inverse();

  char readBuffer[65536];
  FileReadStream is(stdin, readBuffer, sizeof(readBuffer));

  Document d;
  d.ParseStream(is);

  Eigen::Affine3f M_first_inv;
  for(SizeType i = 0; i < d.Size(); i++) {
    const float time = d[i]["timeStamp"].GetDouble();
    const Value& quat_array = d[i][label.c_str()];  // x, y, z, w
    assert(quat_array.Size() == 4);

    // expecting w, x, y, z
    Eigen::Quaternionf Q(
        quat_array[3].GetDouble(),
        quat_array[0].GetDouble(),
        quat_array[1].GetDouble(),
        quat_array[2].GetDouble()
    );

    Eigen::Affine3f M(Eigen::Translation3f::Identity() * Q);

    if(i == 0) {
      M_first_inv = M.inverse();
    }
    M = M_first_inv * M;

    cout << time << " " << (VELO_TO_IMU_INV * M * VELO_TO_IMU) * C << endl;
  }

  return EXIT_SUCCESS;
}
