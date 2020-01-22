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
#include <but_velodyne/InterpolationSE3.h>
#include <but_velodyne/Calibration.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

class NormalIndexed {
public:
    Eigen::Vector3f n;
    int i;
};

std::ostream& operator<<(std::ostream &stream, const NormalIndexed &n) {
  stream << n.i << " " << n.n(0) << " " << n.n(1) << " " << n.n(2);
  return stream;
}

std::istream& operator>>(std::istream &stream, NormalIndexed &n) {
  stream >> n.i >> n.n(0) >> n.n(1) >> n.n(2);
  return stream;
}

void load_normals(const string &fn, vector<NormalIndexed> &normals) {
  std::ifstream file(fn.c_str());
  if(!file.is_open()) {
    std::perror((std::string("Unable to open file: ") + fn).c_str());
    exit(1);
  }
  while(true) {
    NormalIndexed normal;
    file >> normal;
    if(file.eof()) {
      break;
    } else {
      normals.push_back(normal);
    }
  }
}

bool parse_arguments(int argc, char **argv,
                     vector<NormalIndexed> &normals,
                     vector<Eigen::Affine3f> &imu_poses,
                     Eigen::Affine3f &init_calibration,
                     size_t &iterations) {
  string normals_fn, imu_poses_fn, initial_calibration_fn;

  po::options_description desc("IMU Velodyne calibration using wall normals\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
  ;

  desc.add_options()
    ("wall_normals_indexed,n", po::value<string>(&normals_fn)->required(), "Indexed wall normals.")
    ("imu_poses,p", po::value<string>(&imu_poses_fn)->required(), "IMU poses.")
    ("init_calibration,c", po::value<string>(&initial_calibration_fn)->required(), "IMU poses.")
    ("iterations,i", po::value<size_t>(&iterations)->default_value(100), "Iterations.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_to_process;

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

  load_normals(normals_fn, normals);

  return true;
}

void generate_normal_pairs(const vector<NormalIndexed> &normals,
        const vector<Eigen::Affine3f> &imu_poses,
        const Eigen::Matrix3f &Rc,
        vector<NormalsPair> &pairs) {
  pairs.resize(normals.size());
  for(int i = 0; i < normals.size(); i++) {
    const Eigen::Matrix3f Ri = imu_poses[normals[i].i].rotation().matrix();
    const Eigen::Vector3f v = normals[i].n;

    pairs[i].velodyne_normal = v;

    Eigen::Vector3f vi = Ri * Rc * v;
    vi(1) = 0;
    vi.normalize();
    pairs[i].imu_normal = Ri.inverse() * vi;
  }
}

int main(int argc, char** argv) {

  vector<NormalIndexed> normals;
  vector<Eigen::Affine3f> imu_poses;
  Eigen::Affine3f init_calibration;
  size_t iterations;
  if(!parse_arguments(argc, argv, normals, imu_poses, init_calibration,iterations)) {
    return EXIT_FAILURE;
  }

  Eigen::Matrix3f Rc = init_calibration.rotation().matrix();
  Visualizer3D vis;
  for(int i = 0; i < iterations; i++) {
    vector<NormalsPair> pairs;
    generate_normal_pairs(normals, imu_poses, Rc, pairs);
    show_normals(vis, pairs);

    Rc = compute_calibration(pairs).block(0, 0, 3, 3);

    cerr << i << ": " << Rc << endl;
    for(int pi = 0; pi < pairs.size(); pi++) {
      pairs[pi].velodyne_normal = Rc * pairs[pi].velodyne_normal;
      cerr << "Normal " << pi << " calibrated to: " << pairs[pi].velodyne_normal.x() << " " <<
        pairs[pi].velodyne_normal.y() << " " << pairs[pi].velodyne_normal.z() << " " << endl;
    }
    show_normals(vis, pairs);
  }

  Eigen::Affine3f C = init_calibration;
  C.matrix().block(0, 0, 3, 3) = Rc;
  cerr << C << endl;

  return EXIT_SUCCESS;
}
