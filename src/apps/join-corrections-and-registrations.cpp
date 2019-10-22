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

#include <boost/program_options.hpp>

#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

class DoubleIdx {
public:
  DoubleIdx(const int srcIdx_ = -1, const int trgIdx_ = -1) :
    srcIdx(srcIdx_), trgIdx(trgIdx_) {
  }

  bool operator <(const DoubleIdx &o) const {
    if(this->srcIdx != o.srcIdx) {
      return this->srcIdx < o.srcIdx;
    } else {
      return this->trgIdx < o.trgIdx;
    }
  }

  int srcIdx, trgIdx;
};

typedef map<DoubleIdx, Eigen::Affine3f> Registrations;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &corrections,
                     Registrations &registrations) {
  string corrections_filename, registrations_filename;

  po::options_description desc("Join previous corrections with current registrations\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("corrections,c", po::value<string>(&corrections_filename)->required(),
          "File of previous corrections.")
      ("registrations,r", po::value<string>(&registrations_filename)->required(),
          "Registrations poses (with source/target indices).")
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

  corrections = KittiUtils::load_kitti_poses(corrections_filename);

  std::ifstream registrations_file(registrations_filename.c_str());
  if(!registrations_file.is_open()) {
    std::perror((std::string("Unable to open file: ") + registrations_filename).c_str());
    return false;
  }
  while(true) {
    Eigen::Affine3f pose;
    DoubleIdx indices;
    registrations_file >> indices.srcIdx >> indices.trgIdx >> pose;

    if(registrations_file.eof()) {
      break;
    } else {
      registrations[indices] = pose;
    }
  }

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> C;
  Registrations R;
  if(!parse_arguments(argc, argv, C, R)) {
    return EXIT_FAILURE;
  }

  for(Registrations::const_iterator r = R.begin(); r != R.end(); r++) {
    const Eigen::Affine3f T = C[r->first.srcIdx].inverse() * r->second * C[r->first.trgIdx];
    cout << r->first.srcIdx << " " << r->first.trgIdx << " " << T << endl;
  }

  return EXIT_SUCCESS;
}
