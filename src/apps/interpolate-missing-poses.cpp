/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 08/03/2019
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

#include <boost/program_options.hpp>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

using namespace std;
using namespace but_velodyne;


int main(int argc, char** argv) {

  int last_index = -1;
  Eigen::Affine3f last_pose;

  while(true) {
    int index;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    cin >> index >> pose;

    if(cin.eof()) {
      break;
    } else {
      LinearInterpolationSE3 interpolation(last_pose, pose);
      for(int i = last_index+1; i < index; i++) {
        cout << interpolation.estimate((i-last_index)/float(index-last_index)) << endl;
      }

      cout << pose << endl;

      last_pose = pose;
      last_index = index;
    }
  }

  return EXIT_SUCCESS;
}
