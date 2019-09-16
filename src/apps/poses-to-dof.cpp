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

#include <but_velodyne/KittiUtils.h>

using namespace std;
using namespace but_velodyne;


int main(int argc, char** argv) {

  while(true) {
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    cin >> pose;

    if(cin.eof()) {
      return EXIT_SUCCESS;
    }

    float tx, ty, tz, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(pose, tx, ty, tz, roll, pitch, yaw);

    printf("%f %f %f %f %f %f\n", tx, ty, tz, roll, pitch, yaw);
  }
  return EXIT_SUCCESS;
}
