/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 01/05/2015
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

#include <iostream>

#include <but_velodyne/KittiUtils.h>

namespace but_velodyne
{

std::istream& operator>>(std::istream &stream, Eigen::Affine3f &pose) {
  float r1, r2, r3, r4, r5, r6, r7, r8, r9;
  float t1, t2, t3;
  stream >> r1 >> r2 >> r3 >> t1 >> r4 >> r5 >> r6 >> t2 >> r7 >> r8 >> r9 >> t3;

  pose = Eigen::Affine3f::Identity();
  pose.matrix().block(0,0,3,4) <<
      r1, r2, r3, t1,
      r4, r5, r6, t2,
      r7, r8, r9, t3;

  return stream;
}

std::ostream& operator<<(std::ostream &stream, const Eigen::Affine3f &pose) {
  const Eigen::Matrix4f::Scalar *poseArray = pose.matrix().data();
  stream << poseArray[0] << " " << poseArray[4] << " " << poseArray[8] << " " << poseArray[12] << " "
         << poseArray[1] << " " << poseArray[5] << " " << poseArray[9] << " " << poseArray[13] << " "
         << poseArray[2] << " " << poseArray[6] << " " << poseArray[10] << " " << poseArray[14];
  return stream;
}

} /* namespace but_velodyne */
