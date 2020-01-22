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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/Calibration.h>
#include <pcl/common/eigen.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

int main(int argc, char** argv) {

  vector<NormalsPair> imu_velodyne_normals;
  load_normals(cin, imu_velodyne_normals);

  Visualizer3D vis;
  show_normals(vis, imu_velodyne_normals);

  Eigen::Matrix4f calibration = compute_calibration(imu_velodyne_normals);

  cerr << calibration << endl;
  KittiUtils::printPose(cout, calibration);

  for(vector<NormalsPair>::iterator pair = imu_velodyne_normals.begin(); pair < imu_velodyne_normals.end(); pair++) {
    pair->velodyne_normal = calibration.block(0, 0, 3, 3) * pair->velodyne_normal;
  }

  show_normals(vis, imu_velodyne_normals);

  return EXIT_SUCCESS;
}
