/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2015
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
#include <cassert>

#include <pcl/common/transforms.h>

#include <but_velodyne/PolarGridOfClouds.h>

using namespace std;
using namespace pcl;

namespace but_velodyne {

std::ostream& operator<< (std::ostream &out, const CellId &id) {
  out << "[r:" << id.ring << "; p:" << id.polar << "]";
  return out;
}

PolarGridOfClouds::PolarGridOfClouds(
    const VelodynePointCloud &point_cloud,
    int polar_superbins_, int bin_subdivision_, Eigen::Affine3f sensor_pose,
    bool redistribute) :
      polar_superbins(polar_superbins_),
      bin_subdivision(bin_subdivision_),
      rings(point_cloud.ringCount()),
      sensors(1) {
  allocateClouds();
  fill(point_cloud, 0, sensor_pose, redistribute);
}

PolarGridOfClouds::PolarGridOfClouds(const std::vector<VelodynePointCloud::Ptr> &point_clouds,
    const SensorsCalibration &calibration_,
    int polar_superbins_, int bin_subdivision_, bool redistribute) :
      polar_superbins(polar_superbins_),
      bin_subdivision(bin_subdivision_),
      rings(VelodynePointCloud::getMaxRingCount(point_clouds)),
      sensors(calibration_.sensorsCount()) {
  assert(point_clouds.size() == calibration_.sensorsCount());
  allocateClouds();
  for(int i = 0; i < calibration_.sensorsCount(); i++) {
    fill(*point_clouds[i], i, calibration_.ofSensor(i), redistribute);
  }
}

PolarGridOfClouds::PolarGridOfClouds(int polar_superbins_, int bin_subdivision_, int rings_,
    int sensors_) :
      polar_superbins(polar_superbins_),
      bin_subdivision(bin_subdivision_),
      rings(rings_),
      sensors(sensors_) {
  allocateClouds();
}

void PolarGridOfClouds::allocateClouds() {
  int total_cells = polar_superbins*bin_subdivision*rings*sensors;
  for(int i = 0; i < total_cells; i++) {
    polar_grid.push_back(VelodynePointCloud::Ptr(new VelodynePointCloud));
  }
}

void PolarGridOfClouds::fill(
    const VelodynePointCloud &point_cloud,
    int sensor_idx, const Eigen::Affine3f &sensor_pose,
    bool redistribute) {
  std::vector<float> borders;
  if(redistribute) {
    borders = point_cloud.getMaxOfRingRanges();
  }
  for(VelodynePointCloud::const_iterator pt = point_cloud.begin();
        pt < point_cloud.end();
        pt++) {
    int ring;
    if(redistribute) {
      ring = computeNewRingIndex(*pt, borders);
    } else {
      ring = pt->ring;
    }
    assert(ring < rings);
    assert(ring >= 0);

    int polar_bin = getPolarBinIndex(*pt);
    assert(polar_bin < getPolarBins());
    assert(polar_bin >= 0);

    CellId cell_id(polar_bin, ring, sensor_idx);
    this->at(cell_id).push_back(*pt);
    indices.push_back(cell_id);
  }

  transform(sensor_pose.matrix(), sensor_idx);
}

void PolarGridOfClouds::transform(const Eigen::Matrix4f &t, int sensor_idx) {
  if(sensor_idx < 0) {
    for(int i = 0; i < polar_grid.size(); i++) {
      transformPointCloud(*polar_grid[i], *polar_grid[i], t);
    }
  } else {
    for(int polar = 0; polar < getPolarBins(); polar++) {
      for(int ring = 0; ring < rings; ring++) {
        CellId cell_idx(polar, ring, sensor_idx);
        transformPointCloud(at(cell_idx), at(cell_idx), t);
      }
    }
  }
}

int PolarGridOfClouds::computeNewRingIndex(const velodyne_pointcloud::VelodynePoint &point,
                                           const std::vector<float> &borders) {
  float range = compute2DRange(point);
  int new_ring_id;
  for(new_ring_id = rings-2; new_ring_id > 0; new_ring_id--) {
    if(borders[new_ring_id] > range) {
      break;
    }
  }
  return new_ring_id;
}

}
