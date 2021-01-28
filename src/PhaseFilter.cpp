/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 03/08/2020
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

#include <but_velodyne/PhaseFilter.h>

using namespace std;

namespace but_velodyne {

void PhaseFilter::filter(VelodynePointCloud &cloud) const {
  for(VelodynePointCloud::iterator p = cloud.begin(); p < cloud.end();) {
    if(min_phase <= p->phase && p->phase < max_phase) {
      p++;
    } else {
      p = cloud.erase(p);
    }
  }
}

void PhaseFilter::filter(const VelodynePointCloud &input, VelodynePointCloud &slice, VelodynePointCloud &rest) const {
  for(VelodynePointCloud::const_iterator p = input.begin(); p < input.end(); p++) {
    if(min_phase <= p->phase && p->phase < max_phase) {
      slice.push_back(*p);
    } else {
      rest.push_back(*p);
    }
  }
}

void PhaseFilter::filter(const VelodynePointCloud &input, VelodynePointCloud &slice) const {
  for(VelodynePointCloud::const_iterator p = input.begin(); p < input.end(); p++) {
    if(min_phase <= p->phase && p->phase < max_phase) {
      slice.push_back(*p);
    }
  }
}

void PhaseFilter::filter(VelodyneMultiFrame &multiframe) const {
  for(std::vector<VelodynePointCloud::Ptr>::iterator cloud = multiframe.clouds.begin();
      cloud < multiframe.clouds.end(); cloud++) {
    filter(**cloud);
  }
}

void PhaseFilter::filter(const LineCloud &input, LineCloud &slice, vector<int> &indices) const {
  for(int i = 0; i < input.size(); i++) {
    const LineCloud::PointCloudLineWithMiddleAndOrigin &l = input[i];
    if(min_phase <= l.phase && l.phase < max_phase) {
      slice.push_back(l);
      indices.push_back(i);
    }
  }
}

void PhaseFilter::filter(const LineCloud &input, LineCloud &slice) const {
  vector<int> dummy_indices;
  this->filter(input, slice, dummy_indices);
}

}
