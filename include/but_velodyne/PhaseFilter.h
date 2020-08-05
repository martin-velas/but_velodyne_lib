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
#ifndef BUT_VELODYNE_LIB_PHASEFILTER_H
#define BUT_VELODYNE_LIB_PHASEFILTER_H

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>

namespace but_velodyne {

class PhaseFilter {

public:
    PhaseFilter(const float min_phase_, const float max_phase_) :
            min_phase(min_phase_), max_phase(max_phase_) {
    }

    void filter(VelodynePointCloud &cloud) const;

    void filter(const VelodynePointCloud &input, VelodynePointCloud &slice, VelodynePointCloud &rest) const;

    void filter(const VelodynePointCloud &input, VelodynePointCloud &slice) const;

    void filter(VelodyneMultiFrame &multiframe) const;

    float getMaxPhase() const {
      return max_phase;
    }

    float getMinPhase() const {
      return min_phase;
    }

private:
    float min_phase;
    float max_phase;
};

}

#endif //BUT_VELODYNE_LIB_PHASEFILTER_H
