/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/02/2016
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

#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <iostream>
#include <map>
#include <cmath>
#include <stdexcept>

namespace but_velodyne {

float radToDeg(float rad);

float degToRad(float deg);

class VelodyneSpecification {

public:

  typedef enum {
    VLP16, HDL32, HDL64, Unknown
  } Model;

  /**
   * Angles in degrees, O = sensor.
   *
   *    /^ maximal angle
   *   /
   *  /
   * O--------> horizontal line (90°)
   * |\
   * | \
   * |  \
   * |   v minimal angle
   * V
   * (0°)
   */

  static int rings(const Model model) {
    switch(model) {
    case VLP16:
      return 16;
    case HDL32:
      return 32;
    case HDL64:
      return 64;
    default:
      throw std::invalid_argument("Unknown Velodyne model (ring estimation)");
    }
  }

  static float minAngle(const Model model) {
    switch(model) {
    case VLP16:
      return 75.00;
    case HDL32:
      return 59.33;
    case HDL64:
      return 62.13;
    default:
      throw std::invalid_argument("Unknown Velodyne model (min angle estimation)");
    }
  }

  static float maxAngle(const Model model) {
    switch(model) {
    case VLP16:
      return 105.00;
    case HDL32:
      return 100.67;
    case HDL64:
      return 92.00;
    default:
      throw std::invalid_argument("Unknown Velodyne model (max angle estimation)");
    }
  }

  static const float KITTI_HEIGHT;


  /**!
   * O sensor
   * | \
   * |  \ laser ray
   * |   \
   * |    \
   * |-----x point
   * |     |
   *  Range
   *
   * @param ring ID (rings are indexed from furthest to nearest)
   * @param height how height is Velodyne positioned
   * @return expected horizontal range of points from given ring
   */
  static float getExpectedRange(const int ring, const Model model, const int height = KITTI_HEIGHT) {
    int n = rings(model) - ring - 1;       // invert indexing (0=nearest)
    static std::map<int, float> ranges;
    static Model last_model = model;
    if(last_model != model) {
      ranges.clear();
    }
    if(ranges.empty()) {
      float delta = (maxAngle(model) - minAngle(model))/(rings(model)-1);
      for(int n = 0; n < rings(model); n++) {
        float angle = minAngle(model) + n*delta;
        ranges[n] = (angle < 90.0f) ? (tan(degToRad(angle))*height) : INFINITY;
      }
    }
    return ranges[n];
  }
};

}

#endif /* VELODYNEPOINTCLOUD_H_ */
