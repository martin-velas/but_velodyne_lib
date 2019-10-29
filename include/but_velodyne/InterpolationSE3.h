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

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#ifndef INTERPOLATIONSE3_H_
#define INTERPOLATIONSE3_H_

namespace but_velodyne {

class InterpolationSE3 {
public:
  typedef boost::shared_ptr<InterpolationSE3> Ptr;

  virtual ~InterpolationSE3() {
  }

  virtual Eigen::Affine3f estimate(const float t) const =0;
};

class BSplineSE3 : public InterpolationSE3 {
public:
  BSplineSE3(const std::vector<Eigen::Affine3f> &control_poses_);

  virtual ~BSplineSE3() {
  }

  virtual Eigen::Affine3f estimate(const float t) const;

private:
  static const int DEGREE = 4;

  std::vector<Eigen::Affine3f> control_poses;
  Eigen::Matrix4f C;
  std::vector<Eigen::Matrix4f> Omega;
};

class LinearInterpolationSE3 : public InterpolationSE3 {
public:
  LinearInterpolationSE3(const Eigen::Affine3f &start_, const Eigen::Affine3f &finish_) :
    init(start_), delta(start_.inverse() * finish_) {
  }

  virtual ~LinearInterpolationSE3() {
  }

  virtual Eigen::Affine3f estimate(const float t) const;

private:
  const Eigen::Affine3f init, delta;
};

typedef struct less_float_struct : std::binary_function<float,float,bool> {
  const float TOLERANCE = 1e-6;
  bool operator()(const float f1, const float f2) const
  {
      return f1 < f2 && std::abs(f2 - f1) >= TOLERANCE;
  }
} less_float;

class InterpolationBuffered {
public:
  typedef std::map<float, Eigen::Affine3f, less_float> map_type;
  typedef boost::shared_ptr<InterpolationBuffered> Ptr;

  InterpolationBuffered(const InterpolationSE3::Ptr &method_,
      const float phase_offset_, const float phase_scale_) :
    method(method_), phase_offset(phase_offset_), phase_scale(phase_scale_) {
  }

  Eigen::Affine3f estimate(const float t);

private:
  const InterpolationSE3::Ptr method;
  const float phase_offset, phase_scale;
  map_type interpolated_poses;
};

Eigen::Affine3f average_transformations(const std::vector<Eigen::Affine3f> &transformations);

}

#endif /* INTERPOLATIONSE3_H_ */
