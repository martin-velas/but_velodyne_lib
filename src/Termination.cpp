/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/05/2015
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

#include <cmath>
#include <boost/range/numeric.hpp>

#include <but_velodyne/Termination.h>

namespace but_velodyne
{

Termination::Termination(int min_iterations, int max_iterations, float max_time_spent,
            float min_err_deviation, float min_error) :
              err_deviation(min_iterations), validation_err_deviation(min_iterations),
              min_error(min_error), max_time_spent(max_time_spent), max_iterations(max_iterations),
              min_iterations(min_iterations), iterations(0), min_err_deviation(min_err_deviation), reason(NO) {
  stopwatch.restart();
  last_error = INFINITY;
  validation_last_error = INFINITY;
}

void Termination::addNewError(float new_error, float validation_error) {
  last_error = new_error;
  err_deviation.add(new_error);
  validation_last_error = validation_error;
  validation_err_deviation.add(validation_error);
  iterations++;
}

bool Termination::operator()() {

  if (iterations < min_iterations) {
    reason = NO;
  } else if(iterations >= max_iterations) {
    reason = ITERATIONS;
  } else if(stopwatch.elapsed() > max_time_spent) {
    reason = TIME;
  } else if(!err_deviation.isSignificant(min_err_deviation)) {
    reason = ERR_DEVIATION;
  } else if(last_error < min_error) {
    reason = ERROR;
  } else {
    if(!isinf(validation_last_error)) {
      if(!validation_err_deviation.isSignificant(min_err_deviation)) {
        reason = VALIDATION_ERR_DEVIATION;
      } else if(validation_last_error < min_error) {
        reason = VALIDATION_ERROR;
      } else {
        reason = NO;
      }
    } else {
      reason = NO;
    }
  }

  if(reason != NO) {
    std::cerr << "Termination after " << stopwatch.elapsed()
        << "[sec], reason: " << reasonToString(reason) <<". Iterations: " << iterations
        << ", err_deviation: " << err_deviation.getDeviation()
        << ", last_error: " << last_error;
    if(!isinf(validation_last_error)) {
      std::cerr << ", validation_err_deviation: " << validation_err_deviation.getDeviation()
          << ", validation_last_error: " << validation_last_error;
    }
    std::cerr << std::endl;
  }

  return reason != NO;
}

float ErrorDeviation::getDeviation() const {
  float sum_squares = 0;
  float mean = getMean();
  for(boost::circular_buffer<float>::const_iterator e = last_errors.begin();
      e < last_errors.end(); e++) {
    sum_squares += pow(*e - mean, 2);
  }
  return sqrt(sum_squares / last_errors.size());
}

float ErrorDeviation::getMean() const {
  return std::accumulate(last_errors.begin(), last_errors.end(), 0.0) / last_errors.size();
}

} /* namespace but_velodyne */
