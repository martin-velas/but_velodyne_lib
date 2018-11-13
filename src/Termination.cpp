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

namespace po = boost::program_options;

namespace but_velodyne
{

const float Termination::UNKNOWN_ERROR = INFINITY;

void Termination::Parameters::prepareForLoading(po::options_description &options_desc) {
  options_desc.add_options()
    ("min_iterations", po::value<int>(&this->minIterations)->default_value(this->minIterations),
        "Minimal number of registration iterations (similar to ICP iterations)")
    ("max_iterations", po::value<int>(&this->maxIterations)->default_value(this->maxIterations),
        "Maximal number of registration iterations")
    ("max_time_for_registration", po::value<float>(&this->maxTimeSpent)->default_value(this->maxTimeSpent),
        "Maximal time for registration [sec]")
    ("iterations_per_sampling", po::value<int>(&this->iterationsPerSampling)->default_value(this->iterationsPerSampling),
        "After how many iterations the cloud should be re-sampled by the new collar line segments")
    ("target_error", po::value<float>(&this->targetError)->default_value(this->targetError),
        "Minimal error (average distance of line matches) causing termination of registration")
    ("significant_error_deviation", po::value<float>(&this->significantErrorDeviation)->default_value(this->significantErrorDeviation),
        "If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated")
    ("target_validation_error", po::value<float>(&this->targetValidationError)->default_value(this->targetValidationError),
        "Minimal validation error (average distance of line matches) causing termination of registration")
    ("significant_validation_error_deviation", po::value<float>(&this->significantValidationErrorDeviation)->default_value(this->significantValidationErrorDeviation),
        "If standard deviation of validation error from last N=min_iterations iterations is below this value - registration is terminated")
  ;
}

Termination::Termination(const Parameters &term_params_) :
              term_params(term_params_),
              err_deviation(term_params_.minIterations), validation_err_deviation(term_params_.minIterations),
              iterations(0), reason(NO) {
  stopwatch.restart();
  last_error = UNKNOWN_ERROR;
  validation_last_error = UNKNOWN_ERROR;
}

void Termination::addNewError(float new_error, float validation_error) {
  last_error = new_error;
  err_deviation.add(new_error);
  validation_last_error = validation_error;
  validation_err_deviation.add(validation_error);
  iterations++;
}

bool Termination::operator()() {

  if (iterations < term_params.minIterations) {
    reason = NO;
  } else if(iterations >= term_params.maxIterations) {
    reason = ITERATIONS;
  } else if(stopwatch.elapsed() > term_params.maxTimeSpent) {
    reason = TIME;
  } else if(!err_deviation.isSignificant(term_params.significantErrorDeviation)) {
    reason = ERR_DEVIATION;
  } else if(last_error < term_params.targetError) {
    reason = ERROR;
  } else {
    if(!isinf(validation_last_error)) {
      if(!validation_err_deviation.isSignificant(term_params.significantValidationErrorDeviation)) {
        reason = VALIDATION_ERR_DEVIATION;
      } else if(validation_last_error < term_params.targetValidationError) {
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

float ErrorDeviation::getDeviation() {
  if(computed_deviation_valid) {
    return computed_deviation;
  }
  float sum_squares = 0;
  float mean = getMean();
  for(boost::circular_buffer<float>::const_iterator e = last_errors.begin();
      e < last_errors.end(); e++) {
    sum_squares += pow(*e - mean, 2);
  }
  computed_deviation = sqrt(sum_squares / last_errors.size());
  computed_deviation_valid = true;
  return computed_deviation;
}

float ErrorDeviation::getMean() const {
  return std::accumulate(last_errors.begin(), last_errors.end(), 0.0) / last_errors.size();
}

ostream& operator<<(ostream &stream, const Termination::Reason &reason) {
  return (stream << Termination::reasonToString(reason));
}

} /* namespace but_velodyne */
