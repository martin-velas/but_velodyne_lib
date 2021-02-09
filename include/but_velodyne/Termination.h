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

#ifndef TERMINATION_H_
#define TERMINATION_H_

#include <boost/circular_buffer.hpp>
#include <boost/program_options.hpp>

#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/CollarLinesValidation.h>

namespace but_velodyne
{

/**!
 * Standard deviation of the registration error.
 */
class ErrorDeviation {
public:

  /**!
   * @param iterations how many iterations should be considered
   */
  ErrorDeviation(int iterations) :
    last_errors(iterations), computed_deviation(-1.0f), computed_deviation_valid(false) {
  }

  /**!
   * Adds the new error value after the iteration of registration algorithm
   *
   * @param error new error value
   */
  void add(float error) {
    last_errors.push_back(error);
    computed_deviation_valid = false;
  }

  /**!
   * @return the standard deviation of the error in last N iterations
   */
  float getDeviation();

  /**!
   * @return true off the error deviation is over the threshold value
   */
  bool isSignificant(float threshold) {
    return (last_errors.size() != last_errors.capacity()) ||
        (getDeviation() > threshold);
  }

protected:

  /**!
   * @return the mean error of last N iterations
   */
  float getMean() const;

private:
  boost::circular_buffer<float> last_errors;
  float computed_deviation;
  bool computed_deviation_valid;
};

/**!
 * Criteria for termination of the iterative algorithm.
 * Algorithm is terminated when one of the following criteria is met:
 *  - algorithm exceeded maximal number of iterations
 *  - algorithm exceeded time resources given
 *  - standard deviation of the error yielded from last N iterations is insignificant
 *  - error from the last iteration is bellow the threshold
 */
class Termination
{
public:

  class Parameters {
  public:
    Parameters(
        int minIterations_ = 20,
        int maxIterations_ = 500,
        float maxTimeSpent_ = 20,  // sec
        int iterationsPerSampling_ = 1000,       // (iterationsPerSampling > maxIterations) causes no resampling
        float targetError_ = 0.01,
        float targetValidationError_ = 0.01,
        float significantErrorDeviation_ = 0.0001,
        float significantValidationErrorDeviation_ = 0.0001)
    :
      targetError(targetError_),
      targetValidationError(targetValidationError_),
      maxTimeSpent(maxTimeSpent_),
      maxIterations(maxIterations_),
      minIterations(minIterations_),
      significantErrorDeviation(significantErrorDeviation_),
      significantValidationErrorDeviation(significantValidationErrorDeviation_),
      iterationsPerSampling(iterationsPerSampling_) {
    }

    float targetError;
    float targetValidationError;
    float maxTimeSpent;
    int maxIterations;
    int minIterations;
    float significantErrorDeviation;
    float significantValidationErrorDeviation;
    int iterationsPerSampling;

    void prepareForLoading(boost::program_options::options_description &options_desc);

    bool doValidation(void) const {
      return (targetValidationError > 0.0) || (significantValidationErrorDeviation > 0.0);
    }

  } term_params;

  typedef enum {
    ERR_DEVIATION,
    ERROR,
    TIME,
    ITERATIONS,
    VALIDATION_ERR_DEVIATION,
    VALIDATION_ERROR,
    NO
  } Reason;

  static std::string reasonToString(const Reason reason) {
    switch(reason) {
    case ERR_DEVIATION:
      return "min_err_deviation";
    case ERROR:
      return "min_error";
    case VALIDATION_ERR_DEVIATION:
      return "min_validation_err_deviation";
    case VALIDATION_ERROR:
      return "min_validation_error";
    case TIME:
      return "max_time_spent";
    case ITERATIONS:
      return "max_iterations";
    default:
      return "no_reason";
    }
  }

  static bool is_converged(const Reason reason) {
    return reason == ERROR || reason == ERR_DEVIATION ||
        reason == VALIDATION_ERROR || reason == VALIDATION_ERR_DEVIATION;
  }

  /**!
   * @param min_iterations minimal number of the iterations
   * @param max_iterations maximum iterations
   * @param max_time_spent maximum time resources for the algorithm
   * @param min_err_deviation minimal standard deviation of the error allowed (computed from multiple iterations of algorithm)
   * @param min_error minimal algorithm error
   */
  Termination(const Parameters &term_params_);

  /**!
   * Add the error from the last algorithm iteration.
   */
  void addNewError(float error, float validation_error = UNKNOWN_ERROR);

  /**!
   * @return true if algorithm should be terminated
   */
  bool operator()();

  Reason why(void) const {
    return reason;
  }

  float getErrorDeviation(void) {
    return err_deviation.getDeviation();
  }

  float getValidationErrorDeviation(void) {
    return validation_err_deviation.getDeviation();
  }

  static const float UNKNOWN_ERROR;

private:
  Stopwatch stopwatch;
  ErrorDeviation err_deviation;
  ErrorDeviation validation_err_deviation;

  float last_error;
  float validation_last_error;
  int iterations;
  Reason reason;
};

ostream& operator<<(ostream &stream, const Termination::Reason &reason);

} /* namespace but_velodyne */

#endif /* TERMINATION_H_ */
