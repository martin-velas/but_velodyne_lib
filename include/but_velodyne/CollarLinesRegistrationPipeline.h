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

#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <cstdlib>
#include <cstdio>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <boost/circular_buffer.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/MoveEstimation.h>
#include <but_velodyne/Termination.h>

#include <iostream>
#define BUT_VELODYNE_LOG std::cerr << getpid() << ": "

namespace but_velodyne
{

/**!
 * Previously registered Velodyne frame. The position of each 3D point
 * with respect to the current sensor position is kept.
 */
template<typename Meassurement>
class HistoryRecord {
public:

  /**!
   * @param grid_cloud LiDAR measurement in polar grid structure
   * @param index identifier of measurement
   */
  HistoryRecord(boost::shared_ptr<Meassurement> grid_cloud, const int index) :
    grid_cloud(grid_cloud), transformation(Eigen::Matrix4f::Identity()),
    index(index) {
  }

  /**!
   * Update the position of points when the sensor is moved.
   *
   * @param t_update change of the sensor position
   */
  void update(const Eigen::Matrix4f &t_update) {
    Eigen::Matrix4f inverse = t_update.inverse();
    grid_cloud->transform(inverse);
    transformation *= t_update;
  }

  /**!
   * @returns points in polar grid structure.
   */
  const boost::shared_ptr<Meassurement>& getGridCloud() const
  {
    return grid_cloud;
  }

  /**!
   * @return transformation from previous sensor position (when the measurement was taken)
   * to the current sensor position.
   */
  const Eigen::Matrix4f& getTransformation() const
  {
    return transformation;
  }

  /**!
   * @return identifier
   */
  int getIndex() const
  {
    return index;
  }

private:
  boost::shared_ptr<Meassurement> grid_cloud;
  Eigen::Matrix4f transformation;
  int index;
};

class RegistrationOutcome {
public:
  RegistrationOutcome(void) : term_reason(Termination::NO),
    error(Termination::UNKNOWN_ERROR), error_deviation(Termination::UNKNOWN_ERROR),
    validation_error(Termination::UNKNOWN_ERROR), validation_error_deviation(Termination::UNKNOWN_ERROR) {
  }

  RegistrationOutcome(const Eigen::Affine3f &transformation_, const Termination::Reason term_reason_,
      const float error_, const float error_deviation_ = Termination::UNKNOWN_ERROR,
      const float validation_error_ = Termination::UNKNOWN_ERROR,
      const float validation_error_deviation_ = Termination::UNKNOWN_ERROR)
  : transformation(transformation_), term_reason(term_reason_),
    error(error_), error_deviation(error_deviation_),
    validation_error(validation_error_), validation_error_deviation(validation_error_deviation_) {
  }

  bool operator<(const RegistrationOutcome &other) {
    if(!isnan(this->validation_error) && !isnan(other.validation_error))
      return this->validation_error < other.validation_error;
    else
      return this->error < other.error;
  }

  Eigen::Affine3f transformation;
  Termination::Reason term_reason;
  float error, error_deviation, validation_error, validation_error_deviation;
};

ostream& operator<<(ostream &stream, const RegistrationOutcome &outcome);

/**!
 * Registration of multiple Velodyne LiDAR point clouds (sequence).
 */
class CollarLinesRegistrationPipeline {
public:

  /**!
   * Parameters of registration pipeline
   */
  class Parameters {
  public:
    Parameters(
        int linesPerCellGenerated_ = 20,
        int linesPerCellPreserved_ = 5,
        int historySize_ = 1,
        bool pick_by_lowest_error_ = false,
        int nfolds_ = 1,
        Termination::Parameters term_params_ = Termination::Parameters(),
        bool verbose_ = false)
    :
      linesPerCellGenerated(linesPerCellGenerated_),
      linesPerCellPreserved(linesPerCellPreserved_),
      historySize(historySize_),
      pick_by_lowest_error(pick_by_lowest_error_),
      nfolds(nfolds_),
      term_params(term_params_),
      verbose(verbose_) {
    }

    int linesPerCellGenerated;  /// how many collar lines are generated per polar bin
    int linesPerCellPreserved;  /// how many collar lines are preserved within each bin
    int historySize;                    /// number of previous frames used for multi-view approach
    bool pick_by_lowest_error;
    int nfolds;
    bool verbose;

    Termination::Parameters term_params;

    void prepareForLoading(boost::program_options::options_description &options_desc);

  } pipeline_params;

  /**!
   * @param estimator odometry predictor - Kalman filter or Linear predictor
   * @param graph_file destination of pose graph (in SLAM++ format)
   * @param pipeline_params_ parameters of registration pipeline
   * @param registration_params_ parameters of collar line clouds registration itself
   */
  CollarLinesRegistrationPipeline(MoveEstimator &estimator, ostream &graph_file,
                                  Parameters pipeline_params_, CollarLinesRegistration::Parameters registration_params_) :
    estimation(estimator), cumulated_transformation(Eigen::Matrix4f::Identity()),
    pose_index(0), graph_file(graph_file), history(pipeline_params_.historySize),
    pipeline_params(pipeline_params_), registration_params(registration_params_) {
  }

  /**!
   * Registration of new LiDAR meassurement.
   *
   * @param target_cloud new LiDAR meassurement
   * @param covariance [output] covariance matrix of registration for SLAM++ (how much the algorithm trusts the result)
   */
  Eigen::Matrix4f runRegistration(const VelodynePointCloud &target_cloud,
                                  cv::Mat &covariance);

  Eigen::Matrix4f runRegistration(const std::vector<VelodynePointCloud::Ptr> &target_clouds,
                                  const SensorsCalibration &calibration,
                                  cv::Mat &covariance);

  Eigen::Matrix4f runRegistration(PolarGridOfClouds::Ptr target_polar_grid,
                                  cv::Mat &covariance);

  /**!
   * Prints out the estimation of current sensor pose in KITTI format
   *
   * @param odometry estimation between times when last and current frame were taken
   */
  void output(const Eigen::Matrix4f &transformation);

  void registerTwoGrids(const PolarGridOfClouds &source,
                        const PolarGridOfClouds &target,
                        Eigen::Matrix4f initial_transformation,
                        RegistrationOutcome &output);

  void registerLineClouds(const LineCloud &source_line_cloud,
                          const LineCloud &target_line_cloud,
                          const LineCloud &validation_source_line_cloud,
                          const LineCloud &validation_target_line_cloud,
                          const Eigen::Matrix4f &initial_transformation,
                          Termination &termination, RegistrationOutcome &result);

  const std::vector<CLSMatch>& getLastMatches(void) const {
    return last_matches;
  }

protected:

  Eigen::Matrix4f getPrediction();

  std::vector<Eigen::Matrix4f> runRegistrationEffective(
      const PolarGridOfClouds::Ptr &target_grid_cloud);

  void printInfo(float time, Eigen::Matrix4f t, float error);

  void updateHistory(const PolarGridOfClouds::Ptr target_polar_grid,
                     Eigen::Matrix4f transformation);

  void pickBestByAverage(const vector<Eigen::Matrix4f> &transformations,
                         Eigen::Matrix4f &mean_transformation,
                         cv::Mat &covariance);

private:
  MoveEstimation estimation;
  Eigen::Matrix4f cumulated_transformation;
  Stopwatch stopwatch;

  boost::circular_buffer< HistoryRecord<PolarGridOfClouds> > history;

  int pose_index;
  ostream &graph_file;

  CollarLinesRegistration::Parameters registration_params;

  std::vector<CLSMatch> last_matches;
};

float registerLineClouds(
    const LineCloud &source, const LineCloud &target,
    const LineCloud &validation_source, const LineCloud &validation_target,
    const Eigen::Matrix4f &initial_transformation,
    CollarLinesRegistration::Parameters registration_params,
    CollarLinesRegistrationPipeline::Parameters pipeline_params,
    Eigen::Matrix4f &output_transformation, Termination::Reason &termination_reason);

float registerLineClouds(
    const LineCloud &source_line_cloud, const LineCloud &target_line_cloud,
    const LineCloud &validation_source_line_cloud, const LineCloud &validation_target_line_cloud,
    const Eigen::Matrix4f &initial_transformation,
    const CollarLinesRegistration::Parameters &registration_params,
    const CollarLinesRegistrationPipeline::Parameters &pipeline_params,
    Termination &termination, RegistrationOutcome &result, std::vector<CLSMatch> &matches);

} /* namespace but_velodyne */

#endif /* REGISTRATION_H_ */
