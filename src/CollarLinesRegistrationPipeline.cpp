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

#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/CollarLinesValidation.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesFilter.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/RegistrationCrossValidation.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace velodyne_pointcloud;

namespace po = boost::program_options;

namespace but_velodyne
{

void CollarLinesRegistrationPipeline::Parameters::prepareForLoading(po::options_description &options_desc) {
  options_desc.add_options()
      ("lines_per_bin_generated,g", po::value<int>(&this->linesPerCellGenerated)->default_value(this->linesPerCellGenerated),
          "How many collar lines are generated per single polar bin")
      ("lines_per_bin_preserved,p", po::value<int>(&this->linesPerCellPreserved)->default_value(this->linesPerCellPreserved),
          "How many collar lines are preserved per single polar bin after filtering")
      ("min_iterations", po::value<int>(&this->minIterations)->default_value(this->minIterations),
          "Minimal number of registration iterations (similar to ICP iterations)")
      ("max_iterations", po::value<int>(&this->maxIterations)->default_value(this->maxIterations),
          "Maximal number of registration iterations")
      ("max_time_for_registration", po::value<int>(&this->maxTimeSpent)->default_value(this->maxTimeSpent),
          "Maximal time for registration [sec]")
      ("iterations_per_sampling", po::value<int>(&this->iterationsPerSampling)->default_value(this->iterationsPerSampling),
          "After how many iterations the cloud should be re-sampled by the new collar line segments")
      ("target_error", po::value<float>(&this->targetError)->default_value(this->targetError),
          "Minimal error (average distance of line matches) causing termination of registration")
      ("significant_error_deviation", po::value<float>(&this->significantErrorDeviation)->default_value(this->significantErrorDeviation),
          "If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated")
      ("history_size,m", po::value<int>(&this->historySize)->default_value(this->historySize),
          "How many previous frames are used for registration (multi-view CLS-M approach described in the paper)")
      ("pick_by_lowest_error", po::bool_switch(&this->pick_by_lowest_error),
          "Return solution which yields smallest error (validation error is prefered if available).")
      ("nfolds", po::value<int>(&this->nfolds)->default_value(this->nfolds),
          "Repeat registration multiple times, pick the best error.")
  ;
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::registerTwoGrids(const PolarGridOfClouds &source,
                                               const PolarGridOfClouds &target,
                                               const Eigen::Matrix4f &initial_transformation,
                                               float &final_error) {
  vector<RegistrationOutcome> nfold_outcomes(pipeline_params.nfolds);
  for(int fold = 0; fold < pipeline_params.nfolds; fold++) {
    CollarLinesFilter filter(pipeline_params.linesPerCellPreserved);
    Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations, pipeline_params.maxTimeSpent,
                            pipeline_params.significantErrorDeviation, pipeline_params.targetError);
    while(!termination()) {
      LineCloud source_line_cloud(source, pipeline_params.linesPerCellGenerated, filter);
      LineCloud validation_source_line_cloud(source, pipeline_params.linesPerCellGenerated, filter);
      LineCloud target_line_cloud(target, pipeline_params.linesPerCellGenerated, filter);
      LineCloud validation_target_line_cloud(target, pipeline_params.linesPerCellGenerated, filter);
      nfold_outcomes[fold].transformation = Eigen::Affine3f(registerLineClouds(
          source_line_cloud, target_line_cloud, validation_source_line_cloud, validation_target_line_cloud,
          initial_transformation, termination, nfold_outcomes[fold].error));
      nfold_outcomes[fold].term_reason = termination.why();
    }
  }
  RegistrationCrossValidation validation(nfold_outcomes);
  RegistrationOutcome best_outcome = validation.findBest();
  final_error = best_outcome.error;
  return best_outcome.transformation.matrix();
}

float registerLineClouds(
    const LineCloud &source, const LineCloud &target,
    const Eigen::Matrix4f &initial_transformation,
    CollarLinesRegistration::Parameters registration_params,
    CollarLinesRegistrationPipeline::Parameters pipeline_params,
    Eigen::Matrix4f &output_transformation, Termination::Reason &termination_reason) {

  Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations,
      pipeline_params.maxTimeSpent, pipeline_params.significantErrorDeviation,
      pipeline_params.targetError);
  LineCloud dummy_validation_line_cloud;

  float error = registerLineClouds(source, target,
      dummy_validation_line_cloud, dummy_validation_line_cloud,
      initial_transformation, registration_params, pipeline_params,
      FROM_ORIGIN,
      termination, output_transformation);

  termination_reason = termination.why();
  return error;
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::registerLineClouds(const LineCloud &source_line_cloud,
                                               const LineCloud &target_line_cloud,
                                               const LineCloud &validation_source_line_cloud,
                                               const LineCloud &validation_target_line_cloud,
                                               const Eigen::Matrix4f &initial_transformation,
                                               Termination &termination, float &final_error) {
  Eigen::Matrix4f estimated_transformation;
  final_error = but_velodyne::registerLineClouds(source_line_cloud, target_line_cloud,
      validation_source_line_cloud, validation_target_line_cloud,
      initial_transformation, registration_params, pipeline_params,
      FROM_LAST_POSE,
      termination, estimated_transformation);
  return estimated_transformation;
}

float registerLineClouds(const LineCloud &source_line_cloud, const LineCloud &target_line_cloud,
                         const LineCloud &validation_source_line_cloud, const LineCloud &validation_target_line_cloud,
                         const Eigen::Matrix4f &initial_transformation,
                         const CollarLinesRegistration::Parameters &registration_params,
                         const CollarLinesRegistrationPipeline::Parameters &pipeline_params,
                         const TransformationCumulation cummulation,
                         Termination &termination, Eigen::Matrix4f &out_transformation) {
  float final_error = INFINITY;
  float best_validation_error = INFINITY;

  CollarLinesRegistration cls_fitting(source_line_cloud, target_line_cloud,
                                      registration_params, initial_transformation.matrix());
  CollarLinesValidation cls_validation(validation_source_line_cloud, validation_target_line_cloud,
                                      registration_params);
  for(int sampling_it = 0;
      (sampling_it < pipeline_params.iterationsPerSampling) && !termination();
      sampling_it++) {
    float fitting_error = cls_fitting.refine();
    Eigen::Matrix4f transformation = cls_fitting.getTransformation(cummulation);
    float validation_error = cls_validation.computeError(transformation);
    termination.addNewError(fitting_error, validation_error);
    if(!pipeline_params.pick_by_lowest_error || validation_error < best_validation_error) {
      out_transformation = transformation;
      best_validation_error = validation_error;
      final_error = fitting_error;
    }
  }
  return final_error;
}

vector<Eigen::Matrix4f> CollarLinesRegistrationPipeline::runRegistrationEffective(
    const PolarGridOfClouds::Ptr &target_grid_cloud) {

  assert(!history.empty());
  stopwatch.restart();

  Eigen::Matrix4f transformation = getPrediction();
  vector<Eigen::Matrix4f> results;
  for(int i = 0; i < history.size(); i++) {
    int current_iterations;
    float error;
    transformation = registerTwoGrids(*(history[i].getGridCloud()), *target_grid_cloud,
                                      transformation, error);
    printInfo(stopwatch.elapsed(), transformation, error);
    results.push_back(transformation);

    Eigen::Matrix4f edge_trasform = history[i].getTransformation() * transformation;
    PoseGraphEdge edge(history[i].getIndex(), pose_index, edge_trasform);
    graph_file << edge << endl;
  }

  return results;
}

void CollarLinesRegistrationPipeline::pickBestByAverage(const vector<Eigen::Matrix4f> &transformations,
                         Eigen::Matrix4f &mean_transformation,
                         cv::Mat &covariance) {
  vector<MoveParameters> meassurements;
  for(vector<Eigen::Matrix4f>::const_iterator t = transformations.begin();
        t < transformations.end(); t++) {
    meassurements.push_back(MoveParameters(*t));
  }

  MoveParameters mean(0, 0, 0, 0, 0, 0);
  covariance = mean.setAsAverageFrom(meassurements);

  mean_transformation =  getTransformation(mean.x, mean.y, mean.z,
                                           mean.roll, mean.pitch, mean.yaw).matrix();
}

void CollarLinesRegistrationPipeline::updateHistory(const PolarGridOfClouds::Ptr target_polar_grid,
                                 Eigen::Matrix4f transformation) {
  for(int i = 0; i < history.size(); i++) {
    history[i].update(transformation);
  }
  history.push_front(HistoryRecord<PolarGridOfClouds>(target_polar_grid, pose_index));
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::runRegistration(
    const VelodynePointCloud &target_cloud,
    Mat &covariance) {
  PolarGridOfClouds::Ptr target_polar_grid(new PolarGridOfClouds(target_cloud));
  return runRegistration(target_polar_grid, covariance);
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::runRegistration(
    const std::vector<VelodynePointCloud::Ptr> &target_clouds,
    const SensorsCalibration &calibration,
    cv::Mat &covariance) {
  PolarGridOfClouds::Ptr target_polar_grid(new PolarGridOfClouds(target_clouds, calibration));
  return runRegistration(target_polar_grid, covariance);
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::runRegistration(
    PolarGridOfClouds::Ptr target_polar_grid,
    cv::Mat &covariance) {
  Eigen::Matrix4f transformation;
  if(!history.empty()) {
    pickBestByAverage(runRegistrationEffective(target_polar_grid),
                      transformation, covariance);
    estimation.addMeassurement(transformation);
  } else {
    transformation = Eigen::Matrix4f::Identity();
    covariance = cv::Mat::zeros(6, 6, CV_64FC1);
  }
  updateHistory(target_polar_grid, transformation);
  pose_index++;
  return transformation;
}

void CollarLinesRegistrationPipeline::output(const Eigen::Matrix4f &transformation) {
  cumulated_transformation = cumulated_transformation * transformation;
  KittiUtils::printPose(std::cout, cumulated_transformation);
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::getPrediction() {
  Eigen::Matrix4f prediction = estimation.predict();
  BUT_VELODYNE_LOG << "Prediction:" << std::endl << prediction << std::endl << std::endl;
  return prediction;
}

void CollarLinesRegistrationPipeline::printInfo(float time, Eigen::Matrix4f t, float error) {
  BUT_VELODYNE_LOG << std::endl <<
      "TOTAL" << std::endl <<
      " * time: " << time << "s" << std::endl <<
      " * error: " << error << std::endl <<
      "Transformation:" << std::endl << t << std::endl <<
      "*******************************************************************************" <<
      std::endl << std::flush;
}

} /* namespace but_velodyne */
