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

#ifndef ITERATIVELINEPLANEFITTING_H_
#define ITERATIVELINEPLANEFITTING_H_

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/program_options.hpp>

#include <but_velodyne/LineCloud.h>
#include <but_velodyne/Visualizer3D.h>

#include <gtest/gtest.h>

namespace but_velodyne
{

class CLSMatch {
public:
  CLSMatch(const CLS &source_line_,
           const CLS &target_line_,
           const Eigen::Vector3f &src_pt_,
           const Eigen::Vector3f &trg_pt_,
           const float source_q_, const float target_q_) :
    source_line(source_line_), target_line(target_line_) {
    src_pt.getVector3fMap() = src_pt_;
    trg_pt.getVector3fMap() = trg_pt_;
    distance = (src_pt_ - trg_pt_).norm();
  }

  bool isValid(void) const {
    return !isnan(source_line.phase) && !isnan(target_line.phase);
  }

  bool operator< (const CLSMatch &other) const {
    return this->distance < other.distance;
  }

  const CLS &getSourceLine() const {
    return source_line;
  }

  const CLS &getTargetLine() const {
    return target_line;
  }

  const pcl::PointXYZ &getSrcPt() const {
    return src_pt;
  }

  const pcl::PointXYZ &getTrgPt() const {
    return trg_pt;
  }

  float getDistance() const {
    return distance;
  }

  float getSrcQ() const {
    return src_q;
  }

  float getTrgQ() const {
    return trg_q;
  }

  void transformTarget(const Eigen::Affine3f &T);

private:

  CLS source_line, target_line;
  pcl::PointXYZ src_pt, trg_pt;
  float distance;
  float src_q, trg_q;
};

struct CLSMatchByCoeffComparator {
    bool operator ()(const CLSMatch &m1, const CLSMatch &m2) {
      return MAX(fabs(m1.getSrcQ()), fabs(m1.getTrgQ())) < MAX(fabs(m2.getSrcQ()), fabs(m2.getTrgQ()));
    }
};

/**!
 * Registration of two collar line clouds.
 */
class CollarLinesRegistration
{
  typedef Eigen::Vector3f TPoint3D;
  typedef Eigen::Matrix<TPoint3D::Scalar, TPoint3D::RowsAtCompileTime, Eigen::Dynamic> MatrixOfPoints;
  typedef Eigen::DiagonalMatrix<TPoint3D::Scalar, Eigen::Dynamic, Eigen::Dynamic> WeightsMatrix;
public:

  /**!
   * How the weights are assigned to the collar line matches.
   */
  enum Weights {
    RANGE_WEIGHTS,           // matches of close lines are more significant
    VERTICAL_ANGLE_WEIGHTS,     // matches of vertical lines are more significant
    NO_WEIGHTS                  // all line matches are equal
  };

  friend std::istream& operator>> (std::istream &in, Weights &weightning);

  /**!
   * How the threshold for line matches filtering is estimated.
   */
  enum Threshold {
      MEDIAN_THRESHOLD,         // all matches with distance above median are discarded
      QUARTER_THRESHOLD,
      TENTH_THRESHOLD,
      PERC_99_THRESHOLD,
      PERC_90_THRESHOLD,
      VALUE_THRESHOLD,
      PORTION_VALUE_THRESHOLD,
      MEAN_THRESHOLD,           // threshold = mean
      NO_THRESHOLD              // no thresholding - all matches are preserved
  };

  friend std::istream& operator>> (std::istream &in, Threshold &thresholding);

  /**!
   * Options of registration.
   */
  class Parameters
  {
  public:
    Parameters(
        Threshold distance_threshold_ = MEDIAN_THRESHOLD,
        float distance_threshold_decay_ = 1.0f,
        float distance_threshold_value_ = NAN,
        Weights weighting_ = NO_WEIGHTS,
        int nearestNeighbors_ = 1,
        bool estimate_translation_only_ = false,
        bool dont_estimate_roll_pitch_ = false,
        bool separate_sensors_ = false,
        float phase_weights_max_ = -1.0,
        float phase_weights_power_ = 4.0,
        bool visualize_cls_weights_ = false,
        bool visualize_cls_correspondences_ = false,
        bool verbose_ = false) :
        distance_threshold(distance_threshold_),
        distance_threshold_decay(distance_threshold_decay_),
        distance_threshold_value(distance_threshold_value_),
        weighting(weighting_),
        nearestNeighbors(nearestNeighbors_),
        estimate_translation_only(estimate_translation_only_),
        dont_estimate_roll_pitch(dont_estimate_roll_pitch_),
        separate_sensors(separate_sensors_),
        phase_weights_max(phase_weights_max_),
        phase_weights_power(phase_weights_power_),
        visualize_cls_weights(visualize_cls_weights_),
        visualize_cls_correspondences(visualize_cls_correspondences_),
        verbose(verbose_) {
    }
    Threshold distance_threshold;       /// how is the threshold of line matches distance estimated
    float distance_threshold_decay;
    float distance_threshold_value;
    Weights weighting;                  /// optional weighting of line matches
    int nearestNeighbors;
    bool estimate_translation_only;
    bool dont_estimate_roll_pitch;
    bool separate_sensors;
    float phase_weights_max;
    float phase_weights_power;
    bool visualize_cls_weights;
    bool visualize_cls_correspondences;
    bool verbose;

    void prepareForLoading(boost::program_options::options_description &options_desc);

    Parameters discardDebug(void) const {
      Parameters noDebug = *this;
      noDebug.visualize_cls_weights = false;
      noDebug.visualize_cls_correspondences = false;
      noDebug.verbose = false;
      return noDebug;
    }

  } params;

  /**!
   * @param source_cloud_ the line cloud from time T
   * @param target_cloud_ the line cloud from time T+1
   * @param params_ parameters of registration
   * @param initial_transformation_ predicted transformation for initialization
   */
  CollarLinesRegistration(const LineCloud &source_cloud_,
                            const LineCloud &target_cloud_,
                            const Parameters params_,
                            const Eigen::Matrix4f initial_transformation_ = Eigen::Matrix4f::Identity()) :
    source_cloud(source_cloud_), target_cloud(target_cloud_),
    initial_transformation(initial_transformation_),
    params(params_),
    transformation(Eigen::Matrix4f::Identity()),
    matching_time(0), correnspondences_time(0), tranformation_time(0), error_time(0),
    refinements_done(0) {

    source_kdtree.setInputCloud(source_cloud.getMiddles());
    if(params.separate_sensors) {
      fillKdtreesBySensors();
    }
    this->target_cloud.transform(initial_transformation);
  }

  CollarLinesRegistration(const LineCloud &source_cloud_,
                          pcl::KdTreeFLANN<pcl::PointXYZ> &source_kdtree_,
                          const LineCloud &target_cloud_,
                          const Parameters params_,
                          const Eigen::Matrix4f initial_transformation_ = Eigen::Matrix4f::Identity()) :
    source_cloud(source_cloud_), target_cloud(target_cloud_),
    initial_transformation(initial_transformation_),
    params(params_),
    source_kdtree(source_kdtree_),
    transformation(Eigen::Matrix4f::Identity()),
    matching_time(0), correnspondences_time(0), tranformation_time(0), error_time(0),
    refinements_done(0) {

    if(params.separate_sensors) {
      fillKdtreesBySensors();
    }
    this->target_cloud.transform(initial_transformation);
  }

  /**!
   * Run iteration of the registration process.
   *
   * @return average distance (error) of the matching lines
   */
  float refine();

  /**!
   * @return matches of collar lines between source (train indices) and target (queries) cloud
   */
  std::vector<cv::DMatch> getMatches() {
    return matches;
  }

  /**!
   * @return transformation estimated so far
   */
  const Eigen::Matrix4f getTransformation(void) const;

  /**!
   * @return refined transformation
   */
  const Eigen::Matrix4f getRefinedTransformation() const
  {
    return transformation;
  }

  /**!
   * Visualize line correspondences found in last iteration
   */
  void showLinesCorrenspondences();

  /**!
   * @return error (average distance of matching lines) of the last iteration
   */
  float computeError(void);

  const void getLastMatches(std::vector<CLSMatch> &out_matches) const;

  // time counters measuring how much the each step of registration process costs
  float matching_time, correnspondences_time, tranformation_time, error_time;

  float getPhaseWeight(const float phase) const;

  float getPhaseWeight(const float source_phase, const float target_phase) const;

  int getMatches(const int target_index, vector<int> &closest_index, vector<float> &min_distance) const;

  const vector<cv::DMatch> &getMatches() const {
    return matches;
  }

protected:

  float getEffectiveThreshold(void) const;

  void findClosestMatchesByMiddles(void);

  void getCorrespondingPoints(MatrixOfPoints &source_coresp_points,
                              MatrixOfPoints &target_coresp_points);

  void getWeightingMatrix(WeightsMatrix &weightingMatrix) const;

  Eigen::Matrix4f computeTransformationWeighted(const MatrixOfPoints &source_coresp_points,
                             const MatrixOfPoints &target_coresp_points);

  float computeError(const MatrixOfPoints &source_coresp_points,
                     const MatrixOfPoints &target_coresp_points,
                     const Eigen::Matrix4f &transformation) const;

  void filterMatchesByThreshold(const float threshold);

  float getVerticalWeight(const Eigen::Vector3f &source_line_orient,
                          const Eigen::Vector3f &target_line_orient);

  float sinOfAngleWithGround(const Eigen::Vector3f &orientation);

  float thresholdTypeToFraction(void) const;

  float getEffectiveDecay(void) const;

  float getMatchesDistanceThreshold(float ratio) const;

  float getMatchesMean(void) const;

  void fillKdtreesBySensors(void);

private:
  const LineCloud &source_cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> source_kdtree;
  vector< pcl::KdTreeFLANN<pcl::PointXYZ> > source_kdtrees_by_sensor;
  vector< std::vector<int> > source_cloud_indices_by_sensor;
  LineCloud target_cloud;
  std::vector<cv::DMatch> matches;
  std::vector<cv::DMatch> rejected_matches;
  const Eigen::Matrix4f initial_transformation;
  Eigen::Matrix4f last_refinement;
  Eigen::Matrix4f transformation;
  Eigen::VectorXf correspondences_weights;
  int refinements_done;
  std::vector<CLSMatch> last_point_matches;

  FRIEND_TEST(CollarLinesRegistration, getWeightingMatrixTest);
  FRIEND_TEST(CollarLinesRegistration, computeErrorTest);
  FRIEND_TEST(CollarLinesRegistration, findClosestMatchesByMiddlesTest);
  FRIEND_TEST(CollarLinesRegistration, getMatchesTest);
  FRIEND_TEST(CollarLinesRegistration, refineTest);
  FRIEND_TEST(CollarLinesRegistration, getEffectiveThresholdTest);
};

} /* namespace but_velodyne */

#endif /* ITERATIVELINEPLANEFITTING_H_ */
