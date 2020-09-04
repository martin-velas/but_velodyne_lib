/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2014
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

#include <numeric>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/program_options/errors.hpp>

#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/EigenUtils.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace pcl;
using namespace boost;

namespace po = boost::program_options;

namespace but_velodyne
{

void CLSMatch::transformTarget(const Eigen::Affine3f &T) {
  this->trg_pt = transformPoint(this->trg_pt, T);
  this->target_line = this->target_line.transform(T);
}

std::istream& operator>> (std::istream &in, CollarLinesRegistration::Weights &weightning) {
  string token;
  in >> token;

  boost::to_upper(token);
  if (token == "RANGE_WEIGHTS") {
    weightning = CollarLinesRegistration::RANGE_WEIGHTS;
  } else if (token == "VERTICAL_ANGLE_WEIGHTS") {
    weightning = CollarLinesRegistration::VERTICAL_ANGLE_WEIGHTS;
  } else if (token == "NO_WEIGHTS") {
    weightning = CollarLinesRegistration::NO_WEIGHTS;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }
  return in;
}

std::istream& operator>> (std::istream &in, CollarLinesRegistration::Threshold &thresholding) {
  string token;
  in >> token;

  boost::to_upper(token);
  if (token == "MEDIAN_THRESHOLD") {
    thresholding = CollarLinesRegistration::MEDIAN_THRESHOLD;
  } else if (token == "MEAN_THRESHOLD") {
    thresholding = CollarLinesRegistration::MEAN_THRESHOLD;
  } else if (token == "NO_THRESHOLD") {
    thresholding = CollarLinesRegistration::NO_THRESHOLD;
  } else if (token == "QUARTER_THRESHOLD") {
    thresholding = CollarLinesRegistration::QUARTER_THRESHOLD;
  } else if (token == "TENTH_THRESHOLD") {
    thresholding = CollarLinesRegistration::TENTH_THRESHOLD;
  } else if (token == "VALUE_THRESHOLD") {
    thresholding = CollarLinesRegistration::VALUE_THRESHOLD;
  } else if (token == "PORTION_VALUE_THRESHOLD") {
    thresholding = CollarLinesRegistration::PORTION_VALUE_THRESHOLD;
  } else if (token == "TENTH_THRESHOLD") {
    thresholding = CollarLinesRegistration::TENTH_THRESHOLD;
  } else if (token == "PERC_99_THRESHOLD") {
    thresholding = CollarLinesRegistration::PERC_99_THRESHOLD;
  } else if (token == "PERC_90_THRESHOLD") {
    thresholding = CollarLinesRegistration::PERC_90_THRESHOLD;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }
  return in;
}

void CollarLinesRegistration::Parameters::prepareForLoading(po::options_description &options_desc) {
  options_desc.add_options()
      ("matching_threshold", po::value<CollarLinesRegistration::Threshold>(&this->distance_threshold)->default_value(this->distance_threshold),
          "How the value of line matching threshold is estimated (mean/median/... of line pairs distance). Possible values: MEDIAN_THRESHOLD|MEAN_THRESHOLD|PERC_99_THRESHOLD|PERC_90_THRESHOLD|QUARTER_THRESHOLD|TENTH_THRESHOLD|VALUE_THRESHOLD|PORTION_VALUE_THRESHOLD|NO_THRESHOLD")
      ("matching_threshold_value", po::value<float>(&this->distance_threshold_value)->default_value(this->distance_threshold_value),
          "Value of mathing lines threshold in case of VALUE_THRESHOLD or PORTION_VALUE_THRESHOLD option is used")
      ("matching_threshold_decay", po::value<float>(&this->distance_threshold_decay)->default_value(this->distance_threshold_decay),
          "How the amount of matching lines is decaying after each iteration (portion*=decay).")
      ("line_weightning", po::value<CollarLinesRegistration::Weights>(&this->weighting)->default_value(this->weighting),
          "How the weights are assigned to the line matches - prefer vertical lines, close or treat matches as equal. Possible values: RANGE_WEIGHTS|VERTICAL_ANGLE_WEIGHTS|NO_WEIGHTS")
      ("translation_only", po::bool_switch(&this->estimate_translation_only),
          "Estimate only the translation (rotation should be presented as the initial pose)")
      ("no_roll_pitch", po::bool_switch(&this->dont_estimate_roll_pitch),
          "Do not estimate the roll and pitch angles (only the heading and the translation)")
      ("nearest_neighbors", po::value<int>(&this->nearestNeighbors)->default_value(this->nearestNeighbors),
        "How many nearest neighbors (matches) are found for each line of source frame.")
      ("separate_sensors", po::bool_switch(&this->separate_sensors),
          "Do not match lines across the sensors.")
      ("phase_weights_max", po::value<float>(&this->phase_weights_max)->default_value(this->phase_weights_max),
          "The matches are weighted by the phase (which phase is considered most significant)")
      ("phase_weights_power", po::value<float>(&this->phase_weights_power)->default_value(this->phase_weights_power),
         "Power of the phase difference. weight = diff^power;")
      ("visualize_cls_weights", po::bool_switch(&this->visualize_cls_weights),
         "Visualize weights for CLS registration.")
      ("visualize_cls_correspondences", po::bool_switch(&this->visualize_cls_correspondences),
         "Visualize CLS correspondences.")
  ;
}

float CollarLinesRegistration::refine() {
  Stopwatch watch;

  watch.restart();
  findClosestMatchesByMiddles();
  matching_time += watch.elapsed();

  watch.restart();
  MatrixOfPoints source_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size());
  MatrixOfPoints target_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size());
  getCorrespondingPoints(source_coresp_points, target_coresp_points);
  correnspondences_time += watch.elapsed();

  watch.restart();
  last_refinement = computeTransformationWeighted(source_coresp_points, target_coresp_points);
  target_cloud.transform(last_refinement);
  transformation = last_refinement * transformation;
  tranformation_time += watch.elapsed();

  watch.restart();
  float error = computeError(source_coresp_points, target_coresp_points, last_refinement);
  error_time += watch.elapsed();

  refinements_done++;

  return error;
}

float CollarLinesRegistration::computeError() {
  findClosestMatchesByMiddles();

  MatrixOfPoints source_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size());
  MatrixOfPoints target_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size());
  getCorrespondingPoints(source_coresp_points, target_coresp_points);

  float error = computeError(source_coresp_points, target_coresp_points,
                             Eigen::Matrix4f::Identity());
  return error;
}

float CollarLinesRegistration::computeError (
    const MatrixOfPoints &source_coresp_points,
    const MatrixOfPoints &target_coresp_points,
    const Matrix4f &transformation) const {
  const int matches_cnt = matches.size();

  typedef Eigen::Matrix<TPoint3D::Scalar, TPoint3D::RowsAtCompileTime+1, Eigen::Dynamic> MatrixOfHomogeniousPoints;
  MatrixOfHomogeniousPoints target_points_transformed =
      MatrixOfHomogeniousPoints::Ones(TPoint3D::RowsAtCompileTime+1, matches_cnt);
  target_points_transformed.block(0, 0, 3, matches_cnt) = target_coresp_points;
  target_points_transformed = transformation * target_points_transformed;
  MatrixOfPoints difference = source_coresp_points - target_points_transformed.block(0, 0, 3, matches_cnt);
  VectorXf square_distances = difference.cwiseProduct(difference).transpose() * Vector3f::Ones();
  VectorXf distances = square_distances.cwiseSqrt();

  WeightsMatrix weights;
  getWeightingMatrix(weights);
  VectorXf distances_weighted = weights * distances;

  float error = distances_weighted.sum();
  return error;
}

const void CollarLinesRegistration::getLastMatches(std::vector<CLSMatch> &out_matches) const {
  out_matches = last_point_matches;
  Eigen::Affine3f untransform(transformation * initial_transformation);
  untransform = untransform.inverse();
  for(vector<CLSMatch>::iterator m = out_matches.begin(); m < out_matches.end(); m++) {
    m->transformTarget(untransform);
  }
}

void CollarLinesRegistration::fillKdtreesBySensors(void) {
  int sensors_cnt = -1;
  for(LineCloud::const_iterator l = source_cloud.begin(); l < source_cloud.end(); l++) {
    sensors_cnt = MAX(sensors_cnt, l->sensor_id);
  }
  sensors_cnt++;

  vector< PointCloud<PointXYZ>::Ptr > sensors_middles(sensors_cnt);
  for(int sensor_i = 0; sensor_i < sensors_cnt; sensor_i++) {
    sensors_middles[sensor_i].reset(new PointCloud<PointXYZ>);
  }

  source_cloud_indices_by_sensor.resize(sensors_cnt);
  for(int i = 0; i < source_cloud.size(); i++) {
    const LineCloud::PointCloudLineWithMiddleAndOrigin l = source_cloud[i];
    sensors_middles[l.sensor_id]->push_back(l.middle);
    source_cloud_indices_by_sensor[l.sensor_id].push_back(i);
  }

  source_kdtrees_by_sensor.resize(sensors_cnt);
  for(int sensor_i = 0; sensor_i < sensors_cnt; sensor_i++) {
    source_kdtrees_by_sensor[sensor_i].setInputCloud(sensors_middles[sensor_i]);
  }
}

int CollarLinesRegistration::getMatches(const int target_index,
    vector<int> &closest_index, vector<float> &min_distance) const {

  const PointXYZ &target_line_middle = target_cloud[target_index].middle;
  const int target_sensor = target_cloud[target_index].sensor_id;

  const int K = params.nearestNeighbors;
  closest_index.resize(K);
  min_distance.resize(K);

  int matches_count;
  if(params.separate_sensors) {
    matches_count = source_kdtrees_by_sensor[target_sensor].nearestKSearch(target_line_middle, K, closest_index, min_distance);
    for(int i = 0; i < matches_count; i++) {
      closest_index[i] = source_cloud_indices_by_sensor[target_sensor][closest_index[i]];
    }
  } else {
    matches_count = source_kdtree.nearestKSearch(target_line_middle, K, closest_index, min_distance);
  }

  assert(matches_count == K);
  return matches_count;
}

float CollarLinesRegistration::getEffectiveThreshold(void) const {
  if(params.distance_threshold == MEAN_THRESHOLD) {
    return getMatchesMean();
  } else if(params.distance_threshold == VALUE_THRESHOLD) {
    assert(!isnan(params.distance_threshold_value));
    if(isnan(params.distance_threshold_value)) {
      cerr << "Warning: distance_threshold_value was not set!" << endl;
    }
    return params.distance_threshold_value;
  } else if (params.distance_threshold == NO_THRESHOLD) {
    return INFINITY;
  } else {
    return getMatchesDistanceThreshold(thresholdTypeToFraction()*getEffectiveDecay());
  }
}

void CollarLinesRegistration::findClosestMatchesByMiddles(void) {
  matches.clear();

  for(int target_index = 0; target_index < target_cloud.size(); target_index++) {
    vector<int> closest_index;
    vector<float> min_distance;
    int matches_count = getMatches(target_index, closest_index, min_distance);

    for(int i = 0; i < matches_count; i++) {
      const int source_index = closest_index[i];
      float distance = min_distance[i];

      if(params.phase_weights_max > -0.0001) {
        const float source_phase = source_cloud[source_index].phase;
        const float target_phase = target_cloud[target_index].phase;
        const float phase_dist = MAX(1.0 - getPhaseWeight(source_phase, target_phase), 0.0);
        distance *= phase_dist;
      }

      DMatch match(target_index, source_index, distance);
      matches.push_back(match);
    }
  }

  if(params.verbose) {
    cerr << "Matches found: " << matches.size() << endl;
  }

  float effective_threshold = getEffectiveThreshold();
  filterMatchesByThreshold(effective_threshold);

  if(params.verbose) {
    cerr << "Matches filtered: " << matches.size() << endl;
  }
}

void CollarLinesRegistration::filterMatchesByThreshold(const float threshold) {
  rejected_matches.clear();
  for(vector<DMatch>::iterator m = matches.begin(); m < matches.end();) {
    if(m->distance < threshold) {
      m++;
    } else {
      m = matches.erase(m);
      rejected_matches.push_back(*m);
    }
  }
}

float CollarLinesRegistration::thresholdTypeToFraction(void) const {
  switch(params.distance_threshold) {
  case MEDIAN_THRESHOLD:
    return 0.5;
  case QUARTER_THRESHOLD:
    return 0.25;
  case TENTH_THRESHOLD:
    return 0.1;
  case PERC_99_THRESHOLD:
    return 0.99;
  case PERC_90_THRESHOLD:
    return 0.90;
  case PORTION_VALUE_THRESHOLD:
    return params.distance_threshold_value;
  default:
    cerr << "Warning: uknown distance threshold type!" << endl;
    return 0.0;
  }
}

float CollarLinesRegistration::getEffectiveDecay(void) const {
  return pow(params.distance_threshold_decay, refinements_done);
}

float CollarLinesRegistration::getMatchesDistanceThreshold(float ratio) const {
  vector<float> acc(matches.size());
  for(int i = 0; i < matches.size(); i++) {
    acc[i] = matches[i].distance;
  }
  std::sort(acc.begin(), acc.end());
  return acc[acc.size()*ratio];
}

float CollarLinesRegistration::getMatchesMean(void) const {
  float sum = 0;
  for(vector<DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    sum += m->distance;
  }
  return sum / matches.size();
}

float CollarLinesRegistration::getPhaseWeight(const float phase) const {
  return pow(1.0 - fabs(phase - params.phase_weights_max), params.phase_weights_power);
}

float CollarLinesRegistration::getPhaseWeight(const float source_phase, const float target_phase) const {
  if(isnan(source_phase) || isnan(target_phase)) {
    return 0.0;
  } else {
    return MAX(getPhaseWeight(source_phase), getPhaseWeight(target_phase));
  }
}

void CollarLinesRegistration::getCorrespondingPoints(
    MatrixOfPoints &source_coresp_points,
    MatrixOfPoints &target_coresp_points) {

  PointCloud<PointXYZRGB>::Ptr src_vis_cloud(new PointCloud<PointXYZRGB>(matches.size(), 1));
  PointCloud<PointXYZRGB>::Ptr trg_vis_cloud(new PointCloud<PointXYZRGB>(matches.size(), 1));

  correspondences_weights = VectorXf(matches.size());
  int index = 0;
  last_point_matches.clear();
  for(vector<DMatch>::iterator match = matches.begin(); match < matches.end(); match++, index++) {
    const CLS &source_meta_line = source_cloud[match->trainIdx];
    PointCloudLine source_line = source_meta_line.line;
    const CLS &target_meta_line = target_cloud[match->queryIdx];
    PointCloudLine target_line = target_meta_line.line;

    Vector3f source_line_pt, target_line_pt;
    float source_q, target_q;
    source_line.closestPointsWith(target_line, source_line_pt, target_line_pt, source_q, target_q);
    RNG &rng = theRNG();

    if(!(EigenUtils::allFinite(target_line_pt) && EigenUtils::allFinite(source_line_pt))) {
      source_line_pt = Vector3f(0,0,0);
      target_line_pt = Vector3f(0,0,0);
    }

    source_coresp_points.block(0, index, TPoint3D::RowsAtCompileTime, 1) = source_line_pt;
    target_coresp_points.block(0, index, TPoint3D::RowsAtCompileTime, 1) = target_line_pt;

    last_point_matches.push_back(CLSMatch(source_meta_line, target_meta_line,
                                          source_line_pt, target_line_pt,
                                          source_q, target_q));

    float weight;
    if(params.weighting == VERTICAL_ANGLE_WEIGHTS) {
      weight = getVerticalWeight(source_line.orientation, target_line.orientation);
    } else if(params.weighting == RANGE_WEIGHTS) {
      weight = MAX(source_cloud[match->trainIdx].range, target_cloud[match->queryIdx].range);
    } else if(params.phase_weights_max > -0.0001) {
      const float source_phase = source_cloud[match->trainIdx].phase;
      const float target_phase = target_cloud[match->queryIdx].phase;
      weight = getPhaseWeight(source_phase, target_phase);
    } else {
      assert(params.weighting == NO_WEIGHTS);
      weight = 1.0;
    }
    correspondences_weights.data()[index] = weight;

    src_vis_cloud->at(index).getVector3fMap() = source_line.point;
    trg_vis_cloud->at(index).getVector3fMap() = target_line.point;
    src_vis_cloud->at(index).r = 255*weight;
    src_vis_cloud->at(index).g = src_vis_cloud->at(index).b = 0;
    trg_vis_cloud->at(index).r = 255*weight;
    trg_vis_cloud->at(index).g = trg_vis_cloud->at(index).b = 0;
  }

  if(params.visualize_cls_weights || params.visualize_cls_correspondences) {
    Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
    vis->getViewer()->removeAllShapes();
    vis->keepOnlyClouds(0);
  }

  if(params.visualize_cls_weights) {
    Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
    vis->addColorPointCloud(src_vis_cloud)
        .addColorPointCloud(trg_vis_cloud);
  }

  if(params.visualize_cls_correspondences) {
    Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
    for (int i = 0; i < matches.size(); i++) {
      if (i % 5 == 0) {
        const PointCloudLine &source_line = source_cloud[matches[i].trainIdx].line;
        vis->addLine(source_line);
      }
    }
  }

  if(params.visualize_cls_weights || params.visualize_cls_correspondences) {
    Visualizer3D::getCommonVisualizer()->show();
  }
}

float CollarLinesRegistration::getVerticalWeight(const Vector3f &source_line_orient,
                                                   const Vector3f &target_line_orient) {
  static const float min_weight = 0.01;
  return (sinOfAngleWithGround(source_line_orient) *
      sinOfAngleWithGround(target_line_orient)) + min_weight;
}

float CollarLinesRegistration::sinOfAngleWithGround(const Vector3f &orientation) {
  return orientation.y() / orientation.norm();
}

void CollarLinesRegistration::getWeightingMatrix(WeightsMatrix &weighting_matrix) const {
  if(correspondences_weights.size() == 0) {
    weighting_matrix.resize(matches.size());
    weighting_matrix.setIdentity();
    weighting_matrix.diagonal() /= (float)matches.size();
    return;
  }

  const float weights_normalization = 1.0 / correspondences_weights.sum();
  weighting_matrix.diagonal() = correspondences_weights * weights_normalization;

  assert(weighting_matrix.rows() == correspondences_weights.size());
  assert(weighting_matrix.cols() == correspondences_weights.size());
}


Eigen::Matrix4f CollarLinesRegistration::computeTransformationWeighted(
    const MatrixOfPoints &source_coresp_points,
    const MatrixOfPoints &target_coresp_points)
{
  WeightsMatrix weights(correspondences_weights.size());
  getWeightingMatrix(weights);

  // Lets compute the translation
  // Define Column vector using definition of TPoint3D
  TPoint3D centroid_0;
  MatrixOfPoints target_points_weighted = target_coresp_points * weights;
  centroid_0 << target_points_weighted.row(0).sum(),
      target_points_weighted.row(1).sum(), target_points_weighted.row(2).sum();

  TPoint3D centroid_1;
  MatrixOfPoints source_points_weighted = source_coresp_points * weights;
  centroid_1 << source_points_weighted.row(0).sum(),
      source_points_weighted.row(1).sum(), source_points_weighted.row(2).sum();

  typedef Eigen::Matrix<TPoint3D::Scalar, 4, 1> _TyVector4;
  Eigen::Matrix4f transformation = _TyVector4::Ones().asDiagonal();

  if(params.estimate_translation_only) {
    transformation.block(0, 3, 3, 1) = centroid_1 - centroid_0;
    return transformation;
  }

  Eigen::Matrix<TPoint3D::Scalar, 1, Eigen::Dynamic> identity_vec =
                 Eigen::Matrix<TPoint3D::Scalar, 1, Eigen::Dynamic>::Ones(1, target_coresp_points.cols()); //setOnes();

  // Create matrix with repeating values in columns
  MatrixOfPoints translate_0_mat = centroid_0 * identity_vec;
  MatrixOfPoints translate_1_mat = centroid_1 * identity_vec;

  // Translation of source_coresp_points to the target_coresp_points (Remember this is opposite of camera movement)
  // ie if camera is moving forward, the translation of target_coresp_points to source_coresp_points is opposite
  // TPoint3D t = (centroid_1 - centroid_0);

  // Translate the point cloud 0 to the coordinates of point cloud 1
  MatrixOfPoints target_coresp_points_translated(target_coresp_points.rows(), target_coresp_points.cols());
  MatrixOfPoints source_coresp_points_translated(source_coresp_points.rows(), source_coresp_points.cols());

  target_coresp_points_translated = target_coresp_points - translate_0_mat;
  source_coresp_points_translated = source_coresp_points - translate_1_mat;

  // Compute the Covariance matrix of these two pointclouds moved to the origin
  // This is not properly covariance matrix as there is missing the 1/N
  // 1/N is important for computing eigenvalues(scale), not the eigenvectors(directions) - as we are interested in eigenvectors

  Matrix3f A = target_coresp_points_translated * weights * weights * source_coresp_points_translated.transpose();

  if(params.dont_estimate_roll_pitch) {
    A.row(1) << 0, 0, 0;
    A.col(1) << 0, 0, 0;
  }

  // Compute the SVD upon A = USV^t
  Eigen::JacobiSVD<Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Compute the determinant of V*U^t - to find out in what direction the rotation is
  float det = (svd.matrixV() * svd.matrixU().transpose()).determinant();

  // Fix the right hand/left hand rotation : assuming we would like the right hand rotation
  Matrix3f E = Matrix3f::Identity();
  E(2, 2) = (det >= 0) ? 1.0f : -1.0f;

  // Compute the rotation as R = VEU^t
  // R is the rotation of point_0_translated to fit the source_coresp_points_translated
  Matrix3f R = svd.matrixV() * E * (svd.matrixU().transpose());

  transformation.block(0, 0, 3, 3) = R;
  // The translation must be computed as centroid_1 - rotated centroid_0
  transformation.block(0, 3, 3, 1) = centroid_1 - (R * centroid_0);

  return transformation;
}

void CollarLinesRegistration::showLinesCorrenspondences() {
  vector<DMatch> matches_in_plane;
  float min_distance = INFINITY;
  float max_distance = -1;
  for(vector<DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
    PointCloudLine source_line = source_cloud[m->trainIdx].line;
    PointCloudLine target_line = target_cloud[m->queryIdx].line;
    float distance = source_line.distanceTo(target_line, PointCloudLine::OF_CLOSEST_POINTS);
    matches_in_plane.push_back(DMatch(m->queryIdx, m->trainIdx, distance));
    min_distance = MIN(min_distance, distance);
    max_distance = MAX(max_distance, distance);
  }

  Visualizer3D visualizer;
  for(vector<DMatch>::iterator m = matches_in_plane.begin(); m < matches_in_plane.end(); m++) {
    PointCloudLine source_line = source_cloud[m->trainIdx].line;
    PointCloudLine target_line = target_cloud[m->queryIdx].line;
    float distance = (m->distance-min_distance)/(max_distance-min_distance);    // [0;1]
    visualizer.addLine(source_line, distance, 0.0, 0.0);
    visualizer.addLine(target_line, 0.0, distance, 0.0);
  }

  for(vector<DMatch>::iterator m = rejected_matches.begin(); m < rejected_matches.end(); m++) {
    PointCloudLine source_line = source_cloud[m->trainIdx].line;
    PointCloudLine target_line = target_cloud[m->queryIdx].line;
    visualizer.addLine(source_line, 1.0, 1.0, 1.0);
    visualizer.addLine(target_line, 1.0, 1.0, 1.0);
  }
  visualizer.show();
}

const Eigen::Matrix4f CollarLinesRegistration::getTransformation(void) const {
  return transformation*initial_transformation;
}

} /* namespace but_velodyne */
