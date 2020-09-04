#include <math.h>

#include <but_velodyne/CollarLinesRegistration.h>

#include <gtest/gtest.h>

using namespace but_velodyne;

typedef boost::shared_ptr<CollarLinesRegistration> CollarLinesRegistrationPtr;

// expected matches:
// trg[1] -> src[0] (best wo weights)
// trg[0] -> src[1] (best wi phase weights)
void fillWithLines(LineCloud &src_lines, LineCloud &trg_lines) {
  src_lines.push_back(PointCloudLine(Eigen::Vector3f(0.0, 0, 0), Eigen::Vector3f(1.01, 0, 0)), 0, Eigen::Vector3f::UnitX(), 0.0);
  src_lines.push_back(PointCloudLine(Eigen::Vector3f(1.0, 0, 0), Eigen::Vector3f(0.92, 0, 0)), 0, Eigen::Vector3f::UnitX(), 0.9);
  trg_lines.push_back(PointCloudLine(Eigen::Vector3f(1.11, 0, 0), Eigen::Vector3f(0.91, 0, 0)), 0, Eigen::Vector3f::UnitX(), 0.9);
  trg_lines.push_back(PointCloudLine(Eigen::Vector3f(0.10, 0, 0), Eigen::Vector3f(1.00, 0, 0)), 0, Eigen::Vector3f::UnitX(), 0.0);
}

CollarLinesRegistrationPtr getDummyClsReg(void) {
  LineCloud src_lines, trg_lines;
  fillWithLines(src_lines, trg_lines);

  CollarLinesRegistration::Parameters params;
  return CollarLinesRegistrationPtr(new CollarLinesRegistration(src_lines, trg_lines, params));
}

namespace but_velodyne {

TEST(CollarLinesRegistration, getWeightingMatrixTest) {
  CollarLinesRegistrationPtr reg = getDummyClsReg();

  reg->matches.resize(3);
  Eigen::DiagonalMatrix<CollarLinesRegistration::TPoint3D::Scalar, Eigen::Dynamic, Eigen::Dynamic> weights_matrix;
  reg->getWeightingMatrix(weights_matrix);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(0), 1.0 / 3.0);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(1), 1.0 / 3.0);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(2), 1.0 / 3.0);

  reg->correspondences_weights = Eigen::VectorXf(3);
  reg->correspondences_weights << 1, 2, 3;
  reg->getWeightingMatrix(weights_matrix);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(0), 1.0 / 6.0);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(1), 2.0 / 6.0);
  ASSERT_FLOAT_EQ(weights_matrix.diagonal()(2), 3.0 / 6.0);
}

TEST(CollarLinesRegistration, computeErrorTest) {
  CollarLinesRegistrationPtr reg = getDummyClsReg();

  reg->matches.resize(2);
  CollarLinesRegistration::MatrixOfPoints src_points(size_t(CollarLinesRegistration::TPoint3D::RowsAtCompileTime), 2);
  CollarLinesRegistration::MatrixOfPoints trg_points(size_t(CollarLinesRegistration::TPoint3D::RowsAtCompileTime), 2);
  float avg_error;

  src_points << 1, 2, 3, 4, 5, 6;
  trg_points << 1, 2, 3, 4, 5, 6;
  avg_error = reg->computeError(src_points, trg_points, Eigen::Matrix4f::Identity());
  ASSERT_FLOAT_EQ(0.0, avg_error);

  Eigen::Affine3f T = pcl::getTransformation(0, 0, 1, 0, 0, 0);
  avg_error = reg->computeError(src_points, trg_points, T.matrix());
  ASSERT_FLOAT_EQ(1.0, avg_error);

  trg_points << -1, 2, 3, 4, 5, 7;
  avg_error = reg->computeError(src_points, trg_points, Eigen::Matrix4f::Identity());
  ASSERT_FLOAT_EQ(3.0 / 2, avg_error);  // total error is 3 for 2 points
}

TEST(CollarLinesRegistration, findClosestMatchesByMiddlesTest) {
  CollarLinesRegistration::Parameters params;

  LineCloud src_lines, trg_lines;
  fillWithLines(src_lines, trg_lines);

  params.distance_threshold = CollarLinesRegistration::MEDIAN_THRESHOLD;
  CollarLinesRegistration registration_median(src_lines, trg_lines, params);
  registration_median.findClosestMatchesByMiddles();
  ASSERT_EQ(1, registration_median.matches.size());
  ASSERT_EQ(1, registration_median.matches[0].queryIdx);
  ASSERT_EQ(0, registration_median.matches[0].trainIdx);

  registration_median.params.phase_weights_max = 1.0;
  registration_median.params.phase_weights_power = 4.0;
  registration_median.findClosestMatchesByMiddles();
  ASSERT_EQ(1, registration_median.matches.size());
  ASSERT_EQ(0, registration_median.matches[0].queryIdx);
  ASSERT_EQ(1, registration_median.matches[0].trainIdx);

  params.distance_threshold = CollarLinesRegistration::NO_THRESHOLD;
  CollarLinesRegistration registration_all(src_lines, trg_lines, params);
  registration_all.findClosestMatchesByMiddles();
  ASSERT_EQ(2, registration_all.matches.size());
  ASSERT_EQ(0, registration_all.matches[0].queryIdx);
  ASSERT_EQ(1, registration_all.matches[0].trainIdx);
  ASSERT_EQ(1, registration_all.matches[1].queryIdx);
  ASSERT_EQ(0, registration_all.matches[1].trainIdx);
}

/*
 * Test both getMatches() and fillKdtreesBySensors() method.
 */
TEST(CollarLinesRegistration, getMatchesTest) {
  CollarLinesRegistration::Parameters params;
  LineCloud src_lines, trg_lines;
  fillWithLines(src_lines, trg_lines);
  CollarLinesRegistration registration(src_lines, trg_lines, params);

  ASSERT_EQ(2, registration.source_kdtree.getInputCloud()->size());
  ASSERT_EQ(0, registration.source_kdtrees_by_sensor.size());

  vector<int> indices;
  vector<float> sq_distances;
  registration.getMatches(1, indices, sq_distances);
  ASSERT_EQ(0, indices[0]);
  ASSERT_NEAR(0.1, sqrt(sq_distances[0]), 0.01);

  params.separate_sensors = true;
  src_lines[0].sensor_id = 1;
  CollarLinesRegistration registration_separated(src_lines, trg_lines, params);

  ASSERT_EQ(2, registration_separated.source_kdtree.getInputCloud()->size());
  ASSERT_EQ(2, registration_separated.source_kdtrees_by_sensor.size());

  registration_separated.getMatches(1, indices, sq_distances);
  ASSERT_EQ(1, indices[0]);
}

TEST(CollarLinesRegistration, refineTest) {
  CollarLinesRegistration::Parameters params;
  LineCloud src_lines, trg_lines;
  fillWithLines(src_lines, trg_lines);

  CollarLinesRegistration registration(src_lines, trg_lines, params);
  ASSERT_EQ(0, registration.refinements_done);

  registration.refine();
  ASSERT_EQ(1, registration.refinements_done);
}

TEST(CollarLinesRegistration, getEffectiveThresholdTest) {
  CollarLinesRegistrationPtr reg = getDummyClsReg();

  reg->params.distance_threshold = CollarLinesRegistration::VALUE_THRESHOLD;
  reg->params.distance_threshold_value = 0.42;
  ASSERT_FLOAT_EQ(0.42, reg->getEffectiveThreshold());

  reg->matches.push_back(cv::DMatch(0, 1, 0.01));
  reg->matches.push_back(cv::DMatch(0, 1, 0.77));
  reg->matches.push_back(cv::DMatch(0, 1, 10.0));

  reg->params.distance_threshold = CollarLinesRegistration::NO_THRESHOLD;
  ASSERT_LE(10.0, reg->getEffectiveThreshold());

  reg->params.distance_threshold = CollarLinesRegistration::PORTION_VALUE_THRESHOLD;
  reg->params.distance_threshold_value = 0.51;      // more than half of the matches
  ASSERT_FLOAT_EQ(0.77, reg->getEffectiveThreshold());

  reg->params.distance_threshold = CollarLinesRegistration::MEDIAN_THRESHOLD;
  ASSERT_FLOAT_EQ(0.77, reg->getEffectiveThreshold());

  reg->params.distance_threshold_decay = 0.4;
  reg->refinements_done = 2;
  // only 0.4^2 * 0.5 = 0.08 = 8% portion
  ASSERT_FLOAT_EQ(0.4 * 0.4, reg->getEffectiveDecay());
  ASSERT_FLOAT_EQ(0.01, reg->getEffectiveThreshold());

  ASSERT_FLOAT_EQ((0.01 + 0.77 + 10.0) / 3.0, reg->getMatchesMean());

  reg->filterMatchesByThreshold(0.78);
  ASSERT_EQ(2, reg->matches.size());

  ASSERT_FLOAT_EQ(0.01, reg->getMatchesDistanceThreshold(0.1));  // up to 10%

  reg->params.distance_threshold = CollarLinesRegistration::MEDIAN_THRESHOLD;
  ASSERT_FLOAT_EQ(0.5, reg->thresholdTypeToFraction());
  reg->params.distance_threshold = CollarLinesRegistration::PERC_90_THRESHOLD;
  ASSERT_FLOAT_EQ(0.9, reg->thresholdTypeToFraction());
  reg->params.distance_threshold = CollarLinesRegistration::PERC_99_THRESHOLD;
  ASSERT_FLOAT_EQ(0.99, reg->thresholdTypeToFraction());
}

};

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
