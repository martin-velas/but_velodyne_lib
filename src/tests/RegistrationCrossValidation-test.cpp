#include <math.h>

#include <but_velodyne/RegistrationCrossValidation.h>

#include <gtest/gtest.h>

using namespace but_velodyne;

TEST(RegistrationCrossValidation, PositiveNos) {

  vector<RegistrationOutcome> outcomes;
  RegistrationOutcome best_found;

  // no outcomes - exception expected
  EXPECT_THROW(RegistrationCrossValidation{outcomes}, invalid_argument);

  // only one outcome - should be returned
  outcomes.push_back(RegistrationOutcome(Eigen::Affine3f::Identity(), Termination::ERR_DEVIATION, 0.001f));
  RegistrationCrossValidation validation1(outcomes);
  ASSERT_FLOAT_EQ(validation1.findBest().error, 0.001f);

  // 2 outcomes - should be preferred one with lower error
  outcomes.push_back(RegistrationOutcome(pcl::getTransformation(0.1, 0, 0, 0, 0.02, 0), Termination::ERR_DEVIATION, 0.003f));
  RegistrationCrossValidation validation2(outcomes);
  ASSERT_FLOAT_EQ(validation2.findBest().error, 0.001f);

  // 3 outcomes - first should be rejected by consensus. within consensus, lowest error is preferred
  outcomes.push_back(RegistrationOutcome(pcl::getTransformation(0.11, 0, 0, 0, 0.022, 0), Termination::ERR_DEVIATION, 0.0023f));
  RegistrationCrossValidation validation3(outcomes);
  ASSERT_FLOAT_EQ(validation3.findBest().error, 0.0023f);

  // this is ignored since not converged
  outcomes.push_back(RegistrationOutcome(pcl::getTransformation(0.11, 0, 0, 0, 0.022, 0), Termination::ITERATIONS, 0.0001f));
  RegistrationCrossValidation validation4(outcomes);
  ASSERT_FLOAT_EQ(validation4.findBest().error, 0.0023f);

  // this is not ignored, since is converged
  outcomes.push_back(RegistrationOutcome(pcl::getTransformation(0.111, 0, 0, 0, 0.0221, 0), Termination::ERROR, 0.0001f));
  RegistrationCrossValidation validation5(outcomes);
  ASSERT_FLOAT_EQ(validation5.findBest().error, 0.0001f);

}

#ifndef TESTSUITE

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
