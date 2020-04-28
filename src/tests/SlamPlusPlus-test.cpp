#include <math.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/SlamPlusPlus.h>

#include <gtest/gtest.h>

using namespace but_velodyne;

TEST(SlamPlusPlus, CircularLoop) {

  Eigen::Matrix<double, 6, 1> information;
  information << 100, 100, 100, 10000, 10000, 10000;

  const float dx = 0.5;   // sin(30deg)
  const float dz = 0.86;  // sin(60deg) with error
  const float Ry = 1.0;   // rad(60deg) with error

  const Eigen::Affine3f delta = pcl::getTransformation(dx, 0.0, dz, 0.0, Ry, 0.0);

  vector<Eigen::Affine3f> init_poses(1, Eigen::Affine3f::Identity());

  SlamPlusPlus slampp;

  for(int i = 0; i < 6; i++) {
    slampp.addEdge(i, i+1, delta, information.asDiagonal());
    init_poses.push_back(init_poses.back() * delta);
  }
  slampp.addEdge(0, 6, Eigen::Affine3f::Identity(), information.asDiagonal());

  // Visualizer3D vis;
  // vis.getViewer()->removeAllCoordinateSystems();
  // vis.addPoses(init_poses).show();

  Eigen::Affine3f diff;
  diff = init_poses.front().inverse() * init_poses.back();
  cerr << "Diff initial: " << diff << endl;
  ASSERT_GT(diff.translation().norm(), 0.10);

  vector<Eigen::Affine3f> optimized_poses;
  slampp.optimize(optimized_poses);

  diff = optimized_poses.front().inverse() * optimized_poses.back();
  cerr << "Diff optimized: " << diff << endl;
  ASSERT_LT(diff.translation().norm(), 0.03);

  // vis.addPoses(optimized_poses).show();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
