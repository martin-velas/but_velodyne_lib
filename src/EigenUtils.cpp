#include <but_velodyne/EigenUtils.h>

#include <Eigen/Eigenvalues>

namespace but_velodyne {

void getEigenvalues(const Eigen::Matrix3f &covariance, std::vector<float> &eigenvalues) {
  Eigen::Vector3cf ev = covariance.eigenvalues();
  eigenvalues.resize(3);
  eigenvalues[0] = std::real(ev(0));
  eigenvalues[1] = std::real(ev(1));
  eigenvalues[2] = std::real(ev(2));
  sort(eigenvalues.begin(), eigenvalues.end());
}

float tdiff(const Eigen::Affine3f t1, const Eigen::Affine3f t2, const float dist) {
  const Eigen::Affine3f delta = t2 * t1.inverse();
  const float t_error = delta.translation().norm();
  const float r_error = Eigen::AngleAxisf(delta.rotation()).angle();
  return t_error + tan(r_error)*dist;
}

}
