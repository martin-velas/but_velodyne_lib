/*
 * CollarLinesValidation.h
 *
 *  Created on: Nov 1, 2018
 *      Author: Martin Velas (ivelas@fit.vutbr.cz)
 */

#ifndef COLLARLINESVALIDATION_H_
#define COLLARLINESVALIDATION_H_

#include <but_velodyne/CollarLinesRegistration.h>

namespace but_velodyne {

class CollarLinesValidation {

public:

  CollarLinesValidation(const LineCloud &src_cloud_, const LineCloud &trg_cloud_,
      const CollarLinesRegistration::Parameters &registration_params_);

  bool isCapable(void) const;

  float computeError(const Eigen::Matrix4f &transformation);

private:

  const LineCloud src_cloud, trg_cloud;
  const CollarLinesRegistration::Parameters registration_params;
  pcl::KdTreeFLANN<pcl::PointXYZ> src_kdtree;
};

} /* namespace but_velodyne */

#endif /* COLLARLINESVALIDATION_H_ */
