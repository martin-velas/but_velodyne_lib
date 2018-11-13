/*
 * CollarLinesValidation.cpp
 *
 *  Created on: Nov 1, 2018
 *      Author: Martin Velas (ivelas@fit.vutbr.cz)
 */

#include <but_velodyne/Termination.h>
#include <but_velodyne/CollarLinesValidation.h>

namespace but_velodyne {

CollarLinesValidation::CollarLinesValidation(const LineCloud &src_cloud_, const LineCloud &trg_cloud_,
    const CollarLinesRegistration::Parameters &registration_params_) :
        src_cloud(src_cloud_), trg_cloud(trg_cloud_), registration_params(registration_params_) {
  if(src_cloud.size() != 0) {
    src_kdtree.setInputCloud(src_cloud.line_middles.makeShared());
  }
}

bool CollarLinesValidation::isCapable(void) const {
  return src_cloud.size() != 0 && trg_cloud.size() != 0;
}

float CollarLinesValidation::computeError(const Eigen::Matrix4f &transformation) {
  if(!isCapable()) {
    return Termination::UNKNOWN_ERROR;
  }
  CollarLinesRegistration cls_validation(src_cloud, src_kdtree,
      trg_cloud, registration_params, transformation);
  return cls_validation.computeError();
}

} /* namespace but_velodyne */
