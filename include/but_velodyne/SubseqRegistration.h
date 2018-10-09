/*
 * SubseqRegistration.h
 *
 *  Created on: Nov 3, 2017
 *      Author: ivelas
 */

#ifndef SUBSEQREGISTRATION_H_
#define SUBSEQREGISTRATION_H_

#include <but_velodyne/LineCloud.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

namespace but_velodyne {

class SubseqRegistration {
public:
  SubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
      const Eigen::Affine3f &init_transform_,
      CollarLinesRegistrationPipeline::Parameters &params_,
      CollarLinesRegistration::Parameters &registration_params_) :
    src_lines(src_lines_), trg_lines(trg_lines_),
    params(params_), registration_params(registration_params_),
    estimated_transform(init_transform_), term_reason(Termination::NO) {
  }

  virtual ~SubseqRegistration() {}

  virtual Eigen::Affine3f run();

  Termination::Reason getTerminationReason(void) const {
    return term_reason;
  }

protected:

  Eigen::Matrix4f registerLineClouds(
      const LineCloud &source, const LineCloud &target,
      const Eigen::Matrix4f &initial_transformation,
      CollarLinesRegistration::Parameters registration_params,
      CollarLinesRegistrationPipeline::Parameters pipeline_params);

  Eigen::Affine3f estimated_transform;
  LineCloud src_lines, trg_lines;
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;
  Termination::Reason term_reason;
};

class ManualSubseqRegistration : public SubseqRegistration {
public:
  ManualSubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
      const Eigen::Affine3f &init_transform_,
      CollarLinesRegistrationPipeline::Parameters &params_,
      CollarLinesRegistration::Parameters &registration_params_,
      Visualizer3D::Ptr visualizer_ = Visualizer3D::Ptr(new Visualizer3D));

  Eigen::Affine3f run();

protected:

  void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*);

  void setDataToVisualizer();

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*);

  void estimateManualTransform();

  void runAutomaticRegistration(CollarLinesRegistration::Threshold th_type);

  int split_idx;
  Visualizer3D::Ptr visualizer;
  pcl::visualization::PCLVisualizer::Ptr pclVis;
  vector<int> src_indices, trg_indices;
};

} /* namespace but_velodyne */

#endif /* SUBSEQREGISTRATION_H_ */
