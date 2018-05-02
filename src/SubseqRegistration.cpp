/*
 * SubseqRegistration.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: ivelas
 */

#include <but_velodyne/SubseqRegistration.h>

#include <pcl/registration/transformation_estimation_svd.h>

using namespace pcl;

namespace but_velodyne {

Eigen::Affine3f SubseqRegistration::run() {
  return Eigen::Affine3f(
      registerLineClouds(src_lines, trg_lines,
                         estimated_transform.matrix(),
                         registration_params, params));
}

Eigen::Matrix4f SubseqRegistration::registerLineClouds(
    const LineCloud &source, const LineCloud &target,
    const Eigen::Matrix4f &initial_transformation,
    CollarLinesRegistration::Parameters registration_params,
    CollarLinesRegistrationPipeline::Parameters pipeline_params) {
  Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations,
      pipeline_params.maxTimeSpent, pipeline_params.significantErrorDeviation,
      pipeline_params.targetError);
  CollarLinesRegistration icl_fitting(source, target, registration_params,
      initial_transformation);
  Eigen::Matrix4f transformation;
  while (!termination()) {
    float error = icl_fitting.refine();
    termination.addNewError(error);
    transformation = icl_fitting.getRefinedTransformation();
  }
  return transformation*initial_transformation;
}

ManualSubseqRegistration::ManualSubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
    const Eigen::Affine3f &init_transform_,
    CollarLinesRegistrationPipeline::Parameters &params_,
    CollarLinesRegistration::Parameters &registration_params_,
    Visualizer3D::Ptr visualizer_) :
  SubseqRegistration(src_lines_, trg_lines_,
      init_transform_,
      params_, registration_params_),
  split_idx(src_lines_.line_cloud.size()), visualizer(visualizer_) {

  pclVis = visualizer->getViewer();
  pclVis->registerPointPickingCallback(&ManualSubseqRegistration::pickPointCallback, *this);
  pclVis->registerKeyboardCallback(&ManualSubseqRegistration::keyCallback, *this);
}

Eigen::Affine3f ManualSubseqRegistration::run() {
  setDataToVisualizer();
  visualizer->show();
  return estimated_transform;
}

void ManualSubseqRegistration::pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*) {
  int idx = event.getPointIndex();
  if (idx == -1)
    return;

  if(idx < split_idx) {
    if(src_indices.size() < trg_indices.size()) {
      src_indices.push_back(idx);
      setDataToVisualizer();
    } else {
      PCL_WARN("Clicked source point but expected target - ignoring\n");
    }
  } else {
    if(src_indices.size() == trg_indices.size()) {
      trg_indices.push_back(idx-split_idx);
      setDataToVisualizer();
    } else {
      PCL_WARN("Clicked target point but expected source - ignoring\n");
    }
  }
}

void ManualSubseqRegistration::setDataToVisualizer() {
  PointCloud<PointXYZRGB>::Ptr sum_cloud(new PointCloud<PointXYZRGB>);
  *sum_cloud += *Visualizer3D::colorizeCloud(src_lines.line_middles, 255, 0, 0);
  PointCloud<PointXYZ> trg_cloud_transformed;
  transformPointCloud(trg_lines.line_middles, trg_cloud_transformed, estimated_transform);
  *sum_cloud += *Visualizer3D::colorizeCloud(trg_cloud_transformed, 0, 0, 255);
  pclVis->removeAllShapes();
  pclVis->removeAllPointClouds();
  visualizer->addColorPointCloud(sum_cloud);
  for(int i = 0; i < src_indices.size(); i++) {
    visualizer->addArrow(PointCloudLine(src_lines.line_middles[src_indices[i]],
                                       trg_cloud_transformed[trg_indices[i]]));
  }
  if(src_indices.size() < trg_indices.size()) {
    pclVis->addSphere(trg_cloud_transformed[trg_indices.back()], 0.1, "sphere");
  }
}

void ManualSubseqRegistration::keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
  if(event.keyDown()) {
    if(event.getKeySym() == "s") {
      PCL_DEBUG("Running transformation estimation using SVD ...\n");
      estimateManualTransform();
      setDataToVisualizer();
    } else if(event.getKeySym() == "u") {
      if(src_indices.size() < trg_indices.size() && !trg_indices.empty()) {
        trg_indices.pop_back();
      } else if(!src_indices.empty()) {
        src_indices.pop_back();
      }
      setDataToVisualizer();
    } else if(event.getKeySym() == "a") {
      runAutomaticRegistration(CollarLinesRegistration::NO_THRESHOLD);
      setDataToVisualizer();
    } else if(event.getKeySym() == "m") {
      runAutomaticRegistration(CollarLinesRegistration::MEDIAN_THRESHOLD);
      setDataToVisualizer();
    } else if(event.getKeySym() == "n") {
      runAutomaticRegistration(CollarLinesRegistration::QUARTER_THRESHOLD);
      setDataToVisualizer();
    } else if(event.getKeySym() == "b") {
      runAutomaticRegistration(CollarLinesRegistration::TENTH_THRESHOLD);
      setDataToVisualizer();
    } else if(event.getKeySym() == "v") {
      runAutomaticRegistration(CollarLinesRegistration::VALUE_THRESHOLD);
      setDataToVisualizer();
    }
  }
}

void ManualSubseqRegistration::estimateManualTransform() {
  if(src_indices.size() > 2) {
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> tfe;
    Eigen::Matrix4f t;
    tfe.estimateRigidTransformation(trg_lines.line_middles, trg_indices,
        src_lines.line_middles, src_indices, t);
    estimated_transform = Eigen::Affine3f(t);
  } else {
    PCL_WARN("Unable to estimate transformation with less than 3 matches - skipping.\n");
  }
}

void ManualSubseqRegistration::runAutomaticRegistration(CollarLinesRegistration::Threshold th_type) {
  PCL_DEBUG("Running automatic transformation estimation using CLS ...\n");
  registration_params.distance_threshold = th_type;
  estimated_transform = Eigen::Affine3f(registerLineClouds(src_lines, trg_lines,
      estimated_transform.matrix(), registration_params, params));
}

} /* namespace but_velodyne */
