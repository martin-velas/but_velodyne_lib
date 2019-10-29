/*
 * RegistrationCrossValidation.cpp
 *
 *  Created on: Oct 31, 2018
 *      Author: Martin Velas (ivelas@fit.vutbr.cz)
 */

#include <but_velodyne/RegistrationCrossValidation.h>

using namespace but_velodyne;
using namespace pcl;

namespace but_velodyne {

RegistrationCrossValidation::RegistrationCrossValidation(const std::vector<RegistrationOutcome> &nfold_reg_outcomes_) :
  nfold_reg_outcomes(nfold_reg_outcomes_) {
  if(nfold_reg_outcomes.empty()) {
    throw invalid_argument("No registration outcomes presented");
  }
}

bool RegistrationCrossValidation::containsConvergedResults(void) const {
  for(vector<RegistrationOutcome>::const_iterator fold = nfold_reg_outcomes.begin(); fold < nfold_reg_outcomes.end(); fold++) {
    if(Termination::is_converged(fold->term_reason)) {
      return true;
    }
  }
  return false;
}

void RegistrationCrossValidation::removeNotConverged(void) {
  for(vector<RegistrationOutcome>::iterator fold = nfold_reg_outcomes.begin(); fold < nfold_reg_outcomes.end();) {
    if(Termination::is_converged(fold->term_reason)) {
      fold++;
    } else {
      fold = nfold_reg_outcomes.erase(fold);
    }
  }
}

void RegistrationCrossValidation::keepConselsus(void) {
  int consensus_size = (nfold_reg_outcomes.size() + 1) / 2;   // equality or majority
  static const pcl::PointXYZ SAMPLE(10, 10, 10);
  PointCloud<PointXYZ>::Ptr transformed_samples(new PointCloud<PointXYZ>);
  for(vector<RegistrationOutcome>::const_iterator fold = nfold_reg_outcomes.begin(); fold < nfold_reg_outcomes.end(); fold++) {
    transformed_samples->push_back(transformPoint(SAMPLE, fold->transformation));
  }
  float best_consensus_distance = INFINITY;
  vector<int> best_consensus_indices;
  KdTreeFLANN<PointXYZ> tree;
  tree.setInputCloud(transformed_samples);
  for(PointCloud<PointXYZ>::const_iterator seed = transformed_samples->begin(); seed < transformed_samples->end(); seed++) {
    vector<float> distances(consensus_size);
    vector<int> indices(consensus_size);
    tree.nearestKSearch(*seed, consensus_size, indices, distances);
    float sum_distance = accumulate(distances.begin(), distances.end(), 0.0f);
    if(sum_distance < best_consensus_distance) {
      best_consensus_indices = indices;
      best_consensus_distance = sum_distance;
    }
  }
  vector<RegistrationOutcome> new_outcomes;
  for(vector<int>::const_iterator i = best_consensus_indices.begin(); i < best_consensus_indices.end(); i++) {
    new_outcomes.push_back(nfold_reg_outcomes[*i]);
  }
  nfold_reg_outcomes = new_outcomes;
}

RegistrationOutcome RegistrationCrossValidation::pickBestByError(void) {
  sort(nfold_reg_outcomes.begin(), nfold_reg_outcomes.end());
  return nfold_reg_outcomes.front();
}

RegistrationOutcome RegistrationCrossValidation::findBest(void) {
  if(containsConvergedResults()) {
    removeNotConverged();
  }
  if(nfold_reg_outcomes.size() > 2) {
    keepConselsus();
  }
  return pickBestByError();
}

} /* namespace but_velodyne */
