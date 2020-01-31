/*
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 31/01/2020
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

#include <but_velodyne/CollarLinesRegistrationPerPart.h>

namespace but_velodyne {

void visualize_registration(const VelodynePointCloud &src_cloud, const VelodynePointCloud &trg_cloud,
                            const Eigen::Affine3f &T) {
  VelodynePointCloud trg_cloud_transformed;
  transformPointCloud(trg_cloud, trg_cloud_transformed, T);
  Visualizer3D::getCommonVisualizer()->keepOnlyClouds(0)
          .addColorPointCloud(Visualizer3D::colorizeCloudByPhase(src_cloud))
          .setColor(0, 0, 255).addPointCloud(trg_cloud_transformed)
          .show();
}

void register_clouds_parts(const VelodynePointCloud &src_cloud, const VelodynePointCloud &trg_cloud,
                           CollarLinesRegistration::Parameters registration_parameters,
                           CollarLinesRegistrationPipeline::Parameters pipeline_parameters,
                           const size_t parts,
                           const bool visualization, vector<RegistrationOutcome> &results) {
  if (visualization) {
    visualize_registration(src_cloud, trg_cloud, Eigen::Affine3f::Identity());
  }

  PolarGridOfClouds src_grid(src_cloud);
  PolarGridOfClouds trg_grid(trg_cloud);

  LinearMoveEstimator null_estimator(0);
  ofstream null_file("/dev/null");

  registration_parameters.distance_threshold = CollarLinesRegistration::PERC_90_THRESHOLD;
  CollarLinesRegistrationPipeline registration(null_estimator, null_file,
                                               pipeline_parameters, registration_parameters);
  RegistrationOutcome result_whole;
  registration.registerTwoGrids(src_grid, trg_grid, Eigen::Matrix4f::Identity(), result_whole);
  if (visualization) {
    visualize_registration(src_cloud, trg_cloud, result_whole.transformation);
  }

  for (float max_phase = 0.0; max_phase < 0.99; max_phase += 1.0 / parts) {
    registration_parameters.phase_weights_max = max_phase;
    registration_parameters.distance_threshold = CollarLinesRegistration::PORTION_VALUE_THRESHOLD;
    registration_parameters.distance_threshold_value = 1.0 / parts;
    CollarLinesRegistrationPipeline registration_partial(null_estimator, null_file,
                                                         pipeline_parameters, registration_parameters);

    RegistrationOutcome result_part;
    registration_partial.registerTwoGrids(src_grid, trg_grid, Eigen::Matrix4f::Identity(), result_part);
    results.push_back(result_part);
    cerr << "Max phase: " << max_phase << endl;
    if (visualization) {
      visualize_registration(src_cloud, trg_cloud, result_part.transformation);
    }
  }
}

void append_end_to_next_frame(const VelodynePointCloud &previous, VelodynePointCloud &next, float portion) {
  PhaseFilter filter(1.0 - portion, 1.0);
  VelodynePointCloud slice;
  filter.filter(previous, slice);
  for (VelodynePointCloud::iterator p = slice.begin(); p < slice.end(); p++) {
    p->phase -= 1.0;
  }
  next += slice;
}

}
