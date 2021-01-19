/*
 * Odometry estimation by collar line segments map of Velodyne scans.
 *
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
 * Date: 19/11/2020
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

#include <but_velodyne/CollarLinesMap.h>

using namespace pcl;

namespace but_velodyne {

void CollarLinesRegistrationToMap::pruneIfNeeded(const int new_points_cnt) {
  if (lines_map.size() * prune_ratio > new_points_cnt) {
    cerr << "[DEBUG] Before pruning: " << lines_map.size() << endl;
    lines_map.prune(prune_ratio);
    indexed = false;
    cerr << "[DEBUG] After pruning: " << lines_map.size() << endl;
  }
}

Eigen::Affine3f CollarLinesRegistrationToMap::runMapping(const VelodyneMultiFrame &multiframe,
        const SensorsCalibration &calibration, const Eigen::Affine3f &init_pose, const int frame_id,
        vector <CLSMatch> &matches, Termination::Reason &reason) {
  PointCloud<PointXYZ> target_cloud_vis;
  if (vis) {
    multiframe.joinTo(target_cloud_vis);
    vis->keepOnlyClouds(0).setColor(200, 0, 200).addPointCloud(target_cloud_vis, init_pose.matrix())
            .setColor(150, 150, 150).addPointCloud(*lines_map.all_lines.getMiddles()).show();
  }

  PolarGridOfClouds target_polar_grid(multiframe.clouds, calibration);
  LineCloud target_line_cloud(target_polar_grid, params.linesPerCellGenerated, filter);
  Eigen::Affine3f refined_pose = registerLineCloud(target_line_cloud, init_pose, matches, reason);
  addToMap(target_line_cloud, refined_pose, frame_id);
  pruneIfNeeded(target_line_cloud.size());

  if (vis) {
    vis->keepOnlyClouds(1).setColor(0, 200, 0)
            .addPointCloud(target_cloud_vis, refined_pose.matrix()).show();
  }

  return refined_pose;
}

Eigen::Affine3f CollarLinesRegistrationToMap::registerLineCloud(const LineCloud &target,
        const Eigen::Affine3f &initial_transformation, vector<CLSMatch> &matches, Termination::Reason &reason) {
  if (!indexed) {
    buildKdTree();
  }

  Termination termination(params.term_params);
  Eigen::Matrix4f transformation = initial_transformation.matrix();
  CollarLinesRegistration icl_fitting(lines_map.all_lines, map_kdtree, target,
                                      registration_params, transformation);
  while (!termination()) {
    float error = icl_fitting.refine();
    termination.addNewError(error);
    transformation = icl_fitting.getTransformation();
  }
  icl_fitting.getLastMatches(matches);
  reason = termination.why();
  return Eigen::Affine3f(transformation);
}

}
