/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Date: 19/01/2021
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

#include <but_velodyne/FrameCorrectionPerParts.h>

#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

namespace but_velodyne {


InterpolationBuffered::Ptr FrameCorrectionPerParts::getInterpolationFor(const int frame_i, const int slice_i) const {

  const float slice_fraction = 1.0 / slices_cnt;

  const Eigen::Affine3f &first_inv = slice_poses[frame_i * slices_cnt].inverse();
  int pose_i = frame_i * slices_cnt + slice_i;
  InterpolationSE3::Ptr interpolation;

  if (pose_i == 0 || pose_i == slice_poses.size() - 2) {
    Eigen::Affine3f current = first_inv * slice_poses[pose_i];
    Eigen::Affine3f next = first_inv * slice_poses[pose_i + 1];
    interpolation.reset(new LinearInterpolationSE3(current, next));

  } else if (pose_i >= slice_poses.size() - 1) {
    PCL_ERROR("ERROR, this should not happen (pose_i >= slice_poses.size()-1)");
    return InterpolationBuffered::Ptr(NULL);

  } else {
    vector <Eigen::Affine3f> control_points;
    getControlPoints(slice_poses, pose_i, slices_cnt, control_points);
    interpolation.reset(new BSplineSE3(control_points));
  }

  float min_phase = slice_i * slice_fraction;
  return InterpolationBuffered::Ptr(new InterpolationBuffered(interpolation,
          -min_phase, 1.0 / slice_fraction));
}

bool FrameCorrectionPerParts::fixFrame(VelodyneMultiFrame &multiframe, const int frame_i) {
  for(int sensor_i = 0; sensor_i < multiframe.calibration.sensorsCount(); sensor_i++) {
    const Eigen::Affine3f &sensor_pose = multiframe.calibration.ofSensor(sensor_i);
    if (multiframe.hasPointClouds()) {
      fixCloud(*multiframe.clouds[sensor_i], frame_i, sensor_pose);
    }
    if (multiframe.hasLineClouds()) {
      fixCloud(*multiframe.line_clouds[sensor_i], frame_i, sensor_pose);
    }
  }
  return true;
}

int FrameCorrectionPerParts::getSliceIdx(const float phase, const int slices_cnt) const {
  const float slice_size = 1.0 / slices_cnt;
  if (0.9999 < phase && phase < 1.0001) {
    return slices_cnt - 1;
  } else {
    return int(floor(phase / slice_size));
  }
}

void FrameCorrectionPerParts::fixSlice(const LineCloud &slice,
                                       InterpolationBuffered &interpolation,
                                       const Eigen::Affine3f &sensor_pose,
                                       LineCloud &out_cloud) const {
  slice.transform(sensor_pose.matrix(), out_cloud);
  for (LineCloud::iterator l = out_cloud.begin(); l < out_cloud.end(); l++) {
    *l = l->transform(interpolation.estimate(l->phase));
  }
  out_cloud.transform(sensor_pose.inverse().matrix(), out_cloud);
}

void FrameCorrectionPerParts::fixSlice(const VelodynePointCloud &slice,
                                       InterpolationBuffered &interpolation,
                                       const Eigen::Affine3f &sensor_pose,
                                       VelodynePointCloud &out_cloud) const {
  transformPointCloud(slice, out_cloud, sensor_pose);
  for (VelodynePointCloud::iterator p = out_cloud.begin(); p < out_cloud.end(); p++) {
    *p = transformPoint(*p, interpolation.estimate(p->phase));
  }
  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
}

void FrameCorrectionPerParts::getControlPoints(const vector <Eigen::Affine3f> &slice_poses, const int t1_idx,
                                               const int slices_cnt, vector <Eigen::Affine3f> &control_points) const {
  const Eigen::Affine3f relative_pose_inv = slice_poses[(t1_idx / slices_cnt) * slices_cnt].inverse();
  for (int i = t1_idx - 1; i <= t1_idx + 2; i++) {
    control_points.push_back(relative_pose_inv * slice_poses[i]);
  }
}


}
