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


bool FrameCorrectionPerParts::fixFrame(const VelodyneMultiFrame &multiframe,
                                       const int frame_i, const int sensor_i,
                                       VelodynePointCloud &out_cloud) {

  const float slice_fraction = 1.0 / slices_cnt;
  const Eigen::Affine3f &sensor_pose = multiframe.calibration.ofSensor(sensor_i);
  const VelodynePointCloud &in_cloud = *multiframe.clouds[sensor_i];

  const Eigen::Affine3f &first_inv = slice_poses[frame_i * slices_cnt].inverse();

  boost::shared_ptr<vector < VelodynePointCloud> > in_cloud_parts =
          splitByPhase(in_cloud, slices_cnt);

  for (int slice_i = 0; slice_i < slices_cnt; slice_i++) {

    int pose_i = frame_i * slices_cnt + slice_i;
    InterpolationSE3::Ptr interpolation;

    if (pose_i == 0 || pose_i == slice_poses.size() - 2) {
      Eigen::Affine3f current = first_inv * slice_poses[pose_i];
      Eigen::Affine3f next = first_inv * slice_poses[pose_i + 1];
      interpolation.reset(new LinearInterpolationSE3(current, next));

    } else if (pose_i >= slice_poses.size() - 1) {
      PCL_ERROR("ERROR, this should not happen (pose_i >= slice_poses.size()-1)");
      return false;

    } else {
      vector <Eigen::Affine3f> control_points;
      getControlPoints(slice_poses, pose_i, slices_cnt, control_points);
      interpolation.reset(new BSplineSE3(control_points));
    }

    float min_phase = slice_i * slice_fraction;
    InterpolationBuffered interpolation_buffered(interpolation, -min_phase, 1.0 / slice_fraction);

    VelodynePointCloud out_cloud_part;
    fixCloud(in_cloud_parts->at(slice_i), interpolation_buffered, sensor_pose, out_cloud_part);
    out_cloud += out_cloud_part;
  }

  return true;
}

boost::shared_ptr<vector < VelodynePointCloud> >
FrameCorrectionPerParts::splitByPhase(
        const VelodynePointCloud &in_cloud, const int slices_cnt) {
  boost::shared_ptr<vector < VelodynePointCloud> > slices(new vector<VelodynePointCloud>(slices_cnt));
  const float slice_size = 1.0 / slices_cnt;
  for (VelodynePointCloud::const_iterator p = in_cloud.begin(); p < in_cloud.end(); p++) {
    int slice_i;
    if (0.9999 < p->phase && p->phase < 1.0001) {
      slice_i = slices_cnt - 1;
    } else {
      slice_i = int(floor(p->phase / slice_size));
    }
    slices->at(slice_i).push_back(*p);
  }
  return slices;
}

void FrameCorrectionPerParts::fixCloud(const VelodynePointCloud &in_cloud,
                                       InterpolationBuffered &interpolation,
                                       const Eigen::Affine3f &sensor_pose,
                                       VelodynePointCloud &out_cloud) {
  transformPointCloud(in_cloud, out_cloud, sensor_pose);
  for (VelodynePointCloud::iterator p = out_cloud.begin(); p < out_cloud.end(); p++) {
    *p = transformPoint(*p, interpolation.estimate(p->phase));
  }
  transformPointCloud(out_cloud, out_cloud, sensor_pose.inverse());
}

void FrameCorrectionPerParts::getControlPoints(const vector <Eigen::Affine3f> &slice_poses, const int t1_idx,
                                               const int slices_cnt, vector <Eigen::Affine3f> &control_points) {
  const Eigen::Affine3f relative_pose_inv = slice_poses[(t1_idx / slices_cnt) * slices_cnt].inverse();
  for (int i = t1_idx - 1; i <= t1_idx + 2; i++) {
    control_points.push_back(relative_pose_inv * slice_poses[i]);
  }
}


}
