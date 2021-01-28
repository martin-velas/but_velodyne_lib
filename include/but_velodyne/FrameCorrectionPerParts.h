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

#ifndef BUT_VELODYNE_LIB_FRAMECORRECTIONPERPARTS_H
#define BUT_VELODYNE_LIB_FRAMECORRECTIONPERPARTS_H

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/InterpolationSE3.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>

namespace but_velodyne {


class FrameCorrectionPerParts {

public:

    FrameCorrectionPerParts(const std::vector<Eigen::Affine3f> &slice_poses_, const int slices_cnt_) :
            slice_poses(slice_poses_), slices_cnt(slices_cnt_) {
    }

    bool fixFrame(VelodyneMultiFrame &multiframe, const int frame_i);

protected:

    template <typename CloudT>
    void splitByPhase(const CloudT &in_cloud, const int slices_cnt,
                                                          std::vector<CloudT> &slices,
                                                          std::vector< std::vector<int> > &indices) const {
      slices.resize(slices_cnt);
      indices.resize(slices_cnt);
      int idx = 0;
      for (typename CloudT::const_iterator p = in_cloud.begin(); p < in_cloud.end(); p++, idx++) {
        int slice_i = getSliceIdx(p->phase, slices_cnt);
        slices[slice_i].push_back(*p);
        indices[slice_i].push_back(idx);
      }
    }

    template <class CloudT>
    void fixCloud(CloudT &cloud, const int frame_i, const Eigen::Affine3f &sensor_pose) const {
      std::vector<CloudT> slices;
      std::vector< std::vector<int> > indices;
      splitByPhase(cloud, slices_cnt, slices, indices);

      for (int slice_i = 0; slice_i < slices_cnt; slice_i++) {
        InterpolationBuffered::Ptr interpolation_buffered = getInterpolationFor(frame_i, slice_i);
        CloudT slice_fixed;
        fixSlice(slices[slice_i], *interpolation_buffered, sensor_pose, slice_fixed);

        const std::vector<int> &slice_indices = indices[slice_i];
        for(int i = 0; i < slice_fixed.size(); i++) {
          cloud[slice_indices[i]] = slice_fixed[i];
        }
      }
    }

    int getSliceIdx(const float phase, const int slices_cnt) const;

    InterpolationBuffered::Ptr getInterpolationFor(const int frame_i, const int slice_i) const;

    void fixSlice(const VelodynePointCloud &slice,
                  InterpolationBuffered &interpolation,
                  const Eigen::Affine3f &sensor_pose,
                  VelodynePointCloud &out_cloud) const;

    void fixSlice(const LineCloud &slice,
                  InterpolationBuffered &interpolation,
                  const Eigen::Affine3f &sensor_pose,
                  LineCloud &out_cloud) const;

    void getControlPoints(const std::vector<Eigen::Affine3f> &slice_poses, const int t1_idx,
                          const int slices_cnt, std::vector<Eigen::Affine3f> &control_points) const;

private:

    const std::vector<Eigen::Affine3f> &slice_poses;
    const int slices_cnt;
};


}


#endif //BUT_VELODYNE_LIB_FRAMECORRECTIONPERPARTS_H
