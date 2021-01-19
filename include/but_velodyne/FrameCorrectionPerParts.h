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

    bool fixFrame(const VelodyneMultiFrame &multiframe,
                  const int frame_i, const int sensor_i,
                  VelodynePointCloud &out_cloud);

protected:

    boost::shared_ptr< std::vector<VelodynePointCloud> > splitByPhase(
            const VelodynePointCloud &in_cloud, const int slices_cnt);

    void fixCloud(const VelodynePointCloud &in_cloud,
                  InterpolationBuffered &interpolation,
                  const Eigen::Affine3f &sensor_pose,
                  VelodynePointCloud &out_cloud);

    void getControlPoints(const std::vector<Eigen::Affine3f> &slice_poses, const int t1_idx,
                          const int slices_cnt, std::vector<Eigen::Affine3f> &control_points);

private:

    const std::vector<Eigen::Affine3f> &slice_poses;
    const int slices_cnt;
};


}


#endif //BUT_VELODYNE_LIB_FRAMECORRECTIONPERPARTS_H
