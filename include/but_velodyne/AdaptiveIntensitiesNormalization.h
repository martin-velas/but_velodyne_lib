/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 16/01/2020
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

#ifndef BUT_VELODYNE_LIB_ADAPTIVEINTENSITIESNORMALIZATION_H
#define BUT_VELODYNE_LIB_ADAPTIVEINTENSITIESNORMALIZATION_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <but_velodyne/VelodynePointCloud.h>

namespace but_velodyne {

    class AdaptiveIntensitiesNormalization {
    public:
        AdaptiveIntensitiesNormalization(const float expected_mean_, const float expected_std_dev_) :
                expected_mean(expected_mean_), expected_std_dev(expected_std_dev_) {
        }

        void run(pcl::PointCloud<PointWithSource>::Ptr sum_cloud,
                 const SensorsCalibration &calibration,
                 const std::vector <Eigen::Affine3f> &poses,
                 pcl::PointCloud<PointWithSource>::Ptr &out_cloud);

    private:
        const float expected_mean, expected_std_dev;
    };

}

#endif //BUT_VELODYNE_LIB_ADAPTIVEINTENSITIESNORMALIZATION_H
