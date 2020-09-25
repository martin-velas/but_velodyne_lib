/*
 * Registration of dense point clouds.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 29/04/2020
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

#ifndef BUT_VELODYNE_LIB_DENSECLOUDREGISTRATION_H
#define BUT_VELODYNE_LIB_DENSECLOUDREGISTRATION_H

namespace but_velodyne {

class DenseCloudRegistration {

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointNormal PointNormalT;

private:
    pcl::PointCloud<PointNormalT>::Ptr src_cloud;
    pcl::PointCloud<PointNormalT>::Ptr trg_cloud;

public:

    class Parameters {
    public:
        Parameters(
                bool visualization_ = false,
                float leaf_size_ = 0.05,
                float epsilon_ = 1e-6,
                float max_match_distance_ = 0.1,
                size_t max_iterations_ = 100,
                size_t neighbours_for_normal_ = 30,
                float outlier_stdev_thresh_ = 10.0) :
                visualization(visualization_),
                leaf_size(leaf_size_),
                epsilon(epsilon_),
                max_match_distance(max_match_distance_),
                max_iterations(max_iterations_),
                neighbours_for_normal(neighbours_for_normal_),
                outlier_stdev_thresh(outlier_stdev_thresh_) {
        }

        bool visualization;
        float leaf_size;
        float epsilon;
        float max_match_distance;
        size_t max_iterations;
        size_t neighbours_for_normal;
        float outlier_stdev_thresh;

        void loadFrom(boost::program_options::options_description &desc);

    } param;

    DenseCloudRegistration(pcl::PointCloud<PointT>::ConstPtr src_cloud_,
                           pcl::PointCloud<PointT>::ConstPtr trg_cloud_, Parameters parameters_);

    Eigen::Affine3f run(void);
};

}

#endif //BUT_VELODYNE_LIB_DENSECLOUDREGISTRATION_H
