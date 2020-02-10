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

#ifndef BUT_VELODYNE_LIB_COLLARLINESREGISTRATIONPERPART_H
#define BUT_VELODYNE_LIB_COLLARLINESREGISTRATIONPERPART_H

#include <pcl/common/eigen.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

namespace but_velodyne {

    void visualize_registration(const VelodynePointCloud &src_cloud, const VelodynePointCloud &trg_cloud,
                                const Eigen::Affine3f &T);

    void register_clouds_parts(const VelodyneMultiFrame &src_frame, const VelodyneMultiFrame &trg_frame,
                               CollarLinesRegistration::Parameters registration_parameters,
                               CollarLinesRegistrationPipeline::Parameters pipeline_parameters,
                               const size_t parts,
                               const bool visualization, vector<RegistrationOutcome> &results);

    void replace_suffix_from_previous_frame(const VelodynePointCloud &previous, VelodynePointCloud &next, float portion);
}

#endif //BUT_VELODYNE_LIB_COLLARLINESREGISTRATIONPERPART_H
