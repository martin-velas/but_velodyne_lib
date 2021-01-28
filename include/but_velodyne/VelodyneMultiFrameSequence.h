/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 03/08/2020
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


#ifndef BUT_VELODYNE_LIB_VELODYNEMULTIFRAMESEQUENCE_H
#define BUT_VELODYNE_LIB_VELODYNEMULTIFRAMESEQUENCE_H

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/LineCloud.h>

namespace but_velodyne {

class VelodyneMultiFrame {
public:

    typedef boost::shared_ptr <VelodyneMultiFrame> Ptr;

    VelodyneMultiFrame(const std::vector <std::string> &filenames_,
                       const SensorsCalibration &calibration_,
                       bool transform_pcd_files_ = false);

    VelodyneMultiFrame(const std::vector <std::string> &filenames_,
                       const std::vector <VelodynePointCloud::Ptr> &clouds_,
                       const std::vector <LineCloud::Ptr> &line_clouds_,
                       const SensorsCalibration &calibration_) :
            filenames(filenames_), clouds(clouds_), line_clouds(line_clouds_), calibration(calibration_) {
    }

    void transform(const Eigen::Affine3f &pose);

    void save(const std::string &out_dir) const;

    void joinTo(pcl::PointCloud <PointWithSource> &output) const;

    void joinTo(pcl::PointCloud <velodyne_pointcloud::VelodynePoint> &output, bool distinguish_rings = false) const;

    void joinTo(pcl::PointCloud <pcl::PointXYZI> &output) const;

    void joinTo(pcl::PointCloud <pcl::PointXYZ> &output) const;

    void joinTo(LineCloud &output) const;

    bool hasPointClouds(void) const {
      return clouds.size() != 0;
    }

    bool hasLineClouds(void) const {
      return line_clouds.size() != 0;
    }

    std::vector <std::string> filenames;
    std::vector <VelodynePointCloud::Ptr> clouds;
    std::vector <LineCloud::Ptr> line_clouds;
    SensorsCalibration calibration;
};

class VelodyneFileSequence {
public:
    VelodyneFileSequence(const std::vector <std::string> &filenames,
                         const SensorsCalibration &calibration_,
                         bool transform_pcd_files = false);

    bool hasNext(void);

    VelodyneMultiFrame getNext(void);

    VelodyneMultiFrame::Ptr getNextPtr(void);

    void reset(void);

    void next(void);

    bool hasPrev(void);

    VelodyneMultiFrame getPrev(void);

    VelodyneMultiFrame::Ptr getPrevPtr(void);

    int size(void) const {
      return filenames.size() / calibration.sensorsCount();
    }

    int getIndex() const {
      return index;
    }

    VelodyneMultiFrame::Ptr operator[](const int i) const;

private:
    const std::vector <std::string> filenames;
    const SensorsCalibration calibration;
    const bool transform_pcd_files;
    int index;
};

}

#endif //BUT_VELODYNE_LIB_VELODYNEMULTIFRAMESEQUENCE_H
