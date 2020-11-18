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

#ifndef BUT_VELODYNE_LIB_COLLARLINESMAP_H
#define BUT_VELODYNE_LIB_COLLARLINESMAP_H


#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

namespace but_velodyne {

class CLSMap {

public:
    LineCloud all_lines;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMiddlesPtr(void) {
      return all_lines.getMiddles();
    }

    int size(void) {
      return all_lines.size();
    }

    void add(const LineCloud &lines, const int frame_id) {
      all_lines.append(lines, frame_id);
      cerr << "All lines: " << all_lines.size() << "; from new frame [" << frame_id << "]: " << lines.size() << endl;
    }

    void prune(const float ratio) {
      random_shuffle(all_lines.begin(), all_lines.end());
      all_lines.erase(all_lines.begin() + all_lines.size() * (1.0 - ratio),
                      all_lines.end());
    }

    void clear(void) {
      all_lines.erase(all_lines.begin(), all_lines.end());
    }
};


class CollarLinesRegistrationToMap {
public:
    CollarLinesRegistrationPipeline::Parameters params;
    CollarLinesRegistration::Parameters registration_params;

    CollarLinesRegistrationToMap(CollarLinesRegistrationPipeline::Parameters pipeline_params_,
                                 CollarLinesRegistration::Parameters registration_params_,
                                 const float prune_ratio_, const bool &visualization) :
            filter(pipeline_params_.linesPerCellPreserved),
            params(pipeline_params_),
            registration_params(registration_params_),
            prune_ratio(prune_ratio_), indexed(false), last_frame_id(-1) {
      if (visualization) {
        vis = Visualizer3D::getCommonVisualizer();
      }
    }

    void addToMap(const std::vector<VelodynePointCloud::Ptr> &point_clouds,
                  const SensorsCalibration &calibration, const Eigen::Affine3f &pose) {
      PolarGridOfClouds polar_grid(point_clouds, calibration);
      LineCloud line_cloud(polar_grid, params.linesPerCellGenerated, filter);
      addToMap(line_cloud, pose);
    }

    Eigen::Affine3f runMapping(const VelodyneMultiFrame &multiframe,
                               const SensorsCalibration &calibration, const Eigen::Affine3f &init_pose,
                               vector <CLSMatch> &matches);

    void reset(void) {
      lines_map.clear();
      indexed = false;
    }

protected:

    void addToMap(const LineCloud &line_cloud,
                  const Eigen::Affine3f &pose) {
      LineCloud line_cloud_transformed;
      line_cloud.transform(pose.matrix(), line_cloud_transformed);
      lines_map.add(line_cloud_transformed, ++last_frame_id);
      indexed = false;
    }

    void buildKdTree(void) {
      map_kdtree.setInputCloud(lines_map.getMiddlesPtr());
      indexed = true;
    }

    Eigen::Affine3f registerLineCloud(const LineCloud &target,
                                      const Eigen::Affine3f &initial_transformation, vector <CLSMatch> &matches);

private:
    float prune_ratio;
    CollarLinesFilterRangeCheck filter;
    CLSMap lines_map;
    pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree;
    bool indexed;
    Visualizer3D::Ptr vis;
    int last_frame_id;
};

}

#endif //BUT_VELODYNE_LIB_COLLARLINESMAP_H
