/*
 * Overlap.h
 *
 *  Created on: Feb 18, 2019
 *      Author: ivelas
 */

#ifndef OVERLAP_H_
#define OVERLAP_H_

#include <pcl/common/common.h>
#include <velodyne_pointcloud/point_types.h>

#include <but_velodyne/Visualizer3D.h>

namespace but_velodyne {

class SphericalZbuffer {
public:
  typedef boost::shared_ptr<SphericalZbuffer> Ptr;

  SphericalZbuffer(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud_,
      const int azimuthal_bins_, const int polar_bins_, const float depth_quantile);

  float getDepth(const float azimuth, const float polar_angle) const;

  size_t containsPoints(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance);

  size_t containsPoints(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance,
      pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &overlap,
      pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &rest);

  bool containsPoint(const velodyne_pointcloud::VelodynePoint &point,
    const float depth_relative_tolerance, const float depth_absolute_tolerance);

  void addToVisualizer(but_velodyne::Visualizer3D &visualizer,
      const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &src_cloud,
      const float depth_relative_tolerance = 0.0, const float depth_absolute_tolerance = 0.0) const;

  float visitedPortion(void) const;

protected:
  void setDepth(const float azimuth, const float polar_angle, const float depth);

  void visit(const float azimuth, const float polar_angle);

  int getIndex(const float azimuth, const float polar_angle) const;

private:
  const int azimuthal_bins, polar_bins;
  const float azimuthal_resolution, polar_resolution;
  std::vector<float> depths;
  std::vector<bool> visited;
  int cells_occupied;
};

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular);

float harmonic_avg(const float a, const float b);

} /* namespace but_velodyne */

#endif /* OVERLAP_H_ */
