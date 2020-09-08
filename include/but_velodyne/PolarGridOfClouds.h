/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2015
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

#ifndef POLARGRIDOFCLOUDS_H_
#define POLARGRIDOFCLOUDS_H_

#include <velodyne_pointcloud/point_types.h>
#include <boost/array.hpp>
#include <cv.h>
#include <pcl/common/eigen.h>

#include <but_velodyne/VelodynePointCloud.h>

namespace but_velodyne
{

/**!
 * Identifier of the polar cell.
 */
class CellId {
public:
  int polar;    ///! discretized horizontal angle
  int ring;     ///! Velodyne ring ID
  int sensor;   ///! Source sensor ID

  CellId(int p, int r) :
   polar(p), ring(r), sensor(0) {
  }

  CellId(int p, int r, int s) :
   polar(p), ring(r), sensor(s) {
  }

  bool operator<(const CellId &other) const {
    if(this->sensor < other.sensor) {
      return true;
    } else if(this->sensor > other.sensor) {
      return false;
    } else {
      if(this->ring < other.ring) {
        return true;
      } else if(this->ring > other.ring) {
        return false;
      } else {
        return (this->polar < other.polar);
      }
    }
  }

  bool operator==(const CellId &other) const {
    return (this->ring   == other.ring) &&
           (this->polar  == other.polar) &&
           (this->sensor == other.sensor);
  }
};

std::ostream& operator<< (std::ostream &out, const CellId &id);

/**!
 * Polar grid structure grouping the Velodyne 3D points of the same ring and the similar horizontal angle.
 */
class PolarGridOfClouds
{
public:

  typedef boost::shared_ptr<PolarGridOfClouds> Ptr;

  /**!
   * Redistributes the original 3D points into the polar grid structure
   *
   * @param point_cloud oroginal Velodyne point cloud
   */
  PolarGridOfClouds(const VelodynePointCloud &point_cloud,
      int polar_superbins_ = 36, int bin_subdivision_ = 1,
      Eigen::Affine3f sensor_pose_ = Eigen::Affine3f::Identity(),
      bool redistribute = false);

  PolarGridOfClouds(const std::vector<VelodynePointCloud::Ptr> &point_cloud,
      const SensorsCalibration &calibration_,
      int polar_superbins_ = 36, int bin_subdivision_ = 1,
      bool redistribute = false);

  /**!
   * @param cellId identifier of the cell
   * @returns the points of the specific polar grid
   */
  const VelodynePointCloud& operator[](const CellId &cellId) const {
    return *polar_grid[getIdx(cellId)];
  }

  const VelodynePointCloud& at(const CellId &cellId) const {
    return *polar_grid[getIdx(cellId)];
  }

  VelodynePointCloud& at(const CellId &cellId) {
    return *polar_grid[getIdx(cellId)];
  }

  /**!
   * @return number of subdivided polar bins
   */
  int getPolarBins() const {
    return polar_superbins*bin_subdivision;
  }

  /**!
   * Transforms the positions of points in the grid structure. Distribution of points
   * into the polar bins remains the same since only rigid transformation is assumed.
   *
   * @param t rigid 3D transformation [R|t]
   */
  void transform(const Eigen::Matrix4f &t, int sensor_idx = -1);

  const std::vector<CellId>& getIndices() {
    return indices;
  }

  int getIdx(const CellId &cell_id) const {
    return (cell_id.sensor*getPolarBins() + cell_id.polar)*rings + cell_id.ring;
  }

  template <typename PointT>
  int getPolarBinIndex(const PointT &point) {
    return getPolarBinIndex(point, this->getPolarBins());
  }

  template <typename PointT>
  static int getPolarBinIndex(const PointT &point, int polar_bins) {
      float angle = VelodynePointCloud::horizontalAngle(point.z, point.x);
      // we want 0deg to point to the back
      angle += 180;
      if(angle >= 360) {
          angle -= 360;
      }
      float polar_bin_size = 360.0f / polar_bins;
    return floor(angle/polar_bin_size);
  }

  const int polar_superbins;                        ///!! number of polar super-bins
  const int bin_subdivision;                        ///!! each bin is split into N sub-bins
  const int rings;
  const int sensors;

protected:

  PolarGridOfClouds(int polar_superbins_, int bin_subdivision_, int rings_, int sensors_);

  void fill(const VelodynePointCloud &point_cloud,
      int sensor_idx, const Eigen::Affine3f &sensor_pose, bool redistribute);

  void allocateClouds();

  int computeNewRingIndex(const velodyne_pointcloud::VelodynePoint &point,
                          const std::vector<float> &borders);

  std::vector<VelodynePointCloud::Ptr> polar_grid;  // polar bins of rings

  std::vector<CellId> indices;
};

} /* namespace but_velodyne */

#endif /* POLARGRIDOFCLOUDS_H_ */
