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

#ifndef LINESCLOUD_H_
#define LINESCLOUD_H_

#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/CollarLinesFilter.h>

namespace but_velodyne
{

/**!
 * Cloud of collar line segments.
 */
class LineCloud
{
public:

  typedef boost::shared_ptr<LineCloud> Ptr;

  class PointCloudLineWithMiddleAndOrigin {
  public:
    PointCloudLineWithMiddleAndOrigin(const PointCloudLine &line_, const pcl::PointXYZ &middle_,
        const int sensor_id_, const Eigen::Vector3f &normal_,
        const float phase_) :
      line(line_), middle(middle_), sensor_id(sensor_id_), normal(normal_), phase(phase_),
      range(middle_.getVector3fMap().norm()), frame_id(-1) {
    }

    PointCloudLineWithMiddleAndOrigin transform(const Eigen::Affine3f &t) const;

    PointCloudLine line;
    pcl::PointXYZ middle;
    int sensor_id;
    Eigen::Vector3f normal;
    float phase;
    float range;
    int frame_id;
  };

  typedef std::vector<PointCloudLineWithMiddleAndOrigin>::iterator iterator;
  typedef std::vector<PointCloudLineWithMiddleAndOrigin>::const_iterator const_iterator;

  const_iterator begin(void) const {
    return data.begin();
  }

  iterator begin(void) {
    return data.begin();
  }

  const_iterator end(void) const {
    return data.end();
  }

  iterator end(void) {
    return data.end();
  }

  iterator erase(iterator it) {
    return this->data.erase(it);
  }

  iterator erase(iterator begin, iterator end) {
    return this->data.erase(begin, end);
  }

  /**!
   * Initialize empty line cloud.
   */
  LineCloud() :
    filter(CollarLinesFilter(0)) {
    // empty
  }

  /**!
   * @param polar_grid point cloud formated into polar grid
   * @param lines_per_cell_pair_generated how many lines are generated for each bin
   * @param lines_per_cell_pair_preserved how many lines are preserved within each bin
   * @param preservedFactorType which line segments are preferred
   */
  LineCloud(const PolarGridOfClouds &polar_grid,
            const int lines_per_cell_pair_generated,
            CollarLinesFilter &filter_);

  PointCloudLineWithMiddleAndOrigin& operator [](int idx) {
      return data[idx];
  }

  PointCloudLineWithMiddleAndOrigin operator [](int idx) const {
      return data[idx];
  }

  /**!
   * Transform line cloud.
   *
   * @param transformation transformation matrix
   * @param output destination
   */
  void transform(const Eigen::Matrix4f &transformation, LineCloud &output) const;

  /**!
   * Transform line cloud - output is *this cloud.
   *
   * @param transformation transformation matrix
   */
  void transform(const Eigen::Matrix4f &transformation);

  inline LineCloud& operator +=(const LineCloud& other) {
    this->data.insert(this->data.end(), other.data.begin(), other.data.end());
    return *this;
  }

  void append(const LineCloud& other, const int frame_id);

  void push_back(const PointCloudLine &line, const int sensor_id,
      const Eigen::Vector3f &normal, const float phase);

  void push_back(const std::vector<PointCloudLine> &lines, const int sensor_id,
      const std::vector<Eigen::Vector3f> &normals, std::vector<float> &phases);

  void push_back(const PointCloudLineWithMiddleAndOrigin &line_with_metadata) {
    data.push_back(line_with_metadata);
  }

  int size(void) const {
    return this->data.size();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getMiddles(void) const;

protected:
  void generateLineCloudFromCell(const PolarGridOfClouds &polar_grid,
                                 const CellId &source_cell,
                                 const int lines_per_cell_pair_generated,
                                 std::vector<PointCloudLine> &line_cloud,
                                 std::vector<float> &output_phases) const;

  void generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                   CellId cell1, CellId cell2,
                                   int lines_per_cell_pair_generated,
                                   std::vector<PointCloudLine> &line_cloud,
                                   std::vector<float> &output_phases) const;

  std::vector<CellId> getTargetCells(const CellId &source_cell, int total_polar_bins, int bin_subdivision) const;

private:
  static cv::RNG& rng;
  CollarLinesFilter filter;

  std::vector<PointCloudLineWithMiddleAndOrigin> data;       ///! collar line segments
};

typedef LineCloud::PointCloudLineWithMiddleAndOrigin CLS;

std::ostream& operator<<(std::ostream& stream, const CLS &l);

std::ostream& operator<<(std::ostream& stream, const LineCloud &lcd);

std::istream& operator>>(std::istream &stream, CLS &l);

std::istream& operator>>(std::istream &stream, LineCloud &lcd);

} /* namespace but_velodyne */

#endif /* LINESCLOUD_H_ */
