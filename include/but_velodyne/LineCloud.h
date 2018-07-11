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

  pcl::PointCloud<pcl::PointXYZ>::Ptr generateDenseCloud(const int points_per_cell, const int cell_count) const;

  /**!
   * @return all lines
   */
  const std::vector<PointCloudLine>& getLines() const {
    return line_cloud;
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

  std::vector<PointCloudLine> line_cloud;       ///! collar line segments
  pcl::PointCloud<pcl::PointXYZ> line_middles;  ///! midpoints of line segments

  inline LineCloud& operator +=(const LineCloud& other) {
    this->line_cloud.insert(this->line_cloud.end(), other.line_cloud.begin(), other.line_cloud.end());
    this->line_middles.insert(this->line_middles.end(), other.line_middles.begin(), other.line_middles.end());
    return *this;
  }

  void push_back(const PointCloudLine &line);

  void save(const std::string &filename) const;

  void load(const std::string &filename);

  pcl::PointCloud<pcl::PointXYZ>::Ptr encode(void) const;

  void decodeFrom(const pcl::PointCloud<pcl::PointXYZ> &cloud);

  int size(void) const {
    assert(line_cloud.size() == line_middles.size());
    return line_cloud.size();
  }

protected:
  void generateLineCloudFromCell(const PolarGridOfClouds &polar_grid,
                                 const CellId &source_cell,
                                 const int lines_per_cell_pair_generated,
                                 std::vector<PointCloudLine> &line_cloud) const;

  void generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                   CellId cell1, CellId cell2,
                                   int lines_per_cell_pair_generated,
                                   std::vector<PointCloudLine> &line_cloud) const;

  std::vector<CellId> getTargetCells(const CellId &source_cell, int total_polar_bins, int bin_subdivision) const;

private:
  static cv::RNG& rng;
  CollarLinesFilter filter;
};

} /* namespace but_velodyne */

#endif /* LINESCLOUD_H_ */
