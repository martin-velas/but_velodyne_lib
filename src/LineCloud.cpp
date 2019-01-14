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

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/program_options/errors.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/LineCloud.h>

using namespace std;
using namespace pcl;

namespace but_velodyne
{
cv::RNG& LineCloud::rng(cv::theRNG());

LineCloud::LineCloud(const PolarGridOfClouds &polar_grid,
                     const int lines_per_cell_pair_generated,
                     CollarLinesFilter &filter_) :
    filter(filter_)
{
  for(int sensor_idx = 0; sensor_idx < polar_grid.sensors; sensor_idx++) {
    for(int polar = 0; polar < polar_grid.getPolarBins(); polar++) {
      for(int ring = 0; ring < polar_grid.rings-1; ring++) {
        //cerr << "Ring: " << ring << ", expected_range: " << VelodyneSpecification::getExpectedRange(ring, VelodyneSpecification::KITTI_HEIGHT) << endl;
        vector<PointCloudLine> lines_among_cells;
        generateLineCloudFromCell(polar_grid,
                                  CellId(polar, ring, sensor_idx),
                                  lines_per_cell_pair_generated,
                                  lines_among_cells);
        this->push_back(lines_among_cells, sensor_idx);
      }
    }
  }
}

void LineCloud::push_back(const PointCloudLine &line, const int sensor_id) {
  Eigen::Vector3f middle = line.middle();
  PointCloudLineWithMiddleAndOrigin new_line(line, PointXYZ(middle.x(), middle.y(), middle.z()), sensor_id);
  data.push_back(new_line);
}

void LineCloud::push_back(const vector<PointCloudLine> &lines, const int sensor_id) {
  for(vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++) {
    this->push_back(*l, sensor_id);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LineCloud::getMiddles(void) const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr middles(new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<PointCloudLineWithMiddleAndOrigin>::const_iterator l = data.begin(); l < data.end(); l++) {
    middles->push_back(l->middle);
  }
  return middles;
}

void LineCloud::generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                            CellId cell1_id, CellId cell2_id,
                                            int lines_per_cell_pair_generated,
                                            vector<PointCloudLine> &output_lines) const {
  const VelodynePointCloud *cell1 = &polar_grid[cell1_id];
  const VelodynePointCloud *cell2;
  while(true) {
    cell2 = &polar_grid[cell2_id];
    if(cell2->empty() && (cell2_id.ring < polar_grid.rings-1)) {
      cell2_id.ring++;
      /*cerr << cell1_id << " -> " << cell2_id << ", expected_range_diff: "
          << fabs(VelodyneSpecification::getExpectedRange(cell1_id.ring, VelodyneSpecification::KITTI_HEIGHT) -
          VelodyneSpecification::getExpectedRange(cell2_id.ring, VelodyneSpecification::KITTI_HEIGHT)) << endl;*/
    } else {
      break;
    }
  }
  int lines_to_generate = MIN(lines_per_cell_pair_generated, cell1->size()*cell2->size());

  vector<PointCloudLine> generated_lines;
  for(int i = 0; i < lines_to_generate; i++) {
    int cell1_index = rng(cell1->size());
    int cell2_index = rng(cell2->size());
    PointCloudLine generated_line(cell1->at(cell1_index),
                                  cell2->at(cell2_index));
    generated_lines.push_back(generated_line);
  }
  filter.filterLines(generated_lines, output_lines, cell1_id, cell2_id);
}

void LineCloud::generateLineCloudFromCell(const PolarGridOfClouds &polar_grid,
                               const CellId &source_cell,
                               const int lines_per_cell_pair_generated,
                               std::vector<PointCloudLine> &line_cloud) const {
  vector<CellId> target_cells = getTargetCells(source_cell, polar_grid.getPolarBins(), polar_grid.bin_subdivision);
  for(vector<CellId>::iterator target_cell = target_cells.begin(); target_cell < target_cells.end(); target_cell++) {
    generateLineCloudAmongCells(polar_grid,
                                source_cell, *target_cell,
                                lines_per_cell_pair_generated,
                                line_cloud);
  }
}

vector<CellId> LineCloud::getTargetCells(const CellId &source_cell, int total_polar_bins, int bin_subdivision) const {
  vector<CellId> target_cells;

  int min_polar = source_cell.polar - bin_subdivision / 2;
  int max_polar = source_cell.polar + bin_subdivision / 2;

  for(int polar = min_polar; polar <= max_polar; polar++) {
    int polar_periodic = (polar + total_polar_bins) % total_polar_bins;
    target_cells.push_back(CellId(polar_periodic, source_cell.ring+1, source_cell.sensor));
  }

  return target_cells;
}

void LineCloud::transform(const Eigen::Matrix4f &t_matrix, LineCloud &output) const {
  output.data.clear();
  Eigen::Affine3f transformation(t_matrix);
  for(int i = 0; i < this->size(); i++) {
    PointCloudLineWithMiddleAndOrigin new_line(
        data[i].line.transform(t_matrix),
        transformPoint(data[i].middle, transformation),
        data[i].sensor_id);
    output.data.push_back(new_line);
  }
}

void LineCloud::transform(const Eigen::Matrix4f &t_matrix) {
  Eigen::Affine3f transformation(t_matrix);
  for(std::vector<PointCloudLineWithMiddleAndOrigin>::iterator lineWithMidAndOrigin = data.begin();
      lineWithMidAndOrigin < data.end(); lineWithMidAndOrigin++) {
    lineWithMidAndOrigin->line = lineWithMidAndOrigin->line.transform(t_matrix);
    lineWithMidAndOrigin->middle = pcl::transformPoint(lineWithMidAndOrigin->middle, transformation);
  }
}

} /* namespace but_velodyne */
