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
#include <boost/format.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/LineCloud.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;

namespace but_velodyne
{
cv::RNG& LineCloud::rng(cv::theRNG());

Eigen::Vector3f estimate_normal(const PointCloudLine &line, const vector<Eigen::Vector3f> &points) {
  Eigen::Vector3f normal_sum(0, 0, 0);
  const Eigen::Vector3f to_origin = line.middle();
  for(vector<Eigen::Vector3f>::const_iterator p = points.begin(); p < points.end(); p++) {
    Eigen::Vector3f normal = line.orientation.cross(*p-line.point);
    if(normal.dot(to_origin) > 0) { // orientation to sensor
      normal = -normal;
    }
    normal_sum += normal;
  }
  if(!normal_sum.isZero()) {
    normal_sum.normalize();
  }
  return normal_sum;
}

void estimate_normals(const vector<PointCloudLine> &lines, vector<Eigen::Vector3f> &normals) {
  if(lines.size() == 1) {
    normals.push_back(Eigen::Vector3f::Zero());
  } else {
    for(int line_i = 0; line_i < lines.size(); line_i++) {
      vector<Eigen::Vector3f> cell_points;
      for(int pair_i = 0; pair_i < lines.size(); pair_i++) {
        if(line_i != pair_i) {
          cell_points.push_back(lines[pair_i].point);
          cell_points.push_back(lines[pair_i].point + lines[pair_i].orientation);
        }
      }
      normals.push_back(estimate_normal(lines[line_i], cell_points));
    }
  }
}

LineCloud::PointCloudLineWithMiddleAndOrigin LineCloud::PointCloudLineWithMiddleAndOrigin::transform(
    const Eigen::Affine3f &t) const {
  PointCloudLineWithMiddleAndOrigin transformed = PointCloudLineWithMiddleAndOrigin (
    this->line.transform(t.matrix()),
    transformPoint(this->middle, t),
    this->sensor_id,
    t.rotation() * this->normal,
    this->phase
  );
  transformed.range = this->range;
  return transformed;
}

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
        std::vector<float> phases;
        generateLineCloudFromCell(polar_grid,
                                  CellId(polar, ring, sensor_idx),
                                  lines_per_cell_pair_generated,
                                  lines_among_cells, phases);
        vector<Eigen::Vector3f> normals;
        estimate_normals(lines_among_cells, normals);
        this->push_back(lines_among_cells, sensor_idx, normals, phases);
      }
    }
  }
}

void LineCloud::push_back(const PointCloudLine &line, const int sensor_id,
    const Eigen::Vector3f &normal, const float phase) {
  Eigen::Vector3f middle = line.middle();
  PointCloudLineWithMiddleAndOrigin new_line(line, PointXYZ(middle.x(), middle.y(), middle.z()),
      sensor_id, normal, phase);
  data.push_back(new_line);
}

void LineCloud::push_back(const vector<PointCloudLine> &lines, const int sensor_id,
    const std::vector<Eigen::Vector3f> &normals, vector<float> &phases) {
  int i = 0;
  for(vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++) {
    Eigen::Vector3f normal = !normals.empty() ? normals[i] : Eigen::Vector3f::Zero();
    this->push_back(*l, sensor_id, normal, phases[i]);
    i++;
  }
}

void LineCloud::append(const LineCloud& other, const int frame_id) {
  this->data.insert(this->data.end(), other.data.begin(), other.data.end());
  LineCloud::iterator new_it = this->data.begin() + (this->data.size() - other.data.size());
  for(; new_it < this->data.end(); new_it++) {
    new_it->frame_id = frame_id;
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
                                            vector<PointCloudLine> &output_lines,
                                            vector<float> &output_phases) const {
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
  vector<float> generated_phases;
  for(int i = 0; i < lines_to_generate; i++) {
    int cell1_index = rng(cell1->size());
    int cell2_index = rng(cell2->size());
    const VelodynePoint &pt1 = cell1->at(cell1_index);
    const VelodynePoint &pt2 = cell2->at(cell2_index);
    PointCloudLine generated_line(pt1, pt2);
    float phase;
    if(fabs(pt1.phase - pt2.phase) > 0.5) {
      // the overlap of start and end of single revolution
      phase = NAN;
    } else {
      phase = (pt1.phase + pt2.phase) / 2.0;
    }
    generated_lines.push_back(generated_line);
    generated_phases.push_back(phase);
  }
  vector<size_t> indices;
  filter.filterLines(generated_lines, cell1_id, cell2_id, output_lines, indices);
  for(vector<size_t>::iterator i = indices.begin(); i < indices.end(); i++) {
    output_phases.push_back(generated_phases[*i]);
  }
}

void LineCloud::generateLineCloudFromCell(const PolarGridOfClouds &polar_grid,
                               const CellId &source_cell,
                               const int lines_per_cell_pair_generated,
                               std::vector<PointCloudLine> &line_cloud,
                               vector<float> &output_phases) const {
  vector<CellId> target_cells = getTargetCells(source_cell, polar_grid.getPolarBins(), polar_grid.bin_subdivision);
  for(vector<CellId>::iterator target_cell = target_cells.begin(); target_cell < target_cells.end(); target_cell++) {
    generateLineCloudAmongCells(polar_grid,
                                source_cell, *target_cell,
                                lines_per_cell_pair_generated,
                                line_cloud, output_phases);
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
  for(vector<PointCloudLineWithMiddleAndOrigin>::const_iterator l = data.begin(); l < data.end(); l++) {
    output.data.push_back(l->transform(transformation));
  }
}

void LineCloud::transform(const Eigen::Matrix4f &t_matrix) {
  Eigen::Affine3f transformation(t_matrix);
  for(vector<PointCloudLineWithMiddleAndOrigin>::iterator l = data.begin(); l < data.end(); l++) {
    *l = l->transform(transformation);
  }
}

std::ostream& operator<<(std::ostream& stream, const CLS &l) {
  const Eigen::Vector3f &p = l.line.point;
  const Eigen::Vector3f &o = l.line.orientation;
  const pcl::PointXYZ &m = l.middle;
  const Eigen::Vector3f &n = l.normal;

  stream << boost::format("%1% %2% %3% %4% %5% %6% %7% %8% %9% %10% %11% %12% %13% %14% %15% %16%") %
            p.x() %
            p.y() %
            p.z() %

            o.x() %
            o.y() %
            o.z() %

            m.x %
            m.y %
            m.z %

            l.sensor_id %

            n.x() %
            n.y() %
            n.z() %

            l.phase %
            l.range %
            l.frame_id;

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const LineCloud &lcd) {
  for(LineCloud::const_iterator l = lcd.begin(); l < lcd.end(); l++) {
    stream << *l << endl;
  }
  return stream;
}

std::istream& operator>>(std::istream &stream, CLS &l) {
  Eigen::Vector3f &p = l.line.point;
  Eigen::Vector3f &o = l.line.orientation;
  pcl::PointXYZ &m = l.middle;
  Eigen::Vector3f &n = l.normal;

  stream >> p.x() >>
         p.y() >>
         p.z() >>

         o.x() >>
         o.y() >>
         o.z() >>

         m.x >>
         m.y >>
         m.z >>

         l.sensor_id >>

         n.x() >>
         n.y() >>
         n.z() >>

         l.phase >>
         l.range >>
         l.frame_id;

  return stream;
}

std::istream& operator>>(std::istream &stream, LineCloud &lcd) {
  CLS line(PointCloudLine(), PointXYZ(), -1, Eigen::Vector3f::Zero(), -1);

  while(true) {
    stream >> line;
    if(stream.eof()) {
      break;
    } else {
      lcd.push_back(line);
    }
  }
}

void LineCloud::fromFile(const std::string &infile, LineCloud &out_line_cloud) {
  std::cerr << "Processing LineCloud file: " << infile << std::endl << std::flush;
  std::ifstream in_stream(infile.c_str());
  in_stream >> out_line_cloud;
}

} /* namespace but_velodyne */
