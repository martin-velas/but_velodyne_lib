/*
 * CollarLineFilter.cpp
 *
 *  Created on: 23.2.2016
 *      Author: ivelas
 */

#include <pcl/point_types.h>

#include <but_velodyne/CollarLinesFilter.h>

using namespace std;
using namespace pcl;

namespace but_velodyne
{

class PointCloudLineWithIndex {
public:
  PointCloudLineWithIndex(const PointCloudLine &line_, const size_t index_) :
    line(line_), index(index_) {
  }

  bool operator < (const PointCloudLineWithIndex &other) const {
    return this->line < other.line;
  }

  bool lessByHorizontalRangeDiff(const PointCloudLineWithIndex &other) const {
    return this->line.horizontalRangeDiff() < other.line.horizontalRangeDiff();
  }

  PointCloudLine line;
  size_t index;
};

bool CollarLinesFilter::checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const {
  return true;
}

bool orderLinesByHorizontalRangeDiff(const PointCloudLineWithIndex &first, const PointCloudLineWithIndex &second) {
  return first.lessByHorizontalRangeDiff(second);
}

void CollarLinesFilter::filterLines(const vector<PointCloudLine> &in_lines,
                                   const CellId &src_cell, const CellId &targ_cell,
                                   vector<PointCloudLine> &out_lines, vector<size_t> &out_indices) const {
  vector<PointCloudLineWithIndex> filtered_lines;
  for(size_t i = 0; i < in_lines.size(); i++) {
    const PointCloudLine &line = in_lines[i];
    filtered_lines.push_back(PointCloudLineWithIndex(line, i));
  }

  if(comparation_metric == LINE_LENGTH) {
    sort(filtered_lines.begin(), filtered_lines.end());
  } else if (comparation_metric == HORIZONTAL_RANGE_DIFF) {
    sort(filtered_lines.begin(), filtered_lines.end(), orderLinesByHorizontalRangeDiff);
  }
  int lines_to_effectively_preserve = MIN(lines_to_preserve, filtered_lines.size());

  for(int i = 0; i < lines_to_effectively_preserve; i++) {
    out_lines.push_back(filtered_lines[i].line);
    out_indices.push_back(filtered_lines[i].index);
  }
}

bool AngularCollarLinesFilter::checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const {
  float expected_diff = getExpectedRangesDiff(src_cell.ring, targ_cell.ring);

  float actual_diff = line.horizontalRangeDiff();
  //if(line.orientation.norm() > 4 && actual_diff < (expected_diff + TOLERANCE))
    //cerr << "line: " << line << " exp: " << expected_diff << " actual: " << actual_diff << endl;
  return actual_diff < (expected_diff*params.horizontal_range_diff_tolerance_rel + params.horizontal_range_diff_tolerance_abs);
}

float AngularCollarLinesFilter::getExpectedRangesDiff(int src_ring, int targ_ring) const {
  float src_range = VelodyneSpecification::getExpectedRange(src_ring, velodyne_model, VelodyneSpecification::KITTI_HEIGHT);
  float targ_range = VelodyneSpecification::getExpectedRange(targ_ring, velodyne_model, VelodyneSpecification::KITTI_HEIGHT);
  if (clouds_processed > 0) {
    if (params.weight_of_expected_horizontal_range_diff < 0) {
      if (src_range < targ_range) {
        src_range = MIN(src_range, max_ring_ranges[src_range]);
        targ_range = MAX(targ_range, max_ring_ranges[targ_range]);
      }
      else {
        src_range = MAX(src_range, max_ring_ranges[src_range]);
        targ_range = MIN(targ_range, max_ring_ranges[targ_range]);
      }
    }
    else {
      src_range = src_range * params.weight_of_expected_horizontal_range_diff
          + max_ring_ranges[src_range] * (1 - params.weight_of_expected_horizontal_range_diff);
      targ_range = targ_range * params.weight_of_expected_horizontal_range_diff
          + max_ring_ranges[targ_range] * (1 - params.weight_of_expected_horizontal_range_diff);
    }
  }
  return (isinf(src_range)||isinf(targ_range)) ?
      params.max_horizontal_range_diff :
      MIN(fabs(src_range - targ_range), params.max_horizontal_range_diff);
}

void AngularCollarLinesFilter::addNewMaxRingRanges(std::vector<float> max_ring_ranges_) {
  assert(max_ring_ranges_.size() == VelodyneSpecification::rings(velodyne_model));
  for(int r = 0; r < VelodyneSpecification::rings(velodyne_model); r++) {
    max_ring_ranges[r] = max_ring_ranges[r]*clouds_processed + max_ring_ranges_[r];
  }
  clouds_processed++;
}

} /* namespace but_visual_registration */
