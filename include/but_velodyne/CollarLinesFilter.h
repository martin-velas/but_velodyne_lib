/*
 * CollarLineFilter.h
 *
 *  Created on: 23.2.2016
 *      Author: ivelas
 */

#ifndef COLLARLINEFILTER_H_
#define COLLARLINEFILTER_H_

#include <vector>

#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/PolarGridOfClouds.h>

namespace but_velodyne
{

class CollarLinesFilter
{
public:

  enum COMPARATION {
    LINE_LENGTH,
    HORIZONTAL_RANGE_DIFF
  };

  CollarLinesFilter(int lines_to_preserve_, COMPARATION comparation_metric_ = LINE_LENGTH) :
    lines_to_preserve(lines_to_preserve_),
    comparation_metric(comparation_metric_){
  }

  virtual ~CollarLinesFilter() {
  }

  void filterLines(const std::vector<PointCloudLine> &in_lines,
                   const CellId &src_cell, const CellId &targ_cell,
                   std::vector<PointCloudLine> &out_lines, std::vector<size_t> &out_indices) const;

protected:
  virtual bool checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const;

private:
  int lines_to_preserve;
  COMPARATION comparation_metric;
};

class CollarLinesFilterRangeCheck : public CollarLinesFilter {

public:

  CollarLinesFilterRangeCheck(const int lines_to_preserve_)
          : CollarLinesFilter(lines_to_preserve_) {
  }

protected:

  virtual bool checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const;

};

class AngularCollarLinesFilter : public CollarLinesFilter {
public:
  class Parameters {
  public:
    Parameters(
      int lines_to_preserve_ = 50,
      float horizontal_range_diff_tolerance_rel_ = 1.2,
      float horizontal_range_diff_tolerance_abs_ = 0.2,
      float max_horizontal_range_diff_ = 2.0,
      float weight_of_expected_horizontal_range_diff_ = -1) :
        lines_to_preserve(lines_to_preserve_),
        horizontal_range_diff_tolerance_rel(horizontal_range_diff_tolerance_rel_),
        horizontal_range_diff_tolerance_abs(horizontal_range_diff_tolerance_abs_),
        max_horizontal_range_diff(max_horizontal_range_diff_),
        weight_of_expected_horizontal_range_diff(weight_of_expected_horizontal_range_diff_) {
    }
    int lines_to_preserve;
    float horizontal_range_diff_tolerance_rel;
    float horizontal_range_diff_tolerance_abs;
    float max_horizontal_range_diff;
    float weight_of_expected_horizontal_range_diff;
  };

  AngularCollarLinesFilter(COMPARATION comparation_metric_,
                           Parameters params_,
                           VelodyneSpecification::Model model) :
                            CollarLinesFilter(params_.lines_to_preserve, comparation_metric_),
                            params(params_),
                            max_ring_ranges(VelodyneSpecification::rings(model), 0.0),
                            clouds_processed(0),
                            velodyne_model(model) {
  }

  void addNewMaxRingRanges(std::vector<float> max_ring_ranges_);

protected:
  virtual bool checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const;

  float getExpectedRangesDiff(int ring1, int ring2) const;

private:
  Parameters params;
  std::vector<float> max_ring_ranges;
  int clouds_processed;
  VelodyneSpecification::Model velodyne_model;
};

} /* namespace but_visual_registration */

#endif /* COLLARLINEFILTER_H_ */
