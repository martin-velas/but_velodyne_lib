/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
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

#include <cstdlib>
#include <cstdio>

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;


class TimeSpan {
public:

  bool overlaps(const TimeSpan &other) const {
    return (this->from <= other.from && other.from <= this->to) ||
           (this->from <= other.to && other.to <= this->to) ||
           (other.from <= this->from && this->from <= other.to) ||
           (other.from <= this->to && this->to <= other.to);
  }

  float from, to;
};


bool parse_arguments(int argc, char **argv,
                     string &frame_borders_file, int &velodyne_idx,
                     int &extra_hours_elapsed,
                     string &output_dir, vector<string> &clouds_to_process) {

  po::options_description desc("Synchronization of Velodynes in cloud meassurements\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("frame_borders,b", po::value<string>(&frame_borders_file)->required(), "File with expected timestamps last rows is time of termination.")
      ("velodyne_idx,i", po::value<int>(&velodyne_idx)->default_value(-1), "Index of the Velodyne sensor.")
      ("extra_hours_elapsed", po::value<int>(&extra_hours_elapsed)->default_value(0), "Extra hours elapsed for Velodyne sensor.")
      ("output_dir,o", po::value<string>(&output_dir)->required(), "Output dir for PCD files.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_process.size() < 1) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  return true;
}

class SequenceSlicer {

public:

  SequenceSlicer(const vector<string> &cloud_filenames_) : cloud_filenames(cloud_filenames_) {
    cloud_spans.resize(cloud_filenames.size());
    for(int i = 0; i < cloud_filenames.size(); i++) {
      VelodynePointCloud cloud;
      VelodynePointCloud::fromFile(cloud_filenames[i], cloud);
      cloud_spans[i].from = 1000000;
      cloud_spans[i].to = -1;
      for(VelodynePointCloud::const_iterator p = cloud.begin(); p < cloud.end(); p++) {
        cloud_spans[i].from = MIN(cloud_spans[i].from, p->phase);
        cloud_spans[i].to = MAX(cloud_spans[i].to, p->phase);
      }
    }
  }

  bool getSlice(const TimeSpan &slice_span, VelodynePointCloud &slice) {
    assert(cloud_filenames.size() == cloud_spans.size());

    cerr << fixed << slice_span.from << " " << slice_span.to << " " << cloud_spans.front().from << " "  << cloud_spans.back().to << endl;

    float min_phase = INFINITY;
    float max_phase = -INFINITY;

    for(int i = 0; i < cloud_filenames.size(); i++) {
      if(slice_span.overlaps(cloud_spans[i])) {
        VelodynePointCloud raw_cloud;
        VelodynePointCloud::fromFile(cloud_filenames[i], raw_cloud);
        for(VelodynePointCloud::const_iterator p = raw_cloud.begin(); p < raw_cloud.end(); p++) {
          if(slice_span.from <= p->phase && p->phase <= slice_span.to) {
            slice.push_back(*p);
            min_phase = MIN(min_phase, p->phase);
            max_phase = MAX(max_phase, p->phase);
          }
        }
      }
    }

    for(VelodynePointCloud::iterator p = slice.begin(); p < slice.end(); p++) {
      p->phase = (p->phase - min_phase) / (max_phase - min_phase + 0.0001);
    }

    cerr << "--------------------------------------------------" << endl;

    return !slice.empty();
  }

private:

  vector<TimeSpan> cloud_spans;
  vector<string> cloud_filenames;
};


int main(int argc, char** argv) {

  vector<string> cloud_filenames;
  string frame_borders_filename, output_dir;
  int velodyne_idx;
  int extra_hours_elapsed;

  if(!parse_arguments(argc, argv, frame_borders_filename, velodyne_idx, extra_hours_elapsed,
          output_dir, cloud_filenames)) {
    return EXIT_FAILURE;
  }


  SequenceSlicer slicer(cloud_filenames);

  vector<double> frame_borders;
  load_vector_from_file(frame_borders_filename, frame_borders);
  for(int i = 0; i+1 < frame_borders.size(); i++) {
    VelodynePointCloud slice;
    TimeSpan slice_span;
    slice_span.from = frame_borders[i] + 3600*extra_hours_elapsed;
    slice_span.to = frame_borders[i+1] + 3600*extra_hours_elapsed;
    bool slice_found = slicer.getSlice(slice_span, slice);
    if(slice_found) {
      string fn = output_dir + "/" + KittiUtils::getKittiFrameName(i, ".pcd", velodyne_idx);
      io::savePCDFileBinary(fn, slice);
    } else {
      PCL_ERROR("ERROR - slice not found!");
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
