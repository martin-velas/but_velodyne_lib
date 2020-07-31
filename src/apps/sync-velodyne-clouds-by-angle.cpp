/*
 * Synchronization of Velodyne LiDAR point clouds using the angle.
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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;


const float FPS = 10.0;

bool parse_arguments(int argc, char **argv, float &start_angle, int &velodyne_idx,
                     string &output_dir, vector<string> &clouds_to_process, float &fps) {

  po::options_description desc("Synchronization of Velodynes in cloud meassurements\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("start_angle,s", po::value<float>(&start_angle)->required(), "Angle [deg] where the frame should start.")
    ("velodyne_idx,i", po::value<int>(&velodyne_idx)->required(), "Index of the Velodyne sensor.")
    ("output_dir,o", po::value<string>(&output_dir)->required(), "Output dir for PCD files.")
    ("fps,f", po::value<float>(&fps)->default_value(10.0), "Velodyne FPS.")
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

// 0/360 [deg] in the front, 90 left, 180 in the back, 270 right
float horizontalAngle(const VelodynePoint &pt) {
  return 360.0 - VelodynePointCloud::horizontalAngle(pt.z, pt.x);
}

class AngleAndPhase {
public:
    AngleAndPhase(const float angle_, const float phase_, const int source_cloud_idx_) :
      angle(angle_), phase(phase_), source_cloud_idx(source_cloud_idx_) {
    }

    bool operator < (const AngleAndPhase &other) const {
      if(this->phase == other.phase) {
        return this->angle < other.angle;
      } else {
        return this->phase < other.phase;
      }
    }

    float angle, phase;
    int source_cloud_idx;
};

ostream& operator << (ostream& stream, const AngleAndPhase &ap) {
  stream << ap.phase << " " << ap.angle << " " << ap.source_cloud_idx;
}

class Interval {
public:
    Interval(const float from_, const float to_) : from(from_), to(to_) {
    }

    float from, to;
};

size_t get_cloud_where_border_fits(const vector<Interval> &raw_borders_sorted, const float border) {
  for(size_t i = 0; i < raw_borders_sorted.size(); i++) {
    if(border < raw_borders_sorted[i].to) {
      return i;
    }
  }
}

void get_borders(const vector<string> &cloud_filenames, const float fps, const float start_angle,
                 vector<AngleAndPhase> &borders) {
  const float frame_duration = 1.0f / fps;

  vector<Interval> raw_frames_borders;
  float start_time;
  for(size_t i = 0; i < cloud_filenames.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(cloud_filenames[i], cloud);
    raw_frames_borders.push_back(Interval(cloud.front().phase, cloud.back().phase));

    if(i == 0) {
      float angle_offset = start_angle - horizontalAngle(cloud.front());
      if(angle_offset < 0) {
        angle_offset += 360.0;
      }
      float phase_offset = angle_offset / 360.0;
      start_time = cloud.front().phase + frame_duration * phase_offset;
    }
  }

  float border;
  for(int i = 0; (border = start_time + i / fps) < raw_frames_borders.back().to; i++) {
    int cloud_idx = get_cloud_where_border_fits(raw_frames_borders, border);
    borders.push_back(AngleAndPhase(start_angle, border, cloud_idx));
  }
}

struct PhaseComparator {
    bool operator()(const VelodynePoint &pt1, const VelodynePoint &pt2) {
      const float a1 = horizontalAngle(pt1);
      const float a2 = horizontalAngle(pt2);
      if(pt1.phase == pt2.phase) {
        return a1 < a2;
      } else {
        return pt1.phase < pt2.phase;
      }
    }
};

void sync_by_angle(const AngleAndPhase &beginning, const AngleAndPhase &end,
                   const VelodynePointCloud &input, VelodynePointCloud &frame_synced) {

  VelodynePointCloud input_sorted;
  input_sorted += input;
  struct PhaseComparator phase_cmp;
  sort(input_sorted.begin(), input_sorted.end(), phase_cmp);
  bool recording = false;
  int i = 0;

  for(VelodynePointCloud::iterator pt = input_sorted.begin(); pt < input_sorted.end(); pt++, i++) {
    float angle = horizontalAngle(*pt);

    if(!recording && pt->phase >= beginning.phase && angle >= beginning.angle) {
      recording = true;
    }

    if(pt->phase >= end.phase && angle >= end.angle) {
      break;
    }

    if(recording) {
      pt->phase = (pt->phase - beginning.phase) / (end.phase - beginning.phase);
      frame_synced.push_back(*pt);
    }
  }
}

int main(int argc, char** argv) {

  float start_angle;
  vector<string> cloud_filenames;
  string output_dir;
  int velodyne_idx;
  float fps;

  if(!parse_arguments(argc, argv, start_angle, velodyne_idx, output_dir, cloud_filenames, fps)) {
    return EXIT_FAILURE;
  }

  vector<AngleAndPhase> borders;
  get_borders(cloud_filenames, fps, start_angle, borders);

  stringstream ss_borders_fn;
  ss_borders_fn << output_dir << "/frame-beginnings." << velodyne_idx << ".txt";
  ofstream borders_fd(ss_borders_fn.str().c_str());
  borders_fd << fixed;

  for(int i = 0; i+1 < borders.size(); i++) {
    const AngleAndPhase &beginning = borders[i];
    const AngleAndPhase &end = borders[i+1];

    VelodynePointCloud raw_cloud;
    for(int raw_i = beginning.source_cloud_idx; raw_i <= end.source_cloud_idx; raw_i++) {
      VelodynePointCloud raw_part;
      VelodynePointCloud::fromFile(cloud_filenames[raw_i], raw_part);
      raw_cloud += raw_part;
    }

    VelodynePointCloud frame_synced;
    sync_by_angle(beginning, end, raw_cloud, frame_synced);

    string fn = output_dir + "/" + KittiUtils::getKittiFrameName(i, ".pcd", velodyne_idx);
    io::savePCDFileBinary(fn, frame_synced);

    borders_fd << beginning << endl;
  }
  borders_fd << borders.back() << endl;

  return EXIT_SUCCESS;
}
