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
                     string &output_dir, vector<string> &clouds_to_process) {

  po::options_description desc("Synchronization of Velodynes in cloud meassurements\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("start_angle,s", po::value<float>(&start_angle)->required(), "Angle [deg] where the frame should start.")
    ("velodyne_idx,i", po::value<int>(&velodyne_idx)->required(), "Index of the Velodyne sensor.")
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


void get_borders(const vector<string> &cloud_filenames, const float start_angle,
                 vector<AngleAndPhase> &borders) {
  vector<AngleAndPhase> angles_phases;
  const float EPS = 1.0; // [deg]

  for(int i = 0; i < cloud_filenames.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(cloud_filenames[i], cloud);
    for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      const float angle = horizontalAngle(*pt);
      if(start_angle-EPS < angle && angle < start_angle+EPS) {
        angles_phases.push_back(AngleAndPhase(angle, pt->phase, i));
      }
    }
  }
  sort(angles_phases.begin(), angles_phases.end());

  float previous_angle = angles_phases.front().angle;
  for(vector<AngleAndPhase>::const_iterator ap = angles_phases.begin(); ap < angles_phases.end(); ap++) {
    if(previous_angle < start_angle && start_angle <= ap->angle &&
       (borders.empty() || ((borders.back().phase + (1.0/FPS)/2.0) < ap->phase))) {
      borders.push_back(*ap);
    }
    previous_angle = ap->angle;
  }
}

struct {
    bool operator()(const VelodynePoint &pt1, const VelodynePoint &pt2) {
      const float a1 = horizontalAngle(pt1);
      const float a2 = horizontalAngle(pt2);
      if(pt1.phase == pt2.phase) {
        return a1 < a2;
      } else {
        return pt1.phase < pt2.phase;
      }
    }
} phase_cmp;

void sync_by_angle(const AngleAndPhase &beginning, const AngleAndPhase &end,
                   const VelodynePointCloud &input, VelodynePointCloud &frame_synced) {
  //static Visualizer3D vis;

  VelodynePointCloud input_sorted;
  input_sorted += input;
  sort(input_sorted.begin(), input_sorted.end(), phase_cmp);
  bool recording = false;
  int i = 0;
  // cerr << "Beginning: [" << beginning << "], end: [" << end << "]" << endl;
  for(VelodynePointCloud::iterator pt = input_sorted.begin(); pt < input_sorted.end(); pt++, i++) {
    float angle = horizontalAngle(*pt);
    // cerr << "iff [" << beginning << "] <= " << pt->phase << ", " << angle << " < [" << end << "]" << endl;
    if(!recording && pt->phase >= beginning.phase && angle >= beginning.angle) {
      recording = true;
    }
    if(pt->phase >= end.phase && angle >= end.angle) {
      break;
    }
    if(recording) {
      // float original_phase = pt->phase;
      pt->phase = (pt->phase - beginning.phase) / (end.phase - beginning.phase);
      frame_synced.push_back(*pt);
      /* if(i % 231 == 0) {
        char text[32];
        sprintf(text, "%.2f/%.2f", original_phase, angle);
        uchar r, g, b;
        Visualizer3D::colorizeIntensity(pt->phase, r, g, b);
        vis.getViewer()->addText3D(text, *pt, 0.05, r/255.0, g/255.0, b/255.0, text);
      }*/
    }
  }

  /* vis.keepOnlyClouds(0)
    .addColorPointCloud(Visualizer3D::colorizeCloudByPhase(frame_synced))
    .show();
  vis.getViewer()->removeAllShapes();*/
}

int main(int argc, char** argv) {

  float start_angle;
  vector<string> cloud_filenames;
  string output_dir;
  int velodyne_idx;

  if(!parse_arguments(argc, argv, start_angle, velodyne_idx, output_dir, cloud_filenames)) {
    return EXIT_FAILURE;
  }

  vector<AngleAndPhase> borders;
  get_borders(cloud_filenames, start_angle, borders);
  cerr << "Borders found: " << borders.size() << endl;

  stringstream ss_borders_fn;
  ss_borders_fn << output_dir << "/frame-beginnings." << velodyne_idx << ".txt";
  ofstream borders_fd(ss_borders_fn.str());

  for(int i = 0; i+1 < borders.size(); i++) {
    const AngleAndPhase &beginning = borders[i];
    const AngleAndPhase &end = borders[i+1];

    if(end.source_cloud_idx - beginning.source_cloud_idx > 1) {
      cerr << "ERROR! Frame skipped between the borders [" << beginning << "] and [" << end << "]" << endl;
      exit(1);
    }

    VelodynePointCloud raw_cloud;
    VelodynePointCloud::fromFile(cloud_filenames[beginning.source_cloud_idx], raw_cloud);
    if(beginning.source_cloud_idx != end.source_cloud_idx) {
      VelodynePointCloud second_half;
      VelodynePointCloud::fromFile(cloud_filenames[end.source_cloud_idx], second_half);
      raw_cloud += second_half;
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
