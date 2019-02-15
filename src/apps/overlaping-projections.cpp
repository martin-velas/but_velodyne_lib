/*
 * Overlaps estimation among point clouds for CLS registration.
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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     int &frames_dist, bool &circular,
                     float &depth_quantile,
                     float &depth_relative_tolerance, float &depth_absolute_tolerance,
                     bool &visualize) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("frames_dist", po::value<int>(&frames_dist)->required(), "Frames distance to compute overlap")
    ("circular", po::bool_switch(&circular), "The trajectory is considered to be circular.")
    ("depth_quantile", po::value<float>(&depth_quantile)->default_value(0.0),
        "Quantile of depth within the bin (0.5 for median, 0.0 for minimum, 1.0 for maximum.")
    ("depth_relative_tolerance", po::value<float>(&depth_relative_tolerance)->default_value(0.1),
        "Relative tolerance of the depth in Z-buffer.")
    ("depth_absolute_tolerance", po::value<float>(&depth_absolute_tolerance)->default_value(0.3),
        "Absolute tolerance of the depth in Z-buffer.")
    ("visualize", po::bool_switch(&visualize), "Run visualization.")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensor_poses_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensor_poses_filename);
  }

  return true;
}

const float DEG_TO_RAD = M_PI / 180.0;
const float RAD_TO_DEG = 180.0 / M_PI;

const float EPS = 0.0001;
const float AZIMUTHAL_RANGE = 360.0;
const float POLAR_RANGE = 180.0;

float keepInRange(const float value, const float min, const float max, const float eps) {
  return MIN( MAX(value, min+eps), max-eps );
}

void pointToSpherical(const PointXYZ &pt, float &azimuth, float &polar, float &range) {
  azimuth = std::atan2(pt.z, pt.x) * RAD_TO_DEG;
  azimuth = keepInRange(azimuth, -AZIMUTHAL_RANGE/2, AZIMUTHAL_RANGE/2, EPS);
  float horizontal_range = sqrt(pt.x * pt.x + pt.z * pt.z);
  polar = atan(-pt.y / horizontal_range) * RAD_TO_DEG;
  polar = keepInRange(polar, -POLAR_RANGE/2, POLAR_RANGE/2, EPS);
  range = pt.getVector3fMap().norm();
}

void sphericalToPoint(const float azimuth, const float polar, const float range, PointXYZ &pt) {
  pt.y = - sin(polar*DEG_TO_RAD)*range;
  float horizontal_range = cos(polar*DEG_TO_RAD)*range;
  pt.z = sin(azimuth*DEG_TO_RAD)*horizontal_range;
  pt.x = cos(azimuth*DEG_TO_RAD)*horizontal_range;
}

class SphericalZbuffer {
public:
  typedef boost::shared_ptr<SphericalZbuffer> Ptr;

  SphericalZbuffer(const PointCloud<PointXYZ> &cloud_,
      const int azimuthal_bins_, const int polar_bins_, const float depth_quantile) :
      azimuthal_bins(azimuthal_bins_), polar_bins(polar_bins_),
      azimuthal_resolution(AZIMUTHAL_RANGE / azimuthal_bins_), polar_resolution(POLAR_RANGE / polar_bins_),
      depths(azimuthal_bins_*polar_bins_), visited(azimuthal_bins_*polar_bins_, false) {
    vector< vector<float> > depth_sets(azimuthal_bins*polar_bins);
    for(PointCloud<PointXYZ>::const_iterator pt = cloud_.begin(); pt < cloud_.end(); pt++) {
      float azimuth, polar, range;
      pointToSpherical(*pt, azimuth, polar, range);
      int idx = getIndex(azimuth, polar);
      if(idx >= depth_sets.size()) {
        cerr << azimuth << " " << polar << endl;
        cerr << idx << " " << depth_sets.size() << endl;
      }
      depth_sets[idx].push_back(range);
    }

    cells_occupied = 0;
    for(int i = 0; i < depths.size(); i++) {
      if(depth_sets[i].size() < 5) {
        depths[i] = -1;
      } else {
        sort(depth_sets[i].begin(), depth_sets[i].end());
        depths[i] = depth_sets[i][depth_sets[i].size()*depth_quantile];
        cells_occupied++;
      }
    }
  }

  float getDepth(const float azimuth, const float polar_angle) const {
    return depths[getIndex(azimuth, polar_angle)];
  }

  size_t containsPoints(const PointCloud<PointXYZ> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance) {
    size_t result = 0;
    for(PointCloud<PointXYZ>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      if(this->containsPoint(*pt, depth_relative_tolerance, depth_absolute_tolerance)) {
        result++;
      }
    }
    return result;
  }

  size_t containsPoints(const PointCloud<PointXYZ> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance,
      PointCloud<PointXYZ> &overlap, PointCloud<PointXYZ> &rest) {
    for(PointCloud<PointXYZ>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      if(this->containsPoint(*pt, depth_relative_tolerance, depth_absolute_tolerance)) {
        overlap.push_back(*pt);
      } else {
        rest.push_back(*pt);
      }
    }
    return overlap.size();
  }

  bool containsPoint(const PointXYZ &point,
    const float depth_relative_tolerance, const float depth_absolute_tolerance) {
    float azimuth, polar_angle, range;
    pointToSpherical(point, azimuth, polar_angle, range);
    this->visit(azimuth, polar_angle);
    const float zdepth = this->getDepth(azimuth, polar_angle);
    return (zdepth*(1.0 + depth_relative_tolerance) + depth_relative_tolerance) > range;
  }

  void addToVisualizer(Visualizer3D &visualizer, const PointCloud<PointXYZ> &src_cloud,
      const float depth_relative_tolerance = 0.0, const float depth_absolute_tolerance = 0.0) const {
    PointCloud<PointXYZ> vis_cloud;
    vis_cloud.resize(src_cloud.size());
    for(int i = 0; i < src_cloud.size(); i++) {
      float azimuth, polar_angle, range;
      pointToSpherical(src_cloud[i], azimuth, polar_angle, range);
      sphericalToPoint(azimuth, polar_angle,
          this->getDepth(azimuth, polar_angle)*(1.0 + depth_relative_tolerance) + depth_absolute_tolerance,
          vis_cloud[i]);
    }
    visualizer.addPointCloud(vis_cloud);
  }

  float visitedPortion(void) const {
    int visited_cnt = 0;
    for(int i = 0; i < visited.size(); i++) {
      if(visited[i] && depths[i] > 0) {
        visited_cnt++;
      }
    }
    return (float(visited_cnt)) / cells_occupied;
  }

protected:
  void setDepth(const float azimuth, const float polar_angle, const float depth) {
    depths[getIndex(azimuth, polar_angle)] = depth;
  }

  void visit(const float azimuth, const float polar_angle) {
    visited[getIndex(azimuth, polar_angle)] = true;
  }

  int getIndex(const float azimuth, const float polar_angle) const {
    int aidx = floor((azimuth+AZIMUTHAL_RANGE/2.0) / azimuthal_resolution);
    int pidx = floor((polar_angle+POLAR_RANGE/2.0) / polar_resolution);

    int result = MIN(aidx, azimuthal_bins-1) * polar_bins + MIN(pidx, polar_bins-1);

    if(result >= depths.size() || result < 0) {
      cerr << "WARNING - SPHERICAL BINS OVERFLOW!!!" << endl;
      cerr << "azimuthal_bins: " << azimuthal_bins << endl;
      cerr << "polar_bins: " << polar_bins << endl;
      cerr << "azimuthal_resolution: " << azimuthal_resolution << endl;
      cerr << "polar_resolution: " << polar_resolution << endl;

      cerr << "azimuth: " << azimuth << endl;
      cerr << "polar_angle: " << polar_angle << endl;

      cerr << "aidx: " << aidx << endl;
      cerr << "pidx: " << pidx << endl;
      cerr << "result: " << result << endl;
      cerr << "--------------------" << endl;
    }

    return int(aidx) * polar_bins + int(pidx);
  }

private:
  const int azimuthal_bins, polar_bins;
  const float azimuthal_resolution, polar_resolution;
  vector<float> depths;
  vector<bool> visited;
  int cells_occupied;
};

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular) {
  if(!circular) {
    return abs(i-j);
  } else {
    return MIN(abs(i-j), frames_count-abs(i-j));
  }
}

float harmonic_avg(const float a, const float b) {
  return 2.0*a*b/(a+b);
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int expected_frames_dist;
  bool circular;
  float depth_quantile, depth_relative_tolerance, depth_absolute_tolerance;
  bool visualize;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, expected_frames_dist, circular, depth_quantile,
      depth_relative_tolerance, depth_absolute_tolerance, visualize)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  Visualizer3D::Ptr vis;

  for(int i = 0; i < sequence.size(); i++) {
    for(int j = i+1; j < sequence.size(); j++) {
      int frames_distace = get_frames_distance(i, j, sequence.size(), circular);
      if(frames_distace == expected_frames_dist) {
        VelodyneMultiFrame src_frame = sequence[i];
        PointCloud<PointXYZ> src_cloud;
        src_frame.joinTo(src_cloud);
        transformPointCloud(src_cloud, src_cloud, Eigen::Affine3f(poses[i].rotation()));
        SphericalZbuffer src_zbuffer(src_cloud, 72, 36, depth_quantile);

        VelodyneMultiFrame trg_frame = sequence[j];
        PointCloud<PointXYZ> trg_cloud;
        trg_frame.joinTo(trg_cloud);
        transformPointCloud(trg_cloud, trg_cloud, Eigen::Affine3f(poses[j].rotation()));
        Eigen::Vector3f translation = poses[j].translation() - poses[i].translation();
        transformPointCloud(trg_cloud, trg_cloud, translation, Eigen::Quaternionf::Identity());

        size_t points_within;
        if(visualize) {
          if(!vis) {
            vis.reset(new Visualizer3D);
          }
          PointCloud<PointXYZ> within, rest;
          points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance,
              within, rest);
          vis->keepOnlyClouds(0)
              .setColor(0, 0, 0).addPointCloud(src_cloud)
              .setColor(0, 200, 0).addPointCloud(within)
              .setColor(255, 0, 0).addPointCloud(rest)
              .setColor(255, 0, 255);
          src_zbuffer.addToVisualizer(*vis, src_cloud, depth_relative_tolerance, depth_absolute_tolerance);
        } else {
          points_within = src_zbuffer.containsPoints(trg_cloud, depth_relative_tolerance, depth_absolute_tolerance);
        }

        float visited_portion = src_zbuffer.visitedPortion();
        float points_within_portion = (float (points_within)) / trg_cloud.size();

        cout << i << " " << j << " " << harmonic_avg(visited_portion, points_within_portion) << endl;

        if(visualize) {
          vis->show();
        }
      }
    }
  }

  return EXIT_SUCCESS;
}
