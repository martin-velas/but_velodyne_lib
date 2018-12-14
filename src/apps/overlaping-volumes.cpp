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
                     int &frames_dist, bool &circular) {
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

void pointToSpherical(const PointXYZ &pt, float &azimuth, float &polar, float &range) {
  azimuth = std::atan2(pt.z, pt.x) * RAD_TO_DEG;
  float horizontal_range = sqrt(pt.x * pt.x + pt.z * pt.z);
  polar = atan(-pt.y / horizontal_range) * RAD_TO_DEG;
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
  SphericalZbuffer(const PointCloud<PointXYZ> &cloud_, const int azimuthal_bins_, const int polar_bins_) :
      azimuthal_bins(azimuthal_bins_), polar_bins(polar_bins_),
      azimuthal_resolution(AZIMUTHAL_RANGE / azimuthal_bins_), polar_resolution(POLAR_RANGE / polar_bins_),
      depths(azimuthal_bins_*polar_bins_, INFINITY) {
    for(PointCloud<PointXYZ>::const_iterator pt = cloud_.begin(); pt < cloud_.end(); pt++) {
      float azimuth, polar, range;
      pointToSpherical(*pt, azimuth, polar, range);
      setDepth(azimuth, polar, MIN(range, getDepth(azimuth, polar)));
    }
  }

  float getDepth(const float azimuth, const float polar_angle) const {
    return depths[getIndex(azimuth, polar_angle)];
  }

protected:
  void setDepth(const float azimuth, const float polar_angle, const float depth) {
    depths[getIndex(azimuth, polar_angle)] = depth;
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
  static const float AZIMUTHAL_RANGE = 360.0;
  static const float POLAR_RANGE = 180.0;

  const int azimuthal_bins, polar_bins;
  const float azimuthal_resolution, polar_resolution;
  vector<float> depths;
};

class SparseOccupancyArray {

public:

  void set(const size_t idx) {
    occupied_cells.insert(idx);
  }

  bool get(const size_t idx) const {
    return occupied_cells.count(idx) > 0;
  }

private:
  std::set<size_t> occupied_cells;

};

class OccupancyGrid {
public:
  OccupancyGrid(const float max_range_, const float resolution_) :
      max_range(max_range_), resolution(resolution_),
      bins_per_axis(max_range_*2.0/resolution_),
      occupied_count(0) {
    occupancy_grid_size = pow(bins_per_axis, 3);
  }

  //PointCloud<PointXYZ> overlaping_midpoints;

  void fill(const SphericalZbuffer &zbuffer) {
    for(int i = 0; i < occupancy_grid_size; i++) {
      float azimuth, polar, range;
      getMidpointSpherical(i, azimuth, polar, range);
      float zdepth = zbuffer.getDepth(azimuth, polar);
      if(!isinf(zdepth) && range < zdepth) {
        occupancy.set(i);
        occupied_count++;
        //midpoints.push_back(midpoint);
      }
    }
  }

  float computeOverlapWith(const OccupancyGrid &other, const Eigen::Vector3f &translation) {
    Eigen::Vector3i offset =  getOffset(-translation);
    //cerr << "offset: " << offset.x() << " " << offset.y() << " " << offset.z() << endl;

    //overlaping_midpoints.clear();
    int overlaping_cells = 0;
    for(int xi = 0; xi < bins_per_axis; xi++) {
      int other_xi = xi + offset.x();
      if(0 <= other_xi && other_xi < bins_per_axis) {

        for(int yi = 0; yi < bins_per_axis; yi++) {
          int other_yi = yi + offset.y();
          if(0 <= other_yi && other_yi < bins_per_axis) {

            for(int zi = 0; zi < bins_per_axis; zi++) {
              int other_zi = zi + offset.z();
              if(0 <= other_zi && other_zi < bins_per_axis) {

                const int this_idx = getIndex(xi, yi, zi);
                const int other_idx = getIndex(other_xi, other_yi, other_zi);
                if(this->occupancy.get(this_idx) && other.occupancy.get(other_idx)) {
                  overlaping_cells++;
                  //overlaping_midpoints.push_back(getMidpoint(this_idx));
                }
              }
            }
          }
        }
      }
    }

    //return 2.0 * overlaping_cells / (this->occupied_count + other.occupied_count);
    return (float (overlaping_cells)) / other.occupied_count;
  }

protected:
  int getAxisIndex(const float axis_coordinate) const {
    if(-max_range < axis_coordinate && axis_coordinate < max_range) {
      return floor((axis_coordinate + max_range) / resolution);
    } else {
      return -1;
    }
  }

  Eigen::Vector3i getOffset(const Eigen::Vector3f &translation) const {
    Eigen::Vector3f offset_float = translation / resolution;
    return Eigen::Vector3i(offset_float.x(), offset_float.y(), offset_float.z());
  }

  int getIndex(const PointXYZ &pt) const {
    int xi, yi, zi;
    if(getIndex(pt, xi, yi, zi)) {
      return getIndex(xi, yi, zi);
    } else {
      return -1;
    }
  }

  int getIndex(int &xi, int &yi, int &zi) const {
    return (xi*bins_per_axis + yi) * bins_per_axis + zi;
  }

  bool getIndex(const PointXYZ &pt, int &xi, int &yi, int &zi) const {
    xi = getAxisIndex(pt.x);
    yi = getAxisIndex(pt.y);
    zi = getAxisIndex(pt.z);
    return xi >= 0 && yi >= 0 && zi >= 0;;
  }

  float getAxisCoordinate(const int index) const {
    return index * resolution - max_range;
  }

  const PointXYZ& getMidpoint(int index) const {
    if(cells_midpoints.empty()) {
      cells_midpoints.resize(occupancy_grid_size);
      cells_midpoints_spherical.resize(occupancy_grid_size);
      for(int i = 0; i < occupancy_grid_size; i++) {
        int zi = i % bins_per_axis;
        int mod_i = i / bins_per_axis;
        int yi = mod_i % bins_per_axis;
        int xi = mod_i / bins_per_axis;

        cells_midpoints[i].x = getAxisCoordinate(xi)+resolution/2.0;
        cells_midpoints[i].y = getAxisCoordinate(yi)+resolution/2.0;
        cells_midpoints[i].z = getAxisCoordinate(zi)+resolution/2.0;

        pointToSpherical(cells_midpoints[i],
            cells_midpoints_spherical[i].x, cells_midpoints_spherical[i].y, cells_midpoints_spherical[i].z);
      }
    }
    return cells_midpoints[index];
  }

  void getMidpointSpherical(const int i, float &azimuth, float &polar, float &range) {
    if(cells_midpoints_spherical.empty()) {
      getMidpoint(i);
    }
    const PointXYZ &pt = cells_midpoints_spherical[i];
    azimuth = pt.x;
    polar = pt.y;
    range = pt.z;
  }

private:
  size_t occupancy_grid_size;
  SparseOccupancyArray occupancy;
  static PointCloud<PointXYZ> cells_midpoints;
  static PointCloud<PointXYZ> cells_midpoints_spherical;
  int occupied_count;
  float max_range, resolution;
  int bins_per_axis;
};

PointCloud<PointXYZ> OccupancyGrid::cells_midpoints;
PointCloud<PointXYZ> OccupancyGrid::cells_midpoints_spherical;

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular) {
  if(!circular) {
    return abs(i-j);
  } else {
    return MIN(abs(i-j), frames_count-abs(i-j));
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int frames_dist;
  bool circular;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, frames_dist, circular)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);
  vector<OccupancyGrid> occupancy_grids;
  vector< PointCloud<PointXYZ> > clouds;
  //Visualizer3D vis;
  for(int i = 0; i < poses.size() && sequence.hasNext(); i++) {
    PointCloud<PointXYZ> cloud;
    sequence.getNext().joinTo(cloud);
    transformPointCloud(cloud, cloud, Eigen::Affine3f(poses[i].rotation()));
    clouds.push_back(cloud);
    SphericalZbuffer zbuffer(cloud, 72, 36);
    occupancy_grids.push_back(OccupancyGrid(10.0, 0.2));
    occupancy_grids.back().fill(zbuffer);
  }

  for(int i = 0; i < occupancy_grids.size(); i++) {
    for(int j = i+1; j < occupancy_grids.size(); j++) {
      int frames_distace = get_frames_distance(i, j, occupancy_grids.size(), circular);
      if(frames_distace == frames_dist) {
        Eigen::Vector3f translation = poses[j].translation() - poses[i].translation();
        float overlap = occupancy_grids[i].computeOverlapWith(occupancy_grids[j], translation);
        cout << i << " " << j << " " << overlap << endl;
        /*vis.keepOnlyClouds(0)
           .addPointCloud(clouds[i])
           .addPointCloud(clouds[j], getTransformation(translation.x(), translation.y(), translation.z(), 0, 0, 0).matrix())
           .addPointCloud(occupancy_grids[i].overlaping_midpoints)
           .show();*/
      }
    }
  }

  return EXIT_SUCCESS;
}
