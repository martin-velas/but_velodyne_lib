/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 01/05/2015
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

#ifndef KITTIUTILS_H_
#define KITTIUTILS_H_

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>

namespace but_velodyne
{

std::istream& operator>>(std::istream &stream, Eigen::Affine3f &pose);

std::ostream& operator<<(std::ostream &stream, const Eigen::Affine3f &pose);

/**!
 * Auxiliary class for handling KITTI pose files
 */
class KittiUtils
{
public:

  /**!
   * Load odometry from KITTI pose file.
   *
   * @param poses_filename input KITTI pose file
   * @param fail_when_not_found call exit(.) when file not found
   * @return poses of sensory platform for whole data sequence
   */
  static std::vector<Eigen::Affine3f> load_kitti_poses(const std::string poses_filename,
                                                       bool fail_when_not_found = true) {
    std::vector<Eigen::Affine3f> poses;
    std::ifstream poses_file(poses_filename.c_str());
    if(!poses_file.is_open()) {
      std::perror((std::string("Unable to open file: ") + poses_filename).c_str());
      if(fail_when_not_found) {
        exit(1);
      } else {
        return poses;
      }
    }

    while(true) {
      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      poses_file >> pose;

      if(poses_file.eof()) {
        break;
      } else {
        poses.push_back(pose);
      }
    }

    return poses;
  }

  /**!
   * Load odometry from KITTI pose file.
   *
   * @param poses_filename input KITTI pose file
   * @return timed poses of sensory platform for whole data sequence
   */
  template <typename K>
  static bool load_kitti_poses_with_keys(const std::string poses_filename,
      std::map<K, Eigen::Affine3f> &poses_timed) {

    std::ifstream poses_file(poses_filename.c_str());
    if(!poses_file.is_open()) {
      std::perror((std::string("Unable to open file: ") + poses_filename).c_str());
      return false;
    }

    while(true) {
      K time;
      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      poses_file >> time >> pose;

      if(poses_file.eof()) {
        break;
      } else {
        poses_timed[time] = pose;
      }
    }

    return true;
  }

  /**!
   * Save multiple odometry poses to the file.
   *
   * @param poses the poses to save
   * @param stream target output stream
   */
  static void save_kitti_poses(const std::vector<Eigen::Affine3f> &poses,
                               std::ostream &stream) {
    for(std::vector<Eigen::Affine3f>::const_iterator p = poses.begin();
        p < poses.end(); p++) {
      save_kitti_pose(*p, stream);
    }
  }

  /**!
   * Save single poses to the file.
   *
   * @param pose the pose to save
   * @param stream target output stream
   */
  static void save_kitti_pose(const Eigen::Affine3f &pose, std::ostream &stream) {
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 4; c++) {
        stream << pose.matrix()(r, c) << " ";
      }
    }
    stream << std::endl;
  }

  /**!
   * Pose to PCL point conversion - rotation is omitted.
   */
  static pcl::PointXYZ positionFromPose(const Eigen::Affine3f &pose) {
    pcl::PointXYZ position(0.0, 0.0, 0.0);
    return pcl::transformPoint(position, pose);
  }

  /**!
   * @param index index of data frame in sequence
   * @return filename where LiDAR data (point cloud) is stored
   */
  static std::string getKittiFrameName(const int index, const std::string &suffix = ".bin", const int sensor_id = -1) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << index;
    if(sensor_id >= 0) {
      ss << "." << sensor_id;
    }
    ss << suffix;
    return ss.str();
  }

  static size_t kittiNameToIndex(const std::string &fn) {
    boost::filesystem::path cloud_fn(fn);
    return atoi(cloud_fn.filename().c_str());
  }

	static void printPose(std::ostream &ofs, const Eigen::Matrix4f &pose) {
	  ofs << Eigen::Affine3f(pose) << std::endl;
	}
};

} /* namespace but_velodyne */

#endif /* KITTIUTILS_H_ */
