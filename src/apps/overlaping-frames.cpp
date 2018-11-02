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
                     int &frames_cumulated,
                     int &lines_generated,
                     int &lines_preserved,
                     int &min_frames_dist, int &max_frames_dist,
                     bool &circular) {
  string pose_filename, sensor_poses_filename, skip_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
      ("frames_cumulated,f", po::value<int>(&frames_cumulated)->default_value(10), "frames_cumulated")
      ("lines_generated,g", po::value<int>(&lines_generated)->default_value(2), "lines_generated")
      ("lines_preserved,k", po::value<int>(&lines_preserved)->default_value(1), "lines_preserved (kept)")
      ("min_frames_dist", po::value<int>(&min_frames_dist)->default_value(0), "min_frames_distance to compute overlap")
      ("max_frames_dist", po::value<int>(&max_frames_dist)->default_value(10000000), "max_frames_dist to compute overlap")
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

void buildLineClouds(VelodyneFileSequence &cloud_sequence,
    const vector<Eigen::Affine3f> &poses,
    const SensorsCalibration &calibration,
    const int frames_cumulated,
    const int lines_per_cell_gen, const int lines_per_cell_preserve,
    vector<LineCloud> &line_clouds,
    vector< KdTreeFLANN<PointXYZ> > &line_trees) {
  int superframes_cnt = cloud_sequence.size()/frames_cumulated;
  line_clouds.resize(superframes_cnt);
  line_trees.resize(superframes_cnt);

  CollarLinesFilter filter(lines_per_cell_preserve);
  for(int superframe_i = 0; superframe_i < superframes_cnt; superframe_i++) {
    for(int frame_i = 0; frame_i < frames_cumulated; frame_i++) {
      const VelodyneMultiFrame multiframe = cloud_sequence.getNext();
      const PolarGridOfClouds polar_grid(multiframe.clouds, calibration);
      LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
      line_cloud.transform(poses[frame_i+superframe_i*frames_cumulated].matrix());
      line_clouds[superframe_i] += line_cloud;
    }
    line_trees[superframe_i].setInputCloud(line_clouds[superframe_i].line_middles.makeShared());
  }
}

void save_normalized_matrix(const string &filename, cv::Mat distances) {
  cv::normalize(distances, distances, 0.0, 255.0, cv::NORM_MINMAX);
  cv::Mat distances_gray;
  distances.convertTo(distances_gray, CV_8UC1);
  cv::imwrite(filename, distances_gray);
}

void getDistance(const PointCloud<PointXYZ> &cloud, const KdTreeFLANN<PointXYZ> &tree,
    float &out_mean, float &out_median, float &out_quarter) {
  out_mean = 0.0;
  vector<int> idx_found(1);
  vector<float> dist_found(1);
  vector<float> distances(cloud.size());
  for(int i = 0; i < cloud.size(); i++) {
    tree.nearestKSearch(cloud[i], 1, idx_found, dist_found);
    float dist = sqrt(dist_found.front());
    out_mean += dist;
    distances[i] = dist;
  }
  out_mean /= cloud.size();
  sort(distances.begin(), distances.end());
  out_median = distances[distances.size()/2];
  out_quarter = distances[distances.size()/4];
}

int get_frames_distance(const int i, const int j, const int frames_cumulated,
    const int frames_count, const bool circular) {
  if(!circular) {
    return abs(i-j)*frames_cumulated;
  } else {
    return MIN(abs(i-j), frames_count-abs(i-j))*frames_cumulated;
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int lines_generated, lines_preserved, frames_cumulated;
  int min_frames_dist, max_frames_dist;
  bool circular;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, frames_cumulated,
      lines_generated, lines_preserved,
      min_frames_dist, max_frames_dist, circular)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);

  vector<LineCloud> line_clouds;
  vector< KdTreeFLANN<PointXYZ> > line_trees;
  buildLineClouds(sequence, poses, calibration, frames_cumulated,
      lines_generated, lines_preserved, line_clouds, line_trees);

  /**
  cv::Mat mean_distances(line_clouds.size(), line_clouds.size(), CV_32FC1, 100.0);
  cv::Mat median_distances(line_clouds.size(), line_clouds.size(), CV_32FC1, 100.0);
  cv::Mat quarter_distances(line_clouds.size(), line_clouds.size(), CV_32FC1, 100.0);
   */
  for(int i = 0; i < line_clouds.size(); i++) {
    //mean_distances.at<float>(i, i) = 0.0;
    for(int j = i+1; j < line_clouds.size(); j++) {
      int frames_distace = get_frames_distance(i, j, frames_cumulated, line_clouds.size(), circular);
      if(frames_distace < min_frames_dist || frames_distace > max_frames_dist) {
        continue;
      }
      float mean, median, quarter;
      getDistance(line_clouds[i].line_middles, line_trees[j], mean, median, quarter);
      /**
      mean_distances.at<float>(i, j) = mean_distances.at<float>(j, i) = mean;
      median_distances.at<float>(i, j) = median_distances.at<float>(j, i) = median;
      quarter_distances.at<float>(i, j) = quarter_distances.at<float>(j, i) = quarter;
       */
      cout << i*frames_cumulated << " " << j*frames_cumulated << " "
           << mean << " " << median << " " << quarter << endl;
    }
  }

  /**
  cv::FileStorage fs("distances.yaml", cv::FileStorage::WRITE);
  fs << "mean_distances" << mean_distances;
  fs << "median_distances" << median_distances;
  fs << "quarter_distances" << quarter_distances;

  save_normalized_matrix("mean_distances.png", mean_distances);
  save_normalized_matrix("median_distances.png", median_distances);
  save_normalized_matrix("quarter_distances.png", quarter_distances);
  **/

  return EXIT_SUCCESS;
}
