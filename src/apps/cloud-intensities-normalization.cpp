/*
 * Intensity normalization for Velodyne data.
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
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/common.h>
#include <but_velodyne/GlobalOptimization.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/random_sample.hpp>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

static const float ANGULAR_RES = 0.02;
static const float DISTANCE_RES = 0.2;

typedef PointWithSource PointType;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     string &sum_cloud_filename,
                     string &normals_filename, string &indices_filename,
                     float &expected_mean, float &expected_std_dev, string &out_filename) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
    ("sum_cloud_filename,c", po::value<string>(&sum_cloud_filename)->required(), "Input cloud PDC file.")
    ("normals_filename,n", po::value<string>(&normals_filename)->required(), "Normals filename.")
    ("indices_filename,i", po::value<string>(&indices_filename)->required(), "Indices filename.")
    ("expected_mean,m", po::value<float>(&expected_mean)->default_value(300.0), "Target mean of intensities.")
    ("expected_std_dev,d", po::value<float>(&expected_std_dev)->default_value(1.0), "Target standard deviation of intensities.")
    ("out_filename,o", po::value<string>(&out_filename)->required(), "Output normalized cloud filename.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
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

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

class BinIndex {
public:
  int ring, angle, dist;

  BinIndex(const int ring_, const int angle_, const int dist_) :
    ring(ring_), angle(angle_), dist(dist_) {
  }

  bool operator=(const BinIndex &o) const {
    return this->ring == o.ring && this->angle == o.angle && this->dist == o.dist;
  }

  bool operator<(const BinIndex &o) const {
    if(this->ring == o.ring) {
      if(this->angle == o.angle) {
        return this->dist < o.dist;
      } else {
        return this->angle < o.angle;
      }
    } else {
      return this->ring < o.ring;
    }
  }
};

float transform_gauss(float x1, float u1, float s1, float u2, float s2) {
  float x2;
  if(s1 < 0.01) {
    x2 = x1 - u1 + u2;
  } else {
    x2 = ((x1-u1) / s1) * s2 + u2;
  }

  /*cerr << "x1: " << x1 << "; " <<
      "u1: " << u1 << "; " <<
      "s1: " << s1 << "; " <<
      "u2: " << u2 << "; " <<
      "s2: " << s2 << "; " <<
      "x2: " << x2 << endl;*/

  return MIN(MAX(x2, 0.0), 2*u2);
}

void normalize_intensities(const Mat &data, PointCloud<PointType> &cloud,
    float expected_mean, float expected_std_dev) {
  typedef map< BinIndex, vector<float> > GridT;
  GridT desc_grid;
  for(int i = 0; i < data.rows; i++) {
    BinIndex bin(data.at<int>(i, 0), data.at<int>(i, 1), data.at<int>(i, 2));
    desc_grid[bin].push_back(cloud[i].intensity);
  }

  for(GridT::iterator bin = desc_grid.begin(); bin != desc_grid.end(); bin++) {
    float mean = accumulate(bin->second.begin(), bin->second.end(), 0.0) / bin->second.size();
    float variance = 0;
    for(vector<float>::iterator x = bin->second.begin(); x < bin->second.end(); x++) {
      variance += pow(*x-mean, 2);
    }
    variance /= bin->second.size();
    bin->second.clear();
    bin->second.push_back(mean);
    bin->second.push_back(sqrt(variance));
  }

  for(int i = 0; i < data.rows; i++) {
    BinIndex bin(data.at<int>(i, 0), data.at<int>(i, 1), data.at<int>(i, 2));
    cloud[i].intensity = transform_gauss(cloud[i].intensity,
        desc_grid[bin][0], desc_grid[bin][1],
        expected_mean, expected_std_dev);
  }
}

int quantize(float value, float resolution) {
  return floor(value/resolution);
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  string sum_cloud_filename;
  SensorsCalibration calibration;
  string normals_filename, indices_filename, out_filename;
  float expected_mean, expected_std_dev;
  if(!parse_arguments(argc, argv,
      poses, calibration,
      sum_cloud_filename,
      normals_filename, indices_filename,
      expected_mean, expected_std_dev, out_filename)) {
    return EXIT_FAILURE;
  }

  cerr << "Loading points ..." << endl;
  PointCloud<PointType>::Ptr sum_cloud(new PointCloud<PointType>);
  io::loadPCDFile(sum_cloud_filename, *sum_cloud);

  PointCloud<PointXYZ> sum_origin_positions;
  sum_origin_positions.resize(sum_cloud->size());
  for(int i = 0; i < sum_cloud->size(); i++) {
    Origin o = Origin::fromPointSource(sum_cloud->at(i).source, poses.size());
    sum_origin_positions[i].getVector3fMap() = calibration.getSensorPose(poses[o.pose_id], o.sensor_id).translation();
  }
  cerr << "Computed " << sum_origin_positions.size() << " origins." << endl;

  cerr << "Loading normals ..." << endl;
  PointCloud<Normal> subsampled_normals;
  io::loadPCDFile(normals_filename, subsampled_normals);

  cerr << "Loading indices, labels & subsampling ..." << endl;
  PointIndices::Ptr indices(new PointIndices);
  load_vector_from_file(indices_filename, indices->indices);

  cerr << "Compute data for normalization ..." << endl;
  Mat data(indices->indices.size(), 3, CV_32SC1);
  //Visualizer3D vis;
  PointCloud<PointType>::Ptr vis_cloud(new PointCloud<PointType>);
  *vis_cloud += *sum_cloud;
  subsample_cloud<PointType>(vis_cloud, 0.01);
  //vis.setColor(200, 200, 200).addPointCloud(*vis_cloud);
  //cv::RNG& rng(cv::theRNG());
  for(int ni = 0; ni < indices->indices.size(); ni++) {
    int pi = indices->indices[ni];
    PointType pt = sum_cloud->at(pi);
    Eigen::Vector3f pt_to_origin = sum_origin_positions[pi].getVector3fMap() - pt.getVector3fMap();
    float distance = pt_to_origin.norm();
    Eigen::Vector3f normal = subsampled_normals[ni].getNormalVector3fMap();
    float angle = acos(pt_to_origin.dot(normal) / (pt_to_origin.norm()*normal.norm()));

    /*cout << pt.ring << " " << angle << " " << distance << " " << pt.intensity << endl;

    if(angle > 1.8) {
      cerr << "sensor: " << sum_origins[pi].sensor_id << endl;
      PointCloudLine normal_line(pt.getVector3fMap(), normal);
      PointCloudLine pt_to_origin_line(pt.getVector3fMap(), pt_to_origin);
      unsigned r = rng(256);
      unsigned g = rng(256);
      unsigned b = rng(256);
      vis.getViewer()->removeAllShapes();
      vis.setColor(r, g, b).addArrow(normal_line);
      vis.setColor(r, g, b).addArrow(pt_to_origin_line).show();
    }*/

    data.at<int>(ni, 0) = pt.ring;
    data.at<int>(ni, 1) = quantize(angle, ANGULAR_RES);
    data.at<int>(ni, 2) = quantize(distance, DISTANCE_RES);
  }

  cerr << "Intensities normalization ..." << endl;
  PointCloud<PointType> subsampled_cloud;
  extract_indices(sum_cloud, indices, subsampled_cloud);
  normalize_intensities(data, subsampled_cloud, expected_mean, expected_std_dev);

  cerr << "Saving output ..." << endl;
  io::savePCDFileBinary(out_filename, subsampled_cloud);

  return EXIT_SUCCESS;
}
