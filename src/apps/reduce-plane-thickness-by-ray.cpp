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
#include <but_velodyne/point_types.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

//#define VISUALIZE 1

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &input_fn, string &normals_fn, string &normals_indices_fn,
                     vector<Eigen::Affine3f> &poses, SensorsCalibration &calibration,
                     float &wall_thickness_th, float &dist_to_line_threshold,
                     string &output_fn) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Optimization of the wall thickness\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_points,i", po::value<string>(&input_fn)->required(), "Input pcd file.")
    ("normals,n", po::value<string>(&normals_fn)->required(), "Input normals.")
    ("normals_indices", po::value<string>(&normals_indices_fn)->required(), "Point indices, where normals are estimated.")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("wall_thickness_th,t", po::value<float>(&wall_thickness_th)->default_value(0.1), "Wall thickness threshold.")
    ("dist_to_line_threshold,d", po::value<float>(&dist_to_line_threshold)->default_value(0.02), "Distance to line threshold.")
    ("output_cloud,o", po::value<string>(&output_fn)->required(), "Output pcd file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
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

class RayThinner {
public:
  RayThinner(PointCloud<PointWithSource>::ConstPtr cloud_,
      const PointCloud<Normal> &normals_,
      const float wall_thickness_th_, const float line_dist_th_) :
    cloud(cloud_), wall_thickness_th(wall_thickness_th_), line_dist_th(line_dist_th_) {
    tree.setInputCloud(cloud);
    normals.resize(normals_.size());
    for(int i = 0; i < normals.size(); i++) {
      normals[i] = normals_[i].getNormalVector3fMap();
    }
  }

  void findReplacementOriginal(const int input_idx, const PointXYZ &origin,
      PointXYZI &out) {
    const PointWithSource &in = cloud->at(input_idx);
    const Eigen::Vector3f &in_normal = normals[input_idx];
    PointXYZ in_xyz;
    copyXYZ(in, in_xyz);
    PointCloudLine pt_to_origin(in_xyz, origin);
    vector<int> indices;
    vector<float> distances;
    float dot = in_normal.dot(pt_to_origin.orientation);
    float radius_scale = MIN(in_normal.norm() * pt_to_origin.orientation.norm() / dot, 5.7);    // 80deg threshold
    float radius_th = wall_thickness_th * radius_scale;
    //cerr << input_idx << " radius_th: " << radius_th << " dot: " << dot;
    //cerr << " " << in_xyz << " " << origin << " " << in_normal << endl;
    tree.radiusSearch(in, radius_th, indices, distances);
    int counter = 0;
    out.x = out.y = out.z = out.intensity = 0.0;
#ifdef VISUALIZE
    PointCloud<PointWithSource> pts;
#endif
    for(vector<int>::const_iterator i = indices.begin(); i < indices.end(); i++) {
      const PointWithSource &pt = cloud->at(*i);
      if((pt_to_origin.distanceTo(pt) < line_dist_th) &&
         (in_normal.dot(normals[*i]) > 0)) {
        out.x += pt.x;
        out.y += pt.y;
        out.z += pt.z;
        out.intensity += pt.intensity;
        counter++;
#ifdef VISUALIZE
        pts.push_back(pt);
#endif
      }
    }
    out.x /= counter;
    out.y /= counter;
    out.z /= counter;
    out.intensity /= counter;
#ifdef VISUALIZE
    PointCloud<PointXYZI> out_vis;
    out_vis.push_back(out);
    vis.getViewer()->removeAllShapes();
    vis.addLine(pt_to_origin);
    vis.keepOnlyClouds(0)
        .setColor(200, 200, 200).addPointCloud(*cloud)
        .setColor(50, 50, 200).addPointCloud(pts)
        .setColor(200, 0, 0).addPointCloud(out_vis).show();
#endif
  }

  class RangeDenoiser {
  public:
    RangeDenoiser(const float radius_th_, const float wall_thickness_th_) :
      RADIUS_TH(radius_th_), BINS_CNT(RADIUS_TH*2 / RESOLUTION),
      AFFECTED_BINS_RADIUS(wall_thickness_th_ / RESOLUTION),
      histogram(BINS_CNT, 0) {
    }

    float bin2dist(const int bin_i) const {
      return bin_i*RESOLUTION - RADIUS_TH;
    }

    int dist2bin(const float dist) const {
      return (dist + RADIUS_TH)/RESOLUTION;
    }

    void addPoint(const float range_diff) {
      int middle_bin = dist2bin(range_diff);
      for(int i = MAX(middle_bin - AFFECTED_BINS_RADIUS, 0);
          i < MIN(middle_bin + AFFECTED_BINS_RADIUS + 1, BINS_CNT); i++) {
        histogram[i]++;
      }
      //cerr << "range_diff: " << range_diff << "; middle_bin: " << middle_bin << endl;
    }

    float getBestDist(void) const {
      float max_likelyhood = -INFINITY;
      int best_bin;
      stringstream ss_dist, ss_hist, ss_join;
      for(int i = 0; i < BINS_CNT; i++) {
        float dist = RADIUS_TH-fabs(bin2dist(i));
        float dist_times_hist = dist*histogram[i];
        if(dist_times_hist > max_likelyhood) {
          max_likelyhood = dist_times_hist;
          best_bin = i;
        }
        /*cerr << dist << "|";
        cerr << histogram[i] << "|";
        cerr << dist_times_hist << " ";
        ss_dist << dist << " ";
        ss_hist << histogram[i] << " ";
        ss_join << dist_times_hist << " ";*/
      }
      /*cerr << endl;
      cerr << ss_dist.str() << endl;
      cerr << ss_hist.str() << endl;
      cerr << ss_join.str() << endl;*/

      return bin2dist(best_bin);
    }

    static const float RESOLUTION = 0.01;
    const float RADIUS_TH;
    const int BINS_CNT, AFFECTED_BINS_RADIUS;
  private:
    vector<int> histogram;
  };

  void findReplacement(const int input_idx, const PointXYZ &origin,
      PointXYZI &out) {
    const PointWithSource &in = cloud->at(input_idx);
    const Eigen::Vector3f &in_normal = normals[input_idx];
    PointXYZ in_xyz;
    copyXYZ(in, in_xyz);
    const PointCloudLine pt_to_origin(in_xyz, origin);
    Eigen::Vector3f origin_to_pt_unit = -pt_to_origin.orientation;
    origin_to_pt_unit.normalize();
    float in_range = pt_to_origin.orientation.norm();
    vector<int> indices;
    vector<float> distances;
    static const float RADIUS_TH = 1.0;  // 1m
    tree.radiusSearch(in, RADIUS_TH, indices, distances);

    float dot = in_normal.dot(pt_to_origin.orientation);
    float radius_scale = MIN(in_normal.norm() * pt_to_origin.orientation.norm() / dot, 5.7);    // 80deg threshold

#ifdef VISUALIZE
    PointCloud<PointWithSource> pts;
#endif

    static const float COS45 = sqrt(2.0)/2.0;
    RangeDenoiser denoiser(RADIUS_TH, wall_thickness_th*radius_scale);
    for(vector<int>::const_iterator i = indices.begin(); i < indices.end(); i++) {
      const PointWithSource &pt = cloud->at(*i);
      if((pt_to_origin.distanceTo(pt) < line_dist_th) &&
         (in_normal.dot(normals[*i]) > COS45)) {
        float range = (pt.getVector3fMap() - origin.getVector3fMap()).dot(origin_to_pt_unit);
        denoiser.addPoint(range - in_range);
#ifdef VISUALIZE
        cerr << "range: " << range << "; in_range: " << in_range << endl;
        pts.push_back(pt);
#endif
      }
    }

    float best_dist = denoiser.getBestDist();
    //cerr << "best_dist: " << best_dist << endl;
    out.getVector3fMap() = origin.getVector3fMap() + origin_to_pt_unit*(in_range + best_dist);

#ifdef VISUALIZE
    PointCloud<PointXYZI> out_vis;
    out_vis.push_back(out);
    vis.getViewer()->removeAllShapes();
    vis.addLine(pt_to_origin);
    vis.keepOnlyClouds(0)
        .setColor(200, 200, 200).addPointCloud(*cloud)
        .setColor(50, 50, 200).addPointCloud(pts)
        .setColor(200, 0, 0).addPointCloud(out_vis).show();
#endif
  }

private:
  KdTreeFLANN<PointWithSource> tree;
  PointCloud<PointWithSource>::ConstPtr cloud;
  vector<Eigen::Vector3f> normals;
  const float wall_thickness_th, line_dist_th;
#ifdef VISUALIZE
    Visualizer3D vis;
#endif
};

void get_origin(const vector<Eigen::Affine3f> &poses, const SensorsCalibration &calibration,
    const int origin_idx, PointXYZ &origin) {
  const int pose_idx = origin_idx % poses.size();
  const int sensor_idx = origin_idx / poses.size();
  origin.getVector3fMap() = calibration.getSensorPose(poses[pose_idx], sensor_idx).translation();
  //cerr << "origin: " << origin_idx << " " << pose_idx << " " << sensor_idx << " " << origin.getVector3fMap() << endl;
}

int main(int argc, char** argv) {

  string input_fn, output_fn, normals_fn, normals_indices_fn;
  float wall_thickness_th, dist_to_line_threshold;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;

  if(!parse_arguments(argc, argv,
      input_fn, normals_fn, normals_indices_fn,
      poses, calibration,
      wall_thickness_th, dist_to_line_threshold,
      output_fn)) {
    return EXIT_FAILURE;
  }

  cerr << "Loading ... ";

  PointCloud<PointWithSource>::Ptr in_cloud(new PointCloud<PointWithSource>);
  PointCloud<PointXYZI> out_cloud;
  io::loadPCDFile(input_fn, *in_cloud);

  PointCloud<Normal> normals;
  io::loadPCDFile(normals_fn, normals);

  PointIndices::Ptr indicies(new PointIndices);
  load_vector_from_file(normals_indices_fn, indicies->indices);

  extract_indices(in_cloud, indicies, *in_cloud);

  cerr << "DONE" << endl;

  cerr << "Thinning process started ..." << endl;
  RayThinner thinner(in_cloud, normals, wall_thickness_th, dist_to_line_threshold);
  out_cloud.resize(in_cloud->size());
  int percent = in_cloud->size() / 100;
  for(int i = 0; i < in_cloud->size(); i++) {
    const PointWithSource &in = in_cloud->at(i);
    PointXYZ origin;
    get_origin(poses, calibration, in.source, origin);
    thinner.findReplacement(i, origin, out_cloud[i]);
    if(i%percent == 0) {
      cerr << "[" << i/percent << "%] of points processed" << endl;
    }
  }

  cerr << "Orignal cloud has " << out_cloud.size() << " points" << endl;

  vector<int> notnan;
  removeNaNFromPointCloud(out_cloud, out_cloud, notnan);
  io::savePCDFileBinary(output_fn, out_cloud);

  cerr << "After NaNs removal, " << out_cloud.size() << " points left" << endl;

  return EXIT_SUCCESS;
}
