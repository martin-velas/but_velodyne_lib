/*
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

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointXYZ PointType;


class Plane {
public:
  Plane(void) {}

  Plane(const PointNormal &normal, const PointXYZ &pt) :
    a(normal.x), b(normal.y), c(normal.z), centroid(pt.getVector3fMap()) {
    d = -(a*pt.x + b*pt.y + c*pt.z);
  }

  float distance(const PointType &pt) const {
    return a*pt.x + b*pt.y + c*pt.z + d;
  }

  const Eigen::Vector3f& getCentroid(void) const {
    return centroid;
  }

private:
  float a, b, c, d;
  Eigen::Vector3f centroid;
};


bool parse_arguments(int argc, char **argv,
    vector<string> &input_filenames,
    PointCloud<PointType> &ref_cloud,
    Plane &slice,
    float &slice_width,
    string &out_dir) {

  string ref_cloud_fn, slice_fn;

  po::options_description desc("Evaluation of 4RECON backpack\n"
      "======================================\n"
      " * Reference(s): ? Velas et al, Sensors, 2019 ?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("reference,r", po::value<string>(&ref_cloud_fn)->required(), "Reference point cloud.")
      ("slices,s", po::value<string>(&slice_fn)->required(), "Planes to slice the clouds.")
      ("slice_width,w", po::value<float>(&slice_width)->default_value(0.02), "Slice width.")
      ("out_dir,o", po::value<string>(&out_dir)->required(), "Output dir.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  input_filenames = po::collect_unrecognized(parsed.options, po::include_positional);

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

  if(io::loadPCDFile(ref_cloud_fn, ref_cloud) != 0) {
    return false;
  }

  std::ifstream slice_file(slice_fn.c_str());
  if(!slice_file.is_open()) {
    std::perror((std::string("Unable to open file: ") + slice_fn).c_str());
    return false;
  }

  PointNormal normal;
  PointXYZ pt;
  slice_file >> normal.x >> normal.y >> normal.z >> pt.x >> pt.y >> pt.z;

  if(slice_file.eof()) {
    return false;
  } else {
    slice = Plane(normal, pt);
  }

  return true;
}

int slice_cloud(const PointCloud<PointType> &input, const Plane &plane,
    const float max_distance, PointCloud<PointType> &slice) {
  for(PointCloud<PointType>::const_iterator pt = input.begin(); pt < input.end(); pt++) {
    if(-max_distance < plane.distance(*pt) && plane.distance(*pt) < max_distance) {
      slice.push_back(*pt);
    }
  }
  return slice.size();
}

void get_distances(PointCloud<PointType>::ConstPtr reference, const PointCloud<PointType> &input,
    vector<float> &distances) {
  KdTreeFLANN<PointType> ref_index;
  ref_index.setInputCloud(reference);
  for(PointCloud<PointType>::const_iterator pt = input.begin(); pt < input.end(); pt++) {
    vector<int> indices(1);
    vector<float> sq_distances(1);
    ref_index.nearestKSearch(*pt, 1, indices, sq_distances);
    float distance = sqrt(sq_distances.front());
    distances.push_back(distance);
  }
}

void evaluate(const PointCloud<PointType>::Ptr reference, const PointCloud<PointType> &input,
    ofstream &error_file, PointCloud<PointXYZRGB>::Ptr visualization) {
  KdTreeFLANN<PointType> ref_index;
  ref_index.setInputCloud(reference);

  vector<float> distances;
  float max_distance = 0.05;

  get_distances(reference, input, distances);

  *visualization += *Visualizer3D::colorizeCloud(*reference, 0, 255, 0);
  for(int i = 0; i < input.size(); i++) {
    error_file << distances[i] << endl;
    float distance_normalized = MIN(max_distance, distances[i]) / max_distance;
    PointXYZRGB rgb_pt;
    copyXYZ(input[i], rgb_pt);
    rgb_pt.r = distance_normalized*255;
    rgb_pt.g = 0;
    rgb_pt.b = (1-distance_normalized)*255;
    visualization->push_back(rgb_pt);
  }
}

class CloudEraser {
public:
  CloudEraser(PointCloud<PointXYZRGB>::Ptr cloud_, vector<int> &origins_) :
    cloud(cloud_), origins(origins_), visualizer(Visualizer3D::getCommonVisualizer()) {
    visualizer->getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer->getViewer()->registerAreaPickingCallback(&CloudEraser::pickPointsCallback, *this);
  }

  void run() {
    visualizeCloud();
    visualizer->show();
  }

protected:

  void visualizeCloud() {
    visualizer->keepOnlyClouds(0).addColorPointCloud(cloud);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      if(!indices.empty()) {
        vector<bool> preserve(cloud->size(), true);
        for(vector<int>::iterator i = indices.begin(); i < indices.end(); i++) {
          preserve[*i] = false;
        }
        PointCloud<PointXYZRGB>::iterator c_it = cloud->begin();
        vector<int>::iterator o_it = origins.begin();
        for(int i = 0; i < preserve.size(); i++) {
          if(preserve[i]) {
            c_it++;
            o_it++;
          } else {
            c_it = cloud->erase(c_it);
            o_it = origins.erase(o_it);
          }
        }
        visualizeCloud();
      }
    }
  }

private:
  PointCloud<PointXYZRGB>::Ptr cloud;
  vector<int> &origins;
  Visualizer3D::Ptr visualizer;
};

void erase_outliers(vector < PointCloud<PointType> > &input_slices) {
  PointCloud<PointXYZRGB>::Ptr vis_cloud(new PointCloud<PointXYZRGB>);
  vector<int> origins;

  Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
  for(int si = 0; si < input_slices.size(); si++) {
    PointCloud<PointXYZRGB>::Ptr colored_cloud = Visualizer3D::colorizeCloud(
        input_slices[si], vis->rngU(), vis->rngU(), vis->rngU());
    *vis_cloud += *colored_cloud;
    for(int pi = 0; pi < input_slices[si].size(); pi++) {
      origins.push_back(si);
    }
    input_slices[si].clear();
  }

  CloudEraser eraser(vis_cloud, origins);
  eraser.run();

  for(int i = 0; i < vis_cloud->size(); i++) {
    PointType pt;
    copyXYZ(vis_cloud->at(i), pt);
    input_slices[origins[i]].push_back(pt);
  }
}

int main(int argc, char** argv) {

  vector<string> input_filenames;
  PointCloud<PointType> ref_cloud;
  Plane slice_plane;
  float slice_width;
  string out_dir;

  if(!parse_arguments(argc, argv, input_filenames, ref_cloud, slice_plane,
      slice_width, out_dir)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointType>::Ptr ref_slice(new PointCloud<PointType>);
  slice_cloud(ref_cloud, slice_plane, slice_width/2, *ref_slice);
  Eigen::Affine3f shift_origin = Eigen::Affine3f::Identity();
  shift_origin.translation() = slice_plane.getCentroid();
  transformPointCloud(*ref_slice, *ref_slice, shift_origin.inverse());

  vector < PointCloud<PointType> > input_slices(input_filenames.size());
  for(int ci = 0; ci < input_filenames.size(); ci++) {
    PointCloud<PointType> input_cloud;
    io::loadPCDFile(input_filenames[ci], input_cloud);
    slice_cloud(input_cloud, slice_plane, slice_width/2, input_slices[ci]);
    transformPointCloud(input_slices[ci], input_slices[ci], shift_origin.inverse());
  }

  input_slices.push_back(*ref_slice);
  erase_outliers(input_slices);
  *ref_slice = input_slices.back();
  input_slices.pop_back();

  for(int ci = 0; ci < input_slices.size(); ci++) {
    PointCloud<PointXYZRGB>::Ptr visualization(new PointCloud<PointXYZRGB>);

    boost::filesystem::path cloud_file(input_filenames[ci]);
    string basename = out_dir + "/" + cloud_file.stem().string();

    ofstream error_file((basename + ".txt").c_str());
    evaluate(ref_slice, input_slices[ci], error_file, visualization);
    io::savePCDFileBinary(basename + ".pcd", *visualization);
  }

  return EXIT_SUCCESS;
}
