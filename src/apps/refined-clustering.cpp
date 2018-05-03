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
#include <but_velodyne/Clustering.h>
#include <but_velodyne/NormalsEstimation.h>
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &input_fn,
                     string &normals_fn,
                     string &indices_fn,
                     int &gmms_count, float &gmm_train_ratio, int &cluster_size,
                     string &output_fn) {

  po::options_description desc("Optimization of the wall thickness\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_cloud,i", po::value<string>(&input_fn)->required(), "Input pcd file.")
    ("input_normals,n", po::value<string>(&normals_fn)->required(), "File with input normals (subsampled).")
    ("input_indices", po::value<string>(&indices_fn)->required(), "Indices of subsampling.")
    ("gmm_train_ratio,r", po::value<float>(&gmm_train_ratio)->default_value(0.1), "GMM train ratio.")
    ("gmms_count,g", po::value<int>(&gmms_count)->default_value(10), "Number of GMMs used.")
    ("cluster_size,c", po::value<int>(&cluster_size)->default_value(30000), "Expected points within final cluster.")
    ("output_cloud,o", po::value<string>(&output_fn)->required(), "Output pcd file.")
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

  return true;
}

int main(int argc, char** argv) {

  string input_fn, indices_fn, normals_fn, output_fn;
  int gmms_count, cluster_size;
  float gmm_train_ratio;

  if(!parse_arguments(argc, argv,
      input_fn, normals_fn, indices_fn,
      gmms_count, gmm_train_ratio, cluster_size,
      output_fn)) {
    return EXIT_FAILURE;
  }

  cerr << "Loading ... ";

  PointCloud<PointXYZI>::Ptr in_cloud(new PointCloud<PointXYZI>);
  io::loadPCDFile(input_fn, *in_cloud);

  PointCloud<Normal>::Ptr subsampled_normals(new PointCloud<Normal>);
  io::loadPCDFile(normals_fn, *subsampled_normals);

  PointIndices::Ptr subsampling_indicies(new PointIndices);
  load_vector_from_file(indices_fn, subsampling_indicies->indices);

  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  extract_indices(in_cloud, subsampling_indicies, *subsampled_cloud);

  cerr << "[DONE]" << endl << "Clustering ... ";

  vector<int> indices;
  vector<float> probabilities;
  Clustering<PointXYZI> clustering;
  clustering.clusterEM(*subsampled_cloud, *subsampled_normals, gmms_count, indices, probabilities, gmm_train_ratio);

  cerr << "[DONE]" << endl << "Refining ... ";
  vector<int> refined_indices;
  clustering.refineByKMeans(*subsampled_cloud, cluster_size, indices, refined_indices);

  cerr << "[DONE]" << endl << "Saving ... ";

  PointCloud<LabeledPoint> out_cloud;
  colorByClusters(*subsampled_cloud, refined_indices, probabilities, out_cloud);
  io::savePCDFileBinary(output_fn, out_cloud);

  cerr << "[DONE]" << endl;

  return EXIT_SUCCESS;
}
