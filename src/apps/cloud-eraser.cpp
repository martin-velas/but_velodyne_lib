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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

typedef PointXYZ PointType;

class CloudEraser {
public:
  CloudEraser(const PointCloud<PointType>::Ptr cloud_, PointIndices::Ptr input_indices_) :
    cloud(cloud_), indices(input_indices_) {
    visualizer.getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer.getViewer()->registerAreaPickingCallback(&CloudEraser::pickPointsCallback, *this);
  }

  PointIndices::Ptr run() {
    visualizeCloud();
    visualizer.show();
    return indices;
  }

protected:

  void visualizeCloud() {
    PointCloud<PointType> vis_cloud;
    extract_indices(cloud, indices, vis_cloud);
    visualizer.keepOnlyClouds(0).addPointCloud(vis_cloud);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices_to_remove;
    if(event.getPointsIndices(indices_to_remove)) {
      cerr << "Picked " << indices_to_remove.size() << " points" << endl;

      int counter = 0;
      for(vector<int>::iterator i = indices_to_remove.begin(); i < indices_to_remove.end(); i++) {
        indices->indices[*i] = -1;
        counter++;
      }
      cerr << counter << " indices marked" << endl;

      int copy_to_index = 0;
      for(vector<int>::iterator i = indices->indices.begin(); i < indices->indices.end(); i++) {
        if(*i >= 0) {
          indices->indices[copy_to_index] = *i;
          copy_to_index++;
        }
      }
      indices->indices.resize(copy_to_index);

      cerr << "Indices processed." << endl;

      visualizeCloud();
    }
  }

private:
  PointCloud<PointType>::Ptr cloud;
  Visualizer3D visualizer;
  PointIndices::Ptr indices;
};

int main(int argc, char** argv) {

  string src_filename, trg_filename;

  if(argc != 3) {
    cerr << "ERROR, expected arguments: [in_cloud.pcd] [init.indices]" << endl;
    return EXIT_FAILURE;
  }

  PointCloud<PointType>::Ptr in_cloud(new PointCloud<PointType>);
  io::loadPCDFile(argv[1], *in_cloud);

  PointIndices::Ptr indices(new PointIndices);
  load_vector_from_file(argv[2], indices->indices);

  CloudEraser eraser(in_cloud, indices);
  PointIndices::Ptr out_indices = eraser.run();

  for(vector<int>::iterator i = out_indices->indices.begin(); i < out_indices->indices.end(); i++) {
    cout << *i << endl;
  }

  return EXIT_SUCCESS;
}
