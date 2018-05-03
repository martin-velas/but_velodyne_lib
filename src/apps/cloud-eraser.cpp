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

class CloudEraser {
public:
  CloudEraser(const PointCloud<PointXYZRGB>::Ptr &cloud_) :
    cloud(cloud_) {
    vector<int> indices;
    removeNaNFromPointCloud(*cloud, *cloud, indices);
    preserve_mask.resize(cloud->size(), 1);

    visualizer.getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer.getViewer()->registerAreaPickingCallback(&CloudEraser::pickPointsCallback, *this);
    visualizer.getViewer()->registerKeyboardCallback(&CloudEraser::keyCallback, *this);
  }

  PointCloud<PointXYZRGB>::Ptr run() {
    visualizeCloud();
    visualizer.show();
    PointCloud<PointXYZRGB>::Ptr cloud_erased(new PointCloud<PointXYZRGB>);
    vector<int> indices;
    for(int i = 0; i < preserve_mask.size(); i++) {
      if(preserve_mask[i] > 0) {
        indices.push_back(i);
      }
    }
    copyPointCloud(*cloud, indices, *cloud_erased);
    return cloud_erased;
  }

protected:

  void visualizeCloud() {
    PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);
    *cloud_colored += *cloud;
    for(int i = 0; i < preserve_mask.size(); i++) {
      if(preserve_mask[i] == 0) {
        cloud_colored->at(i).r = cloud_colored->at(i).g = cloud_colored->at(i).b = 0;
      } else if(preserve_mask[i] < 0) {
        cloud_colored->at(i).r = 200;
        cloud_colored->at(i).g = cloud_colored->at(i).b = 0;
      }
    }
    visualizer.keepOnlyClouds(0).addColorPointCloud(cloud_colored);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      for(vector<int>::iterator i = indices.begin(); i < indices.end(); i++) {
        preserve_mask[*i] = -1;
      }
      visualizeCloud();
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "d") {
        for(vector<int>::iterator i = preserve_mask.begin(); i < preserve_mask.end(); i++) {
          *i = MAX(*i, 0);
        }
        visualizeCloud();
      } else if(event.getKeySym() == "u") {
        for(vector<int>::iterator i = preserve_mask.begin(); i < preserve_mask.end(); i++) {
          *i = abs(*i);
        }
        visualizeCloud();
      }
    }
  }

private:
  PointCloud<PointXYZRGB>::Ptr cloud;
  Visualizer3D visualizer;
  vector<int> preserve_mask;
};

int main(int argc, char** argv) {

  string src_filename, trg_filename;

  if(argc != 3) {
    cerr << "ERROR, expected arguments: [in_cloud.pcd] [out_cloud.pcd]" << endl;
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZRGB>::Ptr in_cloud(new PointCloud<PointXYZRGB>);
  io::loadPCDFile(argv[1], *in_cloud);

  CloudEraser eraser(in_cloud);
  PointCloud<PointXYZRGB>::Ptr out_cloud = eraser.run();

  io::savePCDFileBinary(argv[2], *out_cloud);

  return EXIT_SUCCESS;
}
