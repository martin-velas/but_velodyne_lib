/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 19/04/2017
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
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class SourcePicker {
public:
  SourcePicker(const PointCloud<PointWithSource> &cloud_) :
    cloud(cloud_) {

    poses_count = -1;
    for(PointCloud<PointWithSource>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      poses_count = MAX(poses_count, pt->source);
    }
    poses_count /= 2;

    vis.getViewer()->registerPointPickingCallback(&SourcePicker::pickPointCallback, *this);
  }

  void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*) {
    int idx = event.getPointIndex();
    sources_to_vis.push_back(cloud[idx].source%poses_count);
    setDataToVisualizer();
  }

  void run(void) {
    setDataToVisualizer();
    vis.show();
  }

protected:
  void setDataToVisualizer() {
    vector< PointCloud<PointWithSource> > vis_clouds(sources_to_vis.size());
    PointCloud<PointWithSource> rest_cloud;
    for(PointCloud<PointWithSource>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      int src = pt->source%poses_count;
      int si;
      for(si = 0; si < sources_to_vis.size(); si++) {
        if(src == sources_to_vis[si]) {
          vis_clouds[si].push_back(*pt);
          break;
        }
      }
      if(si >= sources_to_vis.size()) {
        rest_cloud.push_back(*pt);
      }
    }
    vis.keepOnlyClouds(0)
        .setColor(200, 200, 200).addPointCloud(rest_cloud);
    for(int i = 0; i < sources_to_vis.size(); i++) {
      vis.addPointCloud(vis_clouds[i]);
    }
  }

private:
  const PointCloud<PointWithSource> cloud;
  Visualizer3D vis;
  vector<int> sources_to_vis;
  int poses_count;
};

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  if(argc != 2) {
    cerr << "ERROR, expecting arguments: <cloud-w-sources.pcd>" << endl;
    return EXIT_FAILURE;
  }

  PointCloud<PointWithSource>::Ptr cloud(new PointCloud<PointWithSource>);
  io::loadPCDFile(argv[1], *cloud);
  subsample_cloud<PointWithSource>(cloud, 0.1);

  SourcePicker picker(*cloud);
  picker.run();

  return EXIT_SUCCESS;
}
