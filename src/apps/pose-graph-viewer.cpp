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
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class Edge {

public:

  Edge(const int src_, const int trg_, const Eigen::Affine3f &t_) :
    src(src_), trg(trg_) , t(t_) {
  }

  Edge(void) :
    src(-1), trg(-1) {
  }

  int dist(void) {
    return abs(trg-src);
  }

  int src, trg;
  Eigen::Affine3f t;
};

const int BATCH_SIZE = 1000;

int main(int argc, char** argv) {

  string line;
  vector<Eigen::Affine3f> poses;
  poses.push_back(Eigen::Affine3f::Identity());
  Edge loop;
  bool loop_found = false;
  vector<Edge> edges;
  while(getline(cin, line)) {
    int src_i, trg_i;
    float tx, ty, tz, rx, ry, rz;
    sscanf(line.c_str(), "EDGE3 %d %d %f %f %f %f %f %f", &src_i, &trg_i, &tx, &ty, &tz, &rx, &ry, &rz);
    Eigen::Affine3f t = getTransformation(tx, ty, tz, rx, ry, rz);
    Edge edge(src_i, trg_i, t);

    if(edge.dist() == 1) {
      if(poses.size() == edge.trg) {
        poses.push_back(poses[edge.src]*edge.t);
      } else {
        PCL_ERROR(("Increasing order broken, line: " + line).c_str());
        return EXIT_FAILURE;
      }
    } else {
      if(!loop_found) {
        loop_found = true;
        loop = edge;
      } else {
        edges.push_back(edge);
      }
    }
  }

  Visualizer3D vis;
  vis.getViewer()->setBackgroundColor(0, 0, 0);

  PointCloudLine loop_arrow(poses[loop.src].translation(),
      poses[loop.trg].translation() - poses[loop.src].translation());

  int edge_idx = 0;
  while(edge_idx < edges.size()) {
    vis.getViewer()->removeAllShapes();
    vis.keepOnlyClouds(0);
    vis.addPosesDots(poses);
    vis.addArrow(loop_arrow);
    PointCloud<PointXYZRGB>::Ptr trg_points(new PointCloud<PointXYZRGB>);
    for(int i = 0; i < BATCH_SIZE && edge_idx < edges.size(); i++, edge_idx++) {
      const Edge &e = edges[edge_idx];
      Eigen::Affine3f correction = e.t.inverse() * poses[e.src].inverse() * poses[e.trg];
      PointXYZ src_pt, trg_pt_original, trg_pt_moved;
      src_pt.getVector3fMap() = poses[e.src].translation();
      trg_pt_original.getVector3fMap() = (poses[e.trg]).translation();
      trg_pt_moved.getVector3fMap() = (poses[e.src]*e.t).translation();
      //PointCloudLine connection(src_pt, trg_pt_original);
      PointCloudLine arrow(trg_pt_original, trg_pt_moved);
      float r = vis.rngF();
      float g = vis.rngF();
      float b = vis.rngF();
      //vis.addLine(connection, r, g, b);
      vis.addLine(arrow, r, g, b);
      PointXYZRGB colot_trg_pt(r*256, g*256, b*256);
      copyXYZ(trg_pt_moved, colot_trg_pt);
      trg_points->push_back(colot_trg_pt);
    }
    vis.addColorPointCloud(trg_points).show();
  }

  return EXIT_SUCCESS;
}
