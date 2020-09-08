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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

inline PointCloud<PointXYZI>& operator += (PointCloud<PointXYZI> &this_cloud, const VelodynePointCloud& other)
{
  // Make the resultant point cloud take the newest stamp
  if (other.header.stamp > this_cloud.header.stamp)
    this_cloud.header.stamp = other.header.stamp;

  size_t nr_points = this_cloud.points.size ();
  this_cloud.points.resize (nr_points + other.points.size ());
  for (size_t i = nr_points; i < this_cloud.points.size (); ++i) {
    copyXYZ(other.points[i - nr_points], this_cloud.points[i]);
    this_cloud.points[i].intensity = other.points[i - nr_points].intensity;
  }

  this_cloud.width    = static_cast<uint32_t>(this_cloud.points.size ());
  this_cloud.height   = 1;
  if (other.is_dense && this_cloud.is_dense)
    this_cloud.is_dense = true;
  else
    this_cloud.is_dense = false;
  return this_cloud;
}

void normalizeIntensity(const PointCloud<PointXYZI> &in, PointCloud<PointXYZI> &out) {
  out.resize(in.size());
  float mean = 0;
  for(int i = 0; i < in.size(); i++) {
    copyXYZ(in[i], out[i]);
    mean += in[i].intensity;
  }

  mean /= in.size();

  for(int i = 0; i < in.size(); i++) {
    if(mean < 0.5) {
      out[i].intensity = in[i].intensity/mean*0.5;
    } else {
      out[i].intensity = (in[i].intensity-mean)/(1-mean)*0.5 + 0.5;
    }
  }
}

void colorizeCloudByPrediction(const PointCloud<PointXYZI> &cloud, PointCloud<PointXYZRGB>::Ptr cloud_colored) {
  cloud_colored->resize(cloud.size());
  for(int i = 0; i < cloud.size(); i++) {
    PointXYZRGB &pt = cloud_colored->at(i);
    copyXYZ(cloud[i], pt);
    pt.r = pt.g = pt.b = 0;
    if(cloud[i].intensity < 0) {    // missing
      pt.r = pt.g = pt.b = 0;
    } else if(cloud[i].intensity > 0.5) {
      pt.r = 140*cloud[i].intensity+100;
      pt.g = pt.b = (1-cloud[i].intensity)*340+15;
    } else {
      pt.r = pt.g = pt.b = 140*cloud[i].intensity+115;
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " [-p <poses>] <point-cloud>+" << endl;
    return 1;
  }

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  int history_size = 1;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-p") == 0 && (i < argc - 1)) {
      i++;
      poses = KittiUtils::load_kitti_poses(argv[i]);
    } else if (strcmp(argv[i], "-h") == 0 && (i < argc - 1)) {
      i++;
      history_size = atoi(argv[i]);
    } else {
      filenames.push_back(string(argv[i]));
    }
  }

  boost::circular_buffer< pcl::PointCloud<pcl::PointXYZI>::Ptr > history(history_size);

  Visualizer3D visualizer;
  visualizer.getViewer()->setSize(1280, 720);
  visualizer.getViewer()->setBackgroundColor(0, 0, 0);
  visualization::Camera camera;
  camera.clip[0] = 0.183382;
  camera.clip[1] = 183.382;
  camera.focal[0] = -0.753805;
  camera.focal[1] = 3.82932;
  camera.focal[2] = -0.663768;
  camera.pos[0] = 3.67662;
  camera.pos[1] = -12.366;
  camera.pos[2] = -28.857;
  camera.view[0] = -0.0634353;
  camera.view[1] = -0.869631;
  camera.view[2] = 0.489611;
  visualizer.getViewer()->setCameraPosition(
      camera.pos[0], camera.pos[1], camera.pos[2],
      camera.focal[0], camera.focal[1], camera.focal[2],
      camera.view[0], camera.view[1], camera.view[2]);
  visualizer.getViewer()->setCameraClipDistances(camera.clip[0], camera.clip[1]);

  for (int i = 0; i < filenames.size(); i++) {

    std::cerr << "Processing KITTI file: " << filenames[i] << std::endl << std::flush;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(filenames[i], *current);
    history.push_front(current);

    if (!poses.empty() && i > 0) {
      PointCloud<PointXYZI>::Ptr sum_cloud(new PointCloud<PointXYZI>);
      *sum_cloud += *current;
      Eigen::Affine3f delta = poses[i-1].inverse() * poses[i];
      for(int j = 1; j < history.size(); j++) {
        transformPointCloud(*history[j], *history[j], delta.inverse());
        *sum_cloud += *history[j];
      }
      cerr << "|sum_cloud| = " << sum_cloud->size() << endl;

      if (history.full()) {
        pcl::VoxelGrid<PointXYZI> grid;
        grid.setLeafSize(0.1, 0.1, 0.1);
        grid.setInputCloud(sum_cloud);
        grid.filter(*sum_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colorizeCloudByPrediction(*sum_cloud, rgb_cloud);

        stringstream ss;
        ss << i << ".pcd";
        visualizer.keepOnlyClouds(0).addColorPointCloud(rgb_cloud).saveSnapshot(ss.str());
      }
    }
  }

  return EXIT_SUCCESS;
}
