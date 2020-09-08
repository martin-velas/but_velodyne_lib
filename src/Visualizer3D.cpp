/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 23/01/2015
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

#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

namespace but_velodyne
{

using namespace cv;
using namespace pcl;
using namespace Eigen;

boost::shared_ptr<Visualizer3D> Visualizer3D::commonVisualizer;

Visualizer3D::Visualizer3D(pcl::visualization::PCLVisualizer::Ptr viewer_) :
    rng(cv::theRNG()),
    viewer(viewer_),
    identifier(0),
    color_index(0),
    point_size(1)
{
  viewer->setBackgroundColor(1.0, 1.0, 1.0);
  viewer->addCoordinateSystem(0.5);
  viewer->initCameraParameters();
  viewer->setCameraPosition(5, -5, 0, 0, 0, 0);
  viewer->addText3D("x", PointXYZ(0.5, 0, 0), 0.1, 0.5, 0, 0.5, getId("text"));
  viewer->addText3D("y", PointXYZ(0, 0.5, 0), 0.1, 0.5, 0, 0.5, getId("text"));
  viewer->addText3D("z", PointXYZ(0, 0, 0.5), 0.1, 0.5, 0, 0.5, getId("text"));
}

Visualizer3D::~Visualizer3D() {
  close();
}

Visualizer3D& Visualizer3D::addColorPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const Eigen::Matrix4f &transformation, int viewport) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cloud, *cloud_transformed, transformation);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_vis(cloud_transformed);
  std::string id = getId("cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_transformed, rgb_vis, id, viewport);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                       point_size, id);
  return *this;
}

Visualizer3D& Visualizer3D::addColorPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
    const Eigen::Matrix4f &transformation, int viewport) {

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*cloud, *cloud_transformed, transformation);
  pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb_vis(cloud_transformed);
  std::string id = getId("cloud");
  viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_transformed, rgb_vis, id, viewport);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                       point_size, id);
  return *this;
}

Visualizer3D& Visualizer3D::addCloudColoredByRing(const VelodynePointCloud &cloud, const Eigen::Matrix4f &transformation) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    uchar r, g, b;
    r = pt->ring / ((float) cloud.ringCount()) * 255;
    g = b = 0;

    pcl::PointXYZRGB rgb_pt;
    copyXYZ(*pt, rgb_pt);
    rgb_pt.r = r;
    rgb_pt.g = g;
    rgb_pt.b = b;
    rgb_cloud->push_back(rgb_pt);
  }

  return this->addColorPointCloud(rgb_cloud, transformation);
}

Visualizer3D& Visualizer3D::addLine(const Correspondence3D &line) {
  viewer->addLine(line.source, line.target, rngF(), rngF(), rngF(), getId("line"));
  return *this;
}

Visualizer3D& Visualizer3D::addLine(const PointCloudLine &line) {
  return addLine(line, rngF(), rngF(), rngF());
}

Visualizer3D& Visualizer3D::addLine(const PointCloudLine &line,
                                    float r, float g, float b) {
  viewer->addLine(line.getBeginPoint(), line.getEndPoint(),
                  r, g, b, getId("line"));
  return *this;
}

Visualizer3D& Visualizer3D::addArrow(const PointCloudLine &line) {
  viewer->addArrow(line.getEndPoint(), line.getBeginPoint(),
      rngF(), rngF(), rngF(), false, getId("arrow"));
  return *this;
}

Visualizer3D& Visualizer3D::addArrow(const PointCloudLine &line, string &id) {
  id = getId("arrow");
  viewer->addArrow(line.getEndPoint(), line.getBeginPoint(),
      rngF(), rngF(), rngF(), false, id);
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<Correspondence3D> &lines) {
  for (vector<Correspondence3D>::const_iterator l = lines.begin(); l < lines.end(); l++)
  {
    addLine(*l);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<PointCloudLine> &lines,
                                     float r, float g, float b) {
  for (vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++)
  {
    addLine(*l, r, g, b);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<PointCloudLine> &lines) {
  for (vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++) {
    addLine(*l);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const LineCloud &lineCloud) {
  for (int i = 0; i < lineCloud.size(); i++) {
    addLine(lineCloud[i].line);
  }
  return *this;
}

void Visualizer3D::saveSnapshot(const std::string &filename) {
  viewer->saveScreenshot(filename);
}

Visualizer3D& Visualizer3D::addSenzor(PointXYZ position) {
  string name = "sensor";
  float radius = 1.5;
  viewer->removeShape(name);
  position.y = -2 * radius;
  viewer->addSphere(position, 3, 1.0, 1.0, 0, name);
  return *this;
}

Visualizer3D& Visualizer3D::keepOnlyClouds(int clouds_to_preserve) {
  vector<string> old_ids = all_identifiers;
  all_identifiers.clear();
  int preserved = 0;
  for(int i = old_ids.size() - 1; i >= 0; i--) {
    string id = old_ids[i];
    if(id.find("cloud") != string::npos) {
      if(preserved < clouds_to_preserve) {
        preserved++;
        all_identifiers.push_back(id);
      } else {
        viewer->removePointCloud(id);
      }
    } else {
      all_identifiers.push_back(id);
    }
  }
  reverse(all_identifiers.begin(), all_identifiers.end());
  return *this;
}

Visualizer3D& Visualizer3D::addPosesLoops(const vector<Eigen::Affine3f> &poses,
					  std::vector<cv::DMatch> matches) {
  PointXYZ to(0, 0, 0);
  for(int i = 1; i < poses.size(); i++) {
    PointXYZ from = to;
    to = KittiUtils::positionFromPose(poses[i]);
    this->addLine(PointCloudLine(from, to), .7, .7, .7);
  }

  for(int i = 0; i < matches.size(); i++) {
    PointXYZ from = KittiUtils::positionFromPose(poses[matches[i].queryIdx]);
    PointXYZ to = KittiUtils::positionFromPose(poses[matches[i].trainIdx]);
    this->addArrow(PointCloudLine(from, to));
  }

  return *this;
}

Visualizer3D& Visualizer3D::addPosesDots(const vector<Eigen::Affine3f> &poses, int viewport) {
  PointCloud<PointXYZ> poses_cloud = posesToPoints(poses);
  return addPointCloud(poses_cloud, Eigen::Matrix4f::Identity(), viewport);
}

Visualizer3D& Visualizer3D::addPoses(const vector<Eigen::Affine3f> &poses, float axis_size, int viewport) {
  for(vector<Eigen::Affine3f>::const_iterator p = poses.begin(); p < poses.end(); p++) {
    this->getViewer()->addCoordinateSystem(axis_size, *p, getId("cs"), viewport);
  }
  return *this;
}

Visualizer3D& Visualizer3D::setColor(unsigned r, unsigned g, unsigned b) {
  color_stack.push_back(b);
  color_stack.push_back(g);
  color_stack.push_back(r);
  return *this;
}

Visualizer3D& Visualizer3D::addRingColoredCloud(const VelodynePointCloud &cloud) {
  PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
  for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    uchar red, green, blue;
    red = green = blue = 0.0;
    if(pt->ring%3 == 0) {
      red = 255;
    } else if(pt->ring%3 == 1) {
      green = 255;
    } else {
      blue = 255;
    }
    PointXYZRGB colored_pt(red, green, blue);
    colored_pt.x = pt->x;
    colored_pt.y = pt->y;
    colored_pt.z = pt->z;
    colored_cloud->push_back(colored_pt);
  }
  return addColorPointCloud(colored_cloud);
}


std::string Visualizer3D::getId(const string &what) {
  std::stringstream ss;
  ss << what << "_" << identifier++;
  all_identifiers.push_back(ss.str());
  return ss.str();
}

double Visualizer3D::rngF() {
  if(color_stack.size() > color_index) {
    return color_stack[color_index++] / 256.0;
  } else {
    return rng.uniform(0.0, 1.0);
  }
}

unsigned Visualizer3D::rngU() {
  if(color_stack.size() > color_index) {
    return color_stack[color_index++];
  } else {
    return rng(256);
  }
}

} /* namespace but_velodyne */
