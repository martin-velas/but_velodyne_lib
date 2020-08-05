/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#include <but_velodyne/VelodynePointCloud.h>

#include <vector>
#include <cstdlib>
#include <cassert>
#include <numeric>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <velodyne_pointcloud/point_types.h>

using namespace pcl;
using namespace velodyne_pointcloud;
using namespace std;
using namespace cv;

namespace but_velodyne {

template<>
void subsample_cloud<VelodynePoint>(typename pcl::PointCloud<VelodynePoint>::Ptr cloud, float sampling_ratio) {
  int subsampled_size = cloud->size() * sampling_ratio;
  vector<bool> true_mask(subsampled_size, true);
  vector<bool> mask(cloud->size()-subsampled_size, false);
  mask.insert(mask.end(), true_mask.begin(), true_mask.end());
  random_shuffle(mask.begin(), mask.end());
  mask_cloud(*cloud, mask);
}

template<>
void regular_subsampling<VelodynePoint>(typename pcl::PointCloud<VelodynePoint>::ConstPtr full_cloud, const float sampling_rate,
    pcl::PointIndices::Ptr indices, typename pcl::PointCloud<VelodynePoint>::Ptr subsampled_cloud);

template<>
void regular_subsampling<PointXYZI>(typename pcl::PointCloud<PointXYZI>::ConstPtr full_cloud, const float sampling_rate,
    pcl::PointIndices::Ptr indices, typename pcl::PointCloud<PointXYZI>::Ptr subsampled_cloud);

VelodynePoint operator +(const VelodynePoint &p1, const VelodynePoint &p2) {
  VelodynePoint res;
  res.x = p1.x + p2.x;
  res.y = p1.y + p2.y;
  res.z = p1.z + p2.z;
  res.intensity = p1.intensity + p2.intensity;
  return res;
}

VelodynePoint operator *(const VelodynePoint &p1, float s) {
  VelodynePoint res;
  res.x = p1.x * s;
  res.y = p1.y * s;
  res.z = p1.z * s;
  res.intensity = p1.intensity * s;
  return res;
}

VelodynePoint operator *(float s, const VelodynePoint &p1) {
  return p1*s;
}

VelodynePoint operator -(const VelodynePoint &p1, const VelodynePoint &p2) {
  return p1 + (p2 * (-1));
}

VelodynePoint operator /(const VelodynePoint &p1, float s) {
  return p1 * (1/s);
}

void VelodynePointCloud::normalizeIntensity(float min_intensity, float max_intensity)
{
  float min = 0.0;
  float max = 1.0;
  for (PointCloud<VelodynePoint>::iterator pt = this->begin(); pt < this->end(); pt++)
  {
    pt->intensity = (pt->intensity - min_intensity) /
                    (max_intensity - min_intensity) * (max - min) + min;
  }
}

VelodyneSpecification::Model VelodynePointCloud::estimateModel() {
  int max_ring = -1;
  for(VelodynePointCloud::const_iterator p = this->begin(); p < this->end(); p++) {
    max_ring = MAX(max_ring, p->ring);
  }
  if(max_ring < 0) {
    velodyne_model = VelodyneSpecification::Unknown;
  } else if(max_ring < VelodyneSpecification::rings(VelodyneSpecification::VLP16)) {
    velodyne_model = VelodyneSpecification::VLP16;
  } else if(max_ring < VelodyneSpecification::rings(VelodyneSpecification::HDL32)) {
    velodyne_model = VelodyneSpecification::HDL32;
  } else if(max_ring < VelodyneSpecification::rings(VelodyneSpecification::HDL64)) {
    velodyne_model = VelodyneSpecification::HDL64;
  } else {
    velodyne_model = VelodyneSpecification::Unknown;
  }
  return velodyne_model;
}

void VelodynePointCloud::copyTo(VelodynePointCloud &dest) const {
  dest += *this;
  dest.velodyne_model = this->velodyne_model;
  dest.axis_correction = this->axis_correction;
}

VelodynePointCloud VelodynePointCloud::discartWeakPoints(float threshold) {
  VelodynePointCloud output;
  for(PointCloud<VelodynePoint>::const_iterator pt = this->begin();
      pt < this->end();
      pt++) {
    if(pt->intensity > threshold) {
      output.push_back(*pt);
    }
  }
  return output;
}

VelodynePointCloud VelodynePointCloud::resampleTo(int final_number) {
  VelodynePointCloud resampled;
  unsigned counter = 0;
  unsigned factor = this->size()/final_number + 1;
  for(VelodynePointCloud::iterator pt = this->begin(); pt < this->end(); pt++, counter++) {
    if(counter%factor == 0) {
      resampled.push_back(*pt);
    }
  }
  cerr << resampled.size() << endl;
  assert(resampled.size() == final_number);
  return resampled;
}

VelodynePointCloud VelodynePointCloud::resampleByRatio(float preserve_ratio) {
  return resampleTo(this->size()*preserve_ratio);
}

VelodynePointCloud VelodynePointCloud::computeEdges(float threshold) const
{
  vector< vector<VelodynePoint> > rings;
  vector< vector<int> > indices;
  vector<int> indices_to_rings;
  getRings(rings, indices, indices_to_rings);
  VelodynePointCloud edge_cloud;

  float max_difference = 0;
  float min_difference = INFINITY;
  for (vector<vector<VelodynePoint> >::iterator ring = rings.begin(); ring < rings.end(); ring++)
  {
    if(ring->size() < 2) {
      continue;
    }
    float previous_range, current_range, next_range;
    current_range = computeRange(ring->front());
    next_range = computeRange(*(ring->begin() + 1));
    for (vector<VelodynePoint>::iterator pt = ring->begin() + 1; pt + 1 < ring->end(); pt++)
    {
      previous_range = current_range;
      current_range = next_range;
      next_range = computeRange(*(pt + 1));
      VelodynePoint edge_pt;
      edge_pt.x = pt->x;
      edge_pt.y = pt->y;
      edge_pt.z = pt->z;
      edge_pt.ring = pt->ring;
      edge_pt.intensity = MAX(MAX( previous_range-current_range, next_range-current_range), 0) * 10;
      min_difference = MIN(edge_pt.intensity, min_difference);
      max_difference = MAX(edge_pt.intensity, max_difference);
      edge_cloud.push_back(edge_pt);
    }
  }
  edge_cloud.normalizeIntensity(min_difference, max_difference);
  edge_cloud = edge_cloud.discartWeakPoints(threshold);
  return edge_cloud;
}

void VelodynePointCloud::getRings(vector< vector<VelodynePoint> > &rings,
				  vector< vector<int> > &to_cloud_indices,
				  vector<int> &to_ring_indices) const
{
  rings.clear();
  rings.resize(ringCount());
  to_cloud_indices.clear();
  to_cloud_indices.resize(ringCount());
  int id = 0;
  for (PointCloud<VelodynePoint>::const_iterator pt = this->begin();
      pt < this->end();
      pt++, id++)
  {
    assert(pt->ring < ringCount());
    rings[pt->ring].push_back(*pt);
    to_cloud_indices[pt->ring].push_back(id);
    to_ring_indices.push_back((rings[pt->ring].size()-1)*ringCount() +
			      pt->ring);
  }
}

VelodynePoint VelodynePointCloud::getMinValuePt() const {
  VelodynePoint min;
  min.x = min.y = min.z = min.ring = 0;
  min.intensity = INFINITY;
  for(VelodynePointCloud::const_iterator pt = this->begin(); pt < this->end(); pt++) {
    if(pt->intensity < min.intensity) {
      min = *pt;
    }
  }
  return min;
}

VelodynePoint VelodynePointCloud::getMaxValuePt() const {
  VelodynePoint max;
  max.x = max.y = max.z = max.ring = 0;
  max.intensity = -INFINITY;
  for(VelodynePointCloud::const_iterator pt = this->begin(); pt < this->end(); pt++) {
    if(pt->intensity > max.intensity) {
      max = *pt;
    }
  }
  return max;
}

PointCloud<PointXYZ>::Ptr VelodynePointCloud::getXYZCloudPtr() const {
  PointCloud<PointXYZ>::Ptr cloud_ptr(new PointCloud<PointXYZ>());
  for(VelodynePointCloud::const_iterator pt = begin(); pt < end(); pt++) {
    cloud_ptr->push_back(VelodynePointToPointXYZ(*pt));
  }
  return cloud_ptr;
}

float VelodynePointCloud::getMedianRange() const {
  if(this->size() == 0) {
    return NAN;
  }
  vector<float> ranges;
  for(VelodynePointCloud::const_iterator pt = begin(); pt < end(); pt++) {
    ranges.push_back(computeRange(*pt));
  }
  sort(ranges.begin(), ranges.end());
  return ranges[ranges.size()/2];
}

void VelodynePointCloud::setImageLikeAxisFromKitti() {
  axis_correction <<
        0, -1,  0,  0,
        0,  0, -1,  0,
        1,  0,  0,  0,
        0,  0,  0,  1;
  transformPointCloud(*this, *this, axis_correction);
}

void VelodynePointCloud::setImageLikeAxisFromBut() {
	axis_correction = getTransformation(0, 0, 0, M_PI / 2, 0, 0).matrix();
  transformPointCloud(*this, *this, axis_correction);
}

void VelodynePointCloud::setImageLikeAxisFromDarpa() {
	axis_correction = pcl::getTransformation(0, 0, 0, M_PI/2, M_PI, 0).matrix();
  transformPointCloud(*this, *this, axis_correction);
}

void VelodynePointCloud::addAxisCorrection(const Eigen::Matrix4f &correction) {
	axis_correction = axis_correction * correction;
  transformPointCloud(*this, *this, correction);
}

void VelodynePointCloud::setRingsByPointCount() {
	int ring = 0;
	int ring_size = this->size() / ringCount();
	for (int i = 0; i < this->size(); i++) {
		if (i != 0 && (i % ring_size) == 0) {
			ring++;
		}

		if (ring < ringCount()) {
			points[i].ring = ring;
		} else {
			this->erase(this->begin() + i, this->end());
		}
	}
}

void VelodynePointCloud::setRingsByHorizontalAngles() {
  const float ANGLE_DIFF_THRESH = 60;
  const int WIN_HALF_SIZE = 5;
  int ring = 0;

  vector<float> angles;
  for(int i = 0; i < this->size(); i++) {
    float angle = horizontalAngle(points[i].z, points[i].x);
    angles.push_back(angle);
  }

  /*
   * Convolution with step function:
   *  1.0  -------|
   *              |
   *              |
   * -1.0         |-------
   *
   * ... and preserving only the indices over threshold
   */
  vector<float> possible_breakpoints;
  for(int i = 0; i < angles.size(); i++) {
  	if(WIN_HALF_SIZE < i && i < this->size()-WIN_HALF_SIZE) {
  		float angle_diff = std::accumulate(angles.begin()+i-WIN_HALF_SIZE, angles.begin()+i, 0) -
  										   std::accumulate(angles.begin()+i+1, angles.begin()+i+WIN_HALF_SIZE+1, 0);
  		if(angle_diff > ANGLE_DIFF_THRESH) {
  			possible_breakpoints.push_back(i);
  		}
  	}
  }
  cv::Mat possible_breakpoints_mat(possible_breakpoints);

  const int K = ringCount()-1;
  Mat labels, centers;
  cv::kmeans(possible_breakpoints_mat,
  		K, labels,
      cv::TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 50, 1.0),
         3, KMEANS_PP_CENTERS, centers);
  cv::sort(centers, centers, CV_SORT_EVERY_COLUMN);

  cv::Mat breakpoints = Mat::ones(ringCount(), 1, CV_32FC1)*INFINITY;
  centers.copyTo(breakpoints.rowRange(0, ringCount()-1));
  int cloud_index = 0;
  for(int ring = 0; ring < ringCount(); ring++) {
  	for(; cloud_index < breakpoints.at<float>(ring) && cloud_index < this->size(); cloud_index++) {
  		points[cloud_index].ring = ring;
  	}
  }
}

//================// VelodynePoint //================//

PointXYZ VelodynePointToPointXYZ(const VelodynePoint &in)
{
  PointXYZ out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

/**
 *  Returns false (invalid result) if the point is behind the camera or
 *  it would be projected outside the projection plane.
 */
bool projectPoint(const VelodynePoint &pt,
                  const cv::Mat &projection_matrix,
                  const Rect &plane,
                  Point2f &projected_pt)
{
  if (pt.z < 0)
  {
    return false;
  }

  cv::Mat pt_3D(4, 1, CV_32FC1);

  pt_3D.at<float>(0) = pt.x;
  pt_3D.at<float>(1) = pt.y;
  pt_3D.at<float>(2) = pt.z;
  pt_3D.at<float>(3) = 1.0f; // is homogenious coords. the point's 4. coord is 1

  cv::Mat pt_2D = projection_matrix * pt_3D;

  float w = pt_2D.at<float>(2);
  projected_pt.x = pt_2D.at<float>(0) / w;
  projected_pt.y = pt_2D.at<float>(1) / w;

  return projected_pt.inside(plane);
}

vector<float> VelodynePointCloud::getMaxOfRingRanges() const {
  vector< vector<float> > ranges(ringCount());
  for(const_iterator pt = begin(); pt < end(); pt++) {
    ranges[pt->ring].push_back(pt->x*pt->x + pt->z*pt->z);
  }
  vector<float> almost_maximums;
  for(int ring = 0; ring < ranges.size(); ring++) {
    if(ranges[ring].empty()) {
      almost_maximums.push_back(NAN);
    } else {
      sort(ranges[ring].begin(), ranges[ring].end());
      almost_maximums.push_back(sqrt(ranges[ring][ranges[ring].size()*0.9]));
    }
  }
  return almost_maximums;
}

float VelodynePointCloud::averageIntensity() const {
  float avg = 0.0;
  int count = 0;
  for(const_iterator p = begin(); p < end(); p++) {
    float intensity = p->intensity;
    if(!isnan(intensity) && !isinf(intensity)) {
      avg += intensity;
      count++;
    }
  }
  return (count > 0) ? (avg/count) : 0;
}

void VelodynePointCloud::removeNanPoints() {
  for(iterator pt = this->begin(); pt < this->end();) {
    if(isFinite(*pt)) {
      pt++;
    } else {
      pt = this->erase(pt);
    }
  }
}

Eigen::Affine3f SensorsCalibration::getSensorPose(const Eigen::Affine3f system_pose, const int sensor_id) const {
  return system_pose * sensors_poses[sensor_id];
}

}
