/*
 * Overlap.cpp
 *
 *  Created on: Feb 18, 2019
 *      Author: ivelas
 */

#include <but_velodyne/Overlap.h>


using namespace velodyne_pointcloud;
using namespace pcl;


namespace but_velodyne {

const float DEG_TO_RAD = M_PI / 180.0;
const float RAD_TO_DEG = 180.0 / M_PI;

const float EPS = 0.0001;
const float AZIMUTHAL_RANGE = 360.0;
const float POLAR_RANGE = 180.0;

float keepInRange(const float value, const float min, const float max, const float eps) {
  return MIN( MAX(value, min+eps), max-eps );
}

void pointToSpherical(const VelodynePoint &pt, float &azimuth, float &polar, float &range) {
  azimuth = std::atan2(pt.z, pt.x) * RAD_TO_DEG;
  azimuth = keepInRange(azimuth, -AZIMUTHAL_RANGE/2, AZIMUTHAL_RANGE/2, EPS);
  float horizontal_range = sqrt(pt.x * pt.x + pt.z * pt.z);
  polar = atan(-pt.y / horizontal_range) * RAD_TO_DEG;
  polar = keepInRange(polar, -POLAR_RANGE/2, POLAR_RANGE/2, EPS);
  range = pt.getVector3fMap().norm();
}

void sphericalToPoint(const float azimuth, const float polar, const float range, VelodynePoint &pt) {
  pt.y = - sin(polar*DEG_TO_RAD)*range;
  float horizontal_range = cos(polar*DEG_TO_RAD)*range;
  pt.z = sin(azimuth*DEG_TO_RAD)*horizontal_range;
  pt.x = cos(azimuth*DEG_TO_RAD)*horizontal_range;
}

SphericalZbuffer::SphericalZbuffer(const PointCloud<VelodynePoint> &cloud_,
    const int azimuthal_bins_, const int polar_bins_, const float depth_quantile) :
    azimuthal_bins(azimuthal_bins_), polar_bins(polar_bins_),
    azimuthal_resolution(AZIMUTHAL_RANGE / azimuthal_bins_), polar_resolution(POLAR_RANGE / polar_bins_),
    depths(azimuthal_bins_*polar_bins_), visited(azimuthal_bins_*polar_bins_, false) {
  vector< vector<float> > depth_sets(azimuthal_bins*polar_bins);
  for(PointCloud<VelodynePoint>::const_iterator pt = cloud_.begin(); pt < cloud_.end(); pt++) {
    float azimuth, polar, range;
    pointToSpherical(*pt, azimuth, polar, range);
    int idx = getIndex(azimuth, polar);
    if(idx >= depth_sets.size()) {
      cerr << azimuth << " " << polar << endl;
      cerr << idx << " " << depth_sets.size() << endl;
    }
    depth_sets[idx].push_back(range);
  }

  cells_occupied = 0;
  for(int i = 0; i < depths.size(); i++) {
    if(depth_sets[i].size() < 5) {
      depths[i] = -1;
    } else {
      sort(depth_sets[i].begin(), depth_sets[i].end());
      depths[i] = depth_sets[i][depth_sets[i].size()*depth_quantile];
      cells_occupied++;
    }
  }
}

float SphericalZbuffer::getDepth(const float azimuth, const float polar_angle) const {
  return depths[getIndex(azimuth, polar_angle)];
}

size_t SphericalZbuffer::containsPoints(const PointCloud<VelodynePoint> &cloud,
    const float depth_relative_tolerance, const float depth_absolute_tolerance) {
  size_t result = 0;
  for(PointCloud<VelodynePoint>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    if(this->containsPoint(*pt, depth_relative_tolerance, depth_absolute_tolerance)) {
      result++;
    }
  }
  return result;
}

size_t SphericalZbuffer::containsPoints(const PointCloud<VelodynePoint> &cloud,
    const float depth_relative_tolerance, const float depth_absolute_tolerance,
    PointCloud<VelodynePoint> &overlap, PointCloud<VelodynePoint> &rest) {
  for(PointCloud<VelodynePoint>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    if(this->containsPoint(*pt, depth_relative_tolerance, depth_absolute_tolerance)) {
      overlap.push_back(*pt);
    } else {
      rest.push_back(*pt);
    }
  }
  return overlap.size();
}

bool SphericalZbuffer::containsPoint(const VelodynePoint &point,
  const float depth_relative_tolerance, const float depth_absolute_tolerance) {
  float azimuth, polar_angle, range;
  pointToSpherical(point, azimuth, polar_angle, range);
  this->visit(azimuth, polar_angle);
  const float zdepth = this->getDepth(azimuth, polar_angle);
  return (zdepth*(1.0 + depth_relative_tolerance) + depth_relative_tolerance) > range;
}

void SphericalZbuffer::addToVisualizer(Visualizer3D &visualizer, const PointCloud<VelodynePoint> &src_cloud,
    const float depth_relative_tolerance, const float depth_absolute_tolerance) const {
  PointCloud<VelodynePoint> vis_cloud;
  vis_cloud.resize(src_cloud.size());
  for(int i = 0; i < src_cloud.size(); i++) {
    float azimuth, polar_angle, range;
    pointToSpherical(src_cloud[i], azimuth, polar_angle, range);
    sphericalToPoint(azimuth, polar_angle,
        this->getDepth(azimuth, polar_angle)*(1.0 + depth_relative_tolerance) + depth_absolute_tolerance,
        vis_cloud[i]);
  }
  visualizer.addPointCloud(vis_cloud);
}

float SphericalZbuffer::visitedPortion(void) const {
  int visited_cnt = 0;
  for(int i = 0; i < visited.size(); i++) {
    if(visited[i] && depths[i] > 0) {
      visited_cnt++;
    }
  }
  return (float(visited_cnt)) / cells_occupied;
}

void SphericalZbuffer::setDepth(const float azimuth, const float polar_angle, const float depth) {
  depths[getIndex(azimuth, polar_angle)] = depth;
}

void SphericalZbuffer::visit(const float azimuth, const float polar_angle) {
  visited[getIndex(azimuth, polar_angle)] = true;
}

int SphericalZbuffer::getIndex(const float azimuth, const float polar_angle) const {
  int aidx = floor((azimuth+AZIMUTHAL_RANGE/2.0) / azimuthal_resolution);
  int pidx = floor((polar_angle+POLAR_RANGE/2.0) / polar_resolution);

  int result = MIN(aidx, azimuthal_bins-1) * polar_bins + MIN(pidx, polar_bins-1);

  if(result >= depths.size() || result < 0) {
    cerr << "WARNING - SPHERICAL BINS OVERFLOW!!!" << endl;
    cerr << "azimuthal_bins: " << azimuthal_bins << endl;
    cerr << "polar_bins: " << polar_bins << endl;
    cerr << "azimuthal_resolution: " << azimuthal_resolution << endl;
    cerr << "polar_resolution: " << polar_resolution << endl;

    cerr << "azimuth: " << azimuth << endl;
    cerr << "polar_angle: " << polar_angle << endl;

    cerr << "aidx: " << aidx << endl;
    cerr << "pidx: " << pidx << endl;
    cerr << "result: " << result << endl;
    cerr << "--------------------" << endl;
  }

  return int(aidx) * polar_bins + int(pidx);
}

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular) {
  if(!circular) {
    return abs(i-j);
  } else {
    return MIN(abs(i-j), frames_count-abs(i-j));
  }
}

float harmonic_avg(const float a, const float b) {
  if(-0.0001 < a+b && a+b < 0.0001) {
    return 0;
  } else {
    return 2.0*a*b/(a+b);
  }
}

} /* namespace but_velodyne */
