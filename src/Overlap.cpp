/*
 * Overlap.cpp
 *
 *  Created on: Feb 18, 2019
 *      Author: ivelas
 */

#include <but_velodyne/Overlap.h>


using namespace velodyne_pointcloud;
using namespace pcl;
namespace po = boost::program_options;


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
    const int azimuthal_bins_, const int polar_bins_, const float depth_quantile, const bool use_pts_counters) :
    azimuthal_bins(azimuthal_bins_), polar_bins(polar_bins_),
    azimuthal_resolution(AZIMUTHAL_RANGE / azimuthal_bins_), polar_resolution(POLAR_RANGE / polar_bins_),
    depths(azimuthal_bins_*polar_bins_), visited(azimuthal_bins_*polar_bins_, false),
    pts_counters(azimuthal_bins_*polar_bins_, use_pts_counters ? 0 : 1000000) {
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
    pts_counters[idx] += 10;
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
  const int idx = getIndex(azimuth, polar_angle);
  return pts_counters[idx] > 0 && (zdepth*(1.0 + depth_relative_tolerance) + depth_relative_tolerance) > range;
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
  int idx = getIndex(azimuth, polar_angle);
  visited[idx] = true;
  pts_counters[idx]--;
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

void DenseCloudOverlap::compute(pcl::PointCloud<PointT>::ConstPtr src_cloud,
             pcl::PointCloud<PointT>::ConstPtr trg_cloud,
             float &overlapAbsolute, float &overlapRelative) const {
  PointCloud<PointT>::Ptr src_sampled(new PointCloud<PointT>);
  PointCloud<PointT>::Ptr trg_sampled(new PointCloud<PointT>);
  if(param.leaf_size > 0.0) {
    subsample_by_voxel_grid(src_cloud, *src_sampled, param.leaf_size, param.outlier_stdev_thresh);
    subsample_by_voxel_grid(trg_cloud, *trg_sampled, param.leaf_size, param.outlier_stdev_thresh);
  } else {
    *src_sampled += *src_cloud;
    *trg_sampled += *trg_cloud;
  }

  size_t overlap_trg_points = this->computeTrgOverlap(src_sampled, trg_sampled);
  size_t overlap_src_points = this->computeTrgOverlap(trg_sampled, src_sampled);

  overlapAbsolute = float(overlap_trg_points + overlap_src_points) / 2.0f;
  overlapRelative = harmonic_avg(overlap_trg_points / float(trg_sampled->size()),
                                 overlap_src_points / float(src_sampled->size()));
}

size_t DenseCloudOverlap::computeTrgOverlap(pcl::PointCloud<PointT>::ConstPtr src_cloud,
                                            pcl::PointCloud<PointT>::ConstPtr trg_cloud) const {
  KdTreeFLANN<PointT> tree;
  tree.setInputCloud(src_cloud);

  PointCloud<PointT> trg_overlap, trg_rest;
  for(PointCloud<PointT>::const_iterator pt = trg_cloud->begin(); pt < trg_cloud->end(); pt++) {
    vector<int> indices;
    vector<float> distances;
    tree.radiusSearch(*pt, param.max_match_distance, indices, distances);
    if(indices.empty()) {
      trg_rest.push_back(*pt);
    } else {
      trg_overlap.push_back(*pt);
    }
  }

  if(param.visualization) {
    float overlap = trg_overlap.size() / float(trg_cloud->size());
    cerr << "Overlap: " << overlap << " (src points: " << src_cloud->size() << ", trg_points: " << trg_cloud->size()
         << ", trg overlapping points: " << trg_overlap.size() << ", trg rest: " << trg_rest.size() << ")" << endl;
    io::savePCDFileBinary("src_cloud.pcd", *src_cloud);
    io::savePCDFileBinary("trg_overlap.pcd", trg_overlap);
    io::savePCDFileBinary("trg_rest.pcd", trg_rest);
    Visualizer3D::getCommonVisualizer()->keepOnlyClouds(0)
            .setColor(0, 0, 255).addPointCloud(*src_cloud)
            .setColor(0, 255, 0).addPointCloud(trg_overlap)
            .setColor(255, 0, 0).addPointCloud(trg_rest)
            .show();
  }

  return trg_overlap.size();
}

void DenseCloudOverlap::Parameters::loadFrom(po::options_description &desc) {
  desc.add_options()
    ("visualization,v", po::bool_switch(&visualization),
     "Show visualization.")
    ("leaf_size,l", po::value<float>(&leaf_size)->default_value(leaf_size),
     "Kd-tree leaf size for downsampling. If (size <= 0) then no resampling.")
    ("max_match_distance,m", po::value<float>(&max_match_distance)->default_value(max_match_distance),
     "Correspondence distance threshold.")
    ("outlier_stdev_thresh", po::value<float>(&outlier_stdev_thresh)->default_value(outlier_stdev_thresh),
     "Standard deviation of the distance for statistical outlier removal.")
  ;
}

void ClsOverlapEstimator::Parameters::prepareForLoading(po::options_description &options_desc) {
  options_desc.add_options()
    ("max_t", po::value<float>(&max_t)->default_value(0.2), "Max translation [m] allowed.")
    ("max_R", po::value<float>(&max_R_deg)->default_value(2.0), "Max rotation [deg] allowed.")
  ;
}

float ClsOverlapEstimator::overlapWith(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta,
                  LineCloud &within, LineCloud &rest) {
  const vector<cv::DMatch> matches = getMatches(trg_lines, T_delta);
  for(vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    const LineCloud::PointCloudLineWithMiddleAndOrigin &trg_l = trg_lines[m->queryIdx];
    if(isMatchInlier(*m, trg_lines)) {
      within.push_back(trg_l);
    } else {
      rest.push_back(trg_l);
    }
  }
  return float(within.size()) / trg_lines.size();
}

float ClsOverlapEstimator::overlapWith(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta) {
  const vector<cv::DMatch> matches = getMatches(trg_lines, T_delta);
  int inliers = 0;
  for(vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    if(isMatchInlier(*m, trg_lines)) {
      inliers++;
    }
  }
  return float(inliers) / trg_lines.size();
}

vector<cv::DMatch> ClsOverlapEstimator::getMatches(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta) {
  CollarLinesRegistration::Parameters cls_params;
  cls_params.distance_threshold = CollarLinesRegistration::NO_THRESHOLD;
  CollarLinesRegistration cls_fitting(*src_lines, src_kdtree, trg_lines,
                                      cls_params, T_delta.matrix());
  cls_fitting.computeError();
  return cls_fitting.getMatches();
}

bool ClsOverlapEstimator::isMatchInlier(const cv::DMatch &m, const LineCloud &trg_lines) const {
  const float &src_range = (*src_lines)[m.trainIdx].range;
  const float &trg_range = trg_lines[m.queryIdx].range;
  // the distances are squared in DMatch
  return sqrt(m.distance) < MAX(src_range, trg_range) * max_R_tan + param.max_t;
}

} /* namespace but_velodyne */
