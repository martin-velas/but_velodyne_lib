/*
 * Overlap.h
 *
 *  Created on: Feb 18, 2019
 *      Author: ivelas
 */

#ifndef OVERLAP_H_
#define OVERLAP_H_

#include <pcl/common/common.h>
#include <velodyne_pointcloud/point_types.h>
#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/CollarLinesRegistration.h>

namespace but_velodyne {

class SphericalZbuffer {
public:
  typedef boost::shared_ptr<SphericalZbuffer> Ptr;

  SphericalZbuffer(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud_,
      const int azimuthal_bins_, const int polar_bins_, const float depth_quantile,
      const bool use_pts_counters = false);

  float getDepth(const float azimuth, const float polar_angle) const;

  size_t containsPoints(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance);

  size_t containsPoints(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud,
      const float depth_relative_tolerance, const float depth_absolute_tolerance,
      pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &overlap,
      pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &rest);

  bool containsPoint(const velodyne_pointcloud::VelodynePoint &point,
    const float depth_relative_tolerance, const float depth_absolute_tolerance);

  void addToVisualizer(but_velodyne::Visualizer3D &visualizer,
      const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &src_cloud,
      const float depth_relative_tolerance = 0.0, const float depth_absolute_tolerance = 0.0) const;

  float visitedPortion(void) const;

protected:
  void setDepth(const float azimuth, const float polar_angle, const float depth);

  void visit(const float azimuth, const float polar_angle);

  int getIndex(const float azimuth, const float polar_angle) const;

private:
  const int azimuthal_bins, polar_bins;
  const float azimuthal_resolution, polar_resolution;
  std::vector<float> depths;
  std::vector<bool> visited;
  int cells_occupied;
  std::vector<int> pts_counters;
};

int get_frames_distance(const int i, const int j, const int frames_count, const bool circular);

float harmonic_avg(const float a, const float b);

class DenseCloudOverlap {

    typedef pcl::PointXYZ PointT;

public:

    class Parameters {
    public:
        Parameters(
                bool visualization_ = false,
                float leaf_size_ = 0.05,
                float max_match_distance_ = 0.1,
                float outlier_stdev_thresh_ = 10.0) :
                visualization(visualization_),
                leaf_size(leaf_size_),
                max_match_distance(max_match_distance_) {
        }
        bool visualization;
        float leaf_size;
        float max_match_distance;

        void loadFrom(boost::program_options::options_description &desc);

    } param;

    DenseCloudOverlap(Parameters parameters_) :
            param(parameters_) {
    }

    void compute(pcl::PointCloud<PointT>::ConstPtr src_cloud,
                 pcl::PointCloud<PointT>::ConstPtr trg_cloud,
                 float &overlapAbsolute, float &overlapRelative) const;

protected:

    size_t computeTrgOverlap(pcl::PointCloud<PointT>::ConstPtr src_cloud,
                             pcl::PointCloud<PointT>::ConstPtr trg_cloud) const;
};

class ClsOverlapEstimator {

public:

    class Parameters {
    public:
        Parameters (
                const float max_t_ = 0.2,
                const float max_R_deg_ = 2.0) :
                max_t(max_t_),
                max_R_deg(max_R_deg_) {
        }

        void prepareForLoading(boost::program_options::options_description &options_desc);

        float max_t;
        float max_R_deg;
    } param;

    ClsOverlapEstimator(const LineCloud::Ptr src_lines_, const Parameters param_) :
            src_lines(src_lines_), param(param_) {
      max_R_tan = tan(degToRad(param.max_R_deg));
      src_kdtree.setInputCloud(src_lines_->getMiddles());
    }

    float overlapWith(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta,
                      LineCloud &within, LineCloud &rest);

    float overlapWith(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta);

protected:

    vector<cv::DMatch> getMatches(const LineCloud &trg_lines, const Eigen::Affine3f &T_delta);

    bool isMatchInlier(const cv::DMatch &m, const LineCloud &trg_lines) const;

private:

    const LineCloud::Ptr src_lines;
    float max_R_tan;
    pcl::KdTreeFLANN<pcl::PointXYZ> src_kdtree;

};

} /* namespace but_velodyne */

#endif /* OVERLAP_H_ */
