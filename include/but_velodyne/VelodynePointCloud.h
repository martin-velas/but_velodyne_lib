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

#ifndef VELODYNEPOINTCLOUD_H_
#define VELODYNEPOINTCLOUD_H_

#include <fstream>

#include <but_velodyne/VelodyneSpecification.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/common.h>
#include <but_velodyne/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/random_sample.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

#include <velodyne_pointcloud/point_types.h>

#include <cv.h>

namespace but_velodyne {

/**!
 * @param pt Velodyne 3D point
 * @returns the distance of the point from sensor
 */
template <typename PointT>
inline float computeRange(const PointT &pt)
{
  return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

inline float compute2DRange(const velodyne_pointcloud::VelodynePoint &pt)
{
  return sqrt(pt.x * pt.x + pt.z * pt.z);
}

template <typename T1, typename T2>
void copyXYZ(const T1 &p1, T2 &p2) {
  p2.x = p1.x;
  p2.y = p1.y;
  p2.z = p1.z;
}

/**!
 * Conversion from PCL point to Eigen::Vector3f
 *
 * @param pt Velodyne 3D point
 * @return Eigen representation of 3D points
 */
template<typename PclPointT>
inline Eigen::Vector3f pclPointToVector3f(const PclPointT &pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

/**!
 * Conversion from PCL PointXYZ to VelodynePoint
 *
 * @param pt 3D point
 * @return PCL representation of Velodyne 3D point
 */
inline velodyne_pointcloud::VelodynePoint PointXYZ2VelodynePoint(const pcl::PointXYZ &pt)
{
  velodyne_pointcloud::VelodynePoint pt_ir;
  pt_ir.x = pt.x;
  pt_ir.y = pt.y;
  pt_ir.z = pt.z;
  pt_ir.intensity = 0;
  pt_ir.ring = 0;
  return pt_ir;
}
/**!
 * Conversion from PCL VelodynePoint to PointXYZ
 *
 * @param pt Velodyne 3D point
 * @return PCL 3D point
 */
pcl::PointXYZ VelodynePointToPointXYZ(const velodyne_pointcloud::VelodynePoint &in);

/**!
 * Projects the Velodyne 3D point to the image plane
 *
 * @param pt Velodyne 3D point
 * @param projection_matrix 3x4 projection matrix
 * @param plane image plane (size of the image is required)
 * @param projected_pt [output] projected 2D point
 * @return true iff the 3D point is projected within the image dimensions
 */
bool projectPoint(const velodyne_pointcloud::VelodynePoint &pt,
                  const cv::Mat &projection_matrix,
                  const cv::Rect &plane,
                  cv::Point2f &projected_pt);

template <class PointType>
void subsample_cloud(typename pcl::PointCloud<PointType>::Ptr cloud, float sampling_ratio) {
  pcl::RandomSample<PointType> subsampling;
  subsampling.setInputCloud(cloud);
  subsampling.setSample(cloud->size()*sampling_ratio);
  subsampling.filter(*cloud);
}

template <class PointType1, class PointType2>
void subsample_clouds(typename pcl::PointCloud<PointType1>::Ptr cloud1,
    typename pcl::PointCloud<PointType2>::Ptr cloud2, float sampling_ratio) {
  pcl::RandomSample<PointType1> subsampling;
  subsampling.setInputCloud(cloud1);
  subsampling.setSample(cloud1->size()*sampling_ratio);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  subsampling.filter(indices->indices);

  pcl::ExtractIndices<PointType1> extract1;
  extract1.setInputCloud(cloud1);
  extract1.setIndices(indices);
  extract1.filter(*cloud1);

  pcl::ExtractIndices<PointType2> extract2;
  extract2.setInputCloud(cloud2);
  extract2.setIndices(indices);
  extract2.filter(*cloud2);
}

template <class PointType>
void regular_subsampling(typename pcl::PointCloud<PointType>::ConstPtr full_cloud, const float sampling_rate,
    pcl::PointIndices::Ptr indices, typename pcl::PointCloud<PointType>::Ptr subsampled_cloud) {
  const float LEAF_SIZE = 0.2;
  pcl::VoxelGrid<PointType> grid;
  grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  typename pcl::PointCloud<PointType>::Ptr cumulated_grid_clouds(new pcl::PointCloud<PointType>);
  srand(time(NULL));
  while(cumulated_grid_clouds->size() < full_cloud->size()*sampling_rate) {
    Eigen::Affine3f t = pcl::getTransformation(get_rand(LEAF_SIZE), get_rand(LEAF_SIZE), get_rand(LEAF_SIZE), 0, 0, 0);
    typename pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
    transformPointCloud(*full_cloud, *tmp_cloud, t);
    grid.setInputCloud(tmp_cloud);
    grid.filter(*tmp_cloud);
    pcl::transformPointCloud(*tmp_cloud, *tmp_cloud, t.inverse());
    *cumulated_grid_clouds += *tmp_cloud;
  }

  pcl::KdTreeFLANN<PointType> index;
  index.setInputCloud(full_cloud);
  for(typename pcl::PointCloud<PointType>::const_iterator p = cumulated_grid_clouds->begin(); p < cumulated_grid_clouds->end(); p++) {
    std::vector<int> idxs(1);
    std::vector<float> dist(1);
    index.nearestKSearch(*p, 1, idxs, dist);
    indices->indices.push_back(idxs.front());
    subsampled_cloud->push_back(full_cloud->at(idxs.front()));
  }
}

template <class PointType>
void mask_cloud(pcl::PointCloud<PointType> &cloud, const std::vector<bool> &mask) {
  int subsampled_i = 0;
  for(int i = 0; i < mask.size(); i++) {
    if(mask[i]) {
      cloud.at(subsampled_i) = cloud.at(i);
      subsampled_i++;
    }
  }
  cloud.erase(cloud.begin()+subsampled_i, cloud.end());
}

velodyne_pointcloud::VelodynePoint operator +(const velodyne_pointcloud::VelodynePoint &p1,
                                           const velodyne_pointcloud::VelodynePoint &p2);

velodyne_pointcloud::VelodynePoint operator *(const velodyne_pointcloud::VelodynePoint &p1,
                                           float s);

velodyne_pointcloud::VelodynePoint operator *(float s,
                                           const velodyne_pointcloud::VelodynePoint &p1);

velodyne_pointcloud::VelodynePoint operator -(const velodyne_pointcloud::VelodynePoint &p1,
                                           const velodyne_pointcloud::VelodynePoint &p2);

velodyne_pointcloud::VelodynePoint operator /(const velodyne_pointcloud::VelodynePoint &p1,
                                           float s);

/**!
 * Representation of the Velodyne LiDAR point cloud. The axis are arranged in following manner:
 *
 *    /^ Z-axis
 *   /
 *  /
 * +------> X-axis
 * |
 * |
 * |
 * V Y-axis
 */
class VelodynePointCloud : public pcl::PointCloud<velodyne_pointcloud::VelodynePoint>
{
public:

  typedef boost::shared_ptr<VelodynePointCloud> Ptr;

	VelodynePointCloud() :
		pcl::PointCloud<velodyne_pointcloud::VelodynePoint>(),
		axis_correction(Eigen::Matrix4f::Identity()),
		velodyne_model(VelodyneSpecification::Unknown) {
	}

  /**!
   * Normalization of the point intensities to interval [min_intensity, max_intensity]
   *
   * @param min_intensity lower interval boundary
   * @param max_intensity upper interval boundary
   */
  void normalizeIntensity(float min_intensity, float max_intensity);

  /**!
   * Edge detection in the Velodyne point cloud. Detection searches for the
   * discontinuities along the ring of points.
   *
   * @return point cloud where the intensity of point is proportional to the discontinuity in this point
   */
  VelodynePointCloud computeEdges(float threshold) const;

  /**!
   * @return the point with minimal intensity value
   */
  velodyne_pointcloud::VelodynePoint getMinValuePt() const;

  /**!
   * @return the point with maximal intensity value
   */
  velodyne_pointcloud::VelodynePoint getMaxValuePt() const;

  /**!
   * Regularly resamples the point cloud.
   *
   * @param final_number expected number of preserved points
   * @return resampled point cloud with MIN(this->size(), final_number) points
   */
  VelodynePointCloud resampleTo(int final_number);

  /**!
   * Regularly resamples the point cloud.
   *
   * @param preserve_ratio ratio (espected_size/this->size())
   * @return resampled point cloud with MIN(this->size(), this->size()*preserve_ratio) points
   */
  VelodynePointCloud resampleByRatio(float preserve_ratio);

  /**!
   * @return the shared point to the point cloud data (memory is reallocated)
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZCloudPtr() const;

  /**!
   * @return the median value of all points distances from sensor
   */
  float getMedianRange() const;

  /**!
   * Switches the axis from the KITTI convention
   */
  void setImageLikeAxisFromKitti();

  /**!
   * Switches the axis from the convention used on Toad robot of Robo@FIT group
   */
  void setImageLikeAxisFromBut();

  void setImageLikeAxisFromDarpa();

  void setRingsByPointCount();

  /**
   * Returns 0/360deg in front of LiDAR, 90deg in left, 180deg in back and 270deg in right
   */
	static float horizontalAngle(float to_front, float to_right) {
		const float RAD_TO_DEG = 180.0f / float(CV_PI);
		float angle = std::atan2(to_front, to_right) * RAD_TO_DEG;	// 90..180;-180..0..90

		if (angle < 0) {
			angle += 360;																							// 90..180;180..359,0..90
		}

		angle += 270;
		while (angle >= 360) {
			angle -= 360;																							// 0..90;90..269,270..359
		}

		return angle;
	}

  void setRingsByHorizontalAngles();

  static void fromKittiRaw(const std::string &infile, VelodynePointCloud &out_cloud)
  {
      out_cloud.clear();
      // load point cloud
      std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
      if (!input.good())
      {
        std::cerr << "Could not read file: " << infile << std::endl;
        exit(EXIT_FAILURE);
      }
      input.seekg(0, std::ios::beg);

      int i;
      for (i = 0; input.good() && !input.eof(); i++)
      {
        velodyne_pointcloud::VelodynePoint point;
        input.read((char *)&point.x, 3 * sizeof(float));
        input.read((char *)&point.intensity, sizeof(float));
        out_cloud.push_back(point);
      }
      input.close();
      out_cloud.setVelodyneModel(VelodyneSpecification::HDL64);

      std::cerr << "Read KTTI point cloud " << infile << " with " << i-1 << " points." << std::endl;
  }

  /**!
   * Loads the Velodyne point cloud from the KITTI *.bin file. The ring ID is also
   * estimated.
   *
   * @param infile input file
   * @param out_cloud [output] destination
   */
  static void fromKitti(const std::string &infile, VelodynePointCloud &out_cloud)
  {
  	fromKittiRaw(infile, out_cloud);

    out_cloud.setImageLikeAxisFromKitti();
    out_cloud.setRingsByHorizontalAngles();
    //out_cloud.setRingsByPointCount();
    out_cloud.removeNanPoints();
  }

  static void fromFile(const std::string &infile, VelodynePointCloud &out_cloud, bool transform_pcd = false) {
    std::cerr << "Processing KITTI file: " << infile << std::endl << std::flush;
    if (infile.find(".pcd") != std::string::npos) {
      pcl::io::loadPCDFile(infile, out_cloud);
      if(transform_pcd) {
        out_cloud.setImageLikeAxisFromKitti();
      }
      out_cloud.estimateModel();
    } else {
      VelodynePointCloud::fromKitti(infile, out_cloud);
    }
  }

  static VelodyneSpecification::Model getSourceModel(const std::string &cloud_infile) {
    if (cloud_infile.find(".pcd") != std::string::npos) {
      VelodynePointCloud cloud;
      pcl::io::loadPCDFile(cloud_infile, cloud);
      return cloud.estimateModel();
    } else {
      return VelodyneSpecification::HDL64;
    }
  }

  static int getMaxRingCount(const std::vector<VelodynePointCloud> &point_clouds) {
    int max_ring_count = -1;
    for(int i = 0; i < point_clouds.size(); i++) {
      max_ring_count = MAX(max_ring_count, point_clouds[i].ringCount());
    }
    return max_ring_count;
  }

  static int getMaxRingCount(const std::vector<VelodynePointCloud::Ptr> &point_clouds) {
    int max_ring_count = -1;
    for(int i = 0; i < point_clouds.size(); i++) {
      max_ring_count = MAX(max_ring_count, point_clouds[i]->ringCount());
    }
    return max_ring_count;
  }

  std::vector<float> getMaxOfRingRanges() const;

  float averageIntensity() const;

  void getRings(
      std::vector<std::vector<velodyne_pointcloud::VelodynePoint> > &rings,
      std::vector<std::vector<int> > &to_cloud_indices,
      std::vector<int> &to_ring_indices) const;

  void removeNanPoints();

  Eigen::Matrix4f getAxisCorrection() const {
    return axis_correction;
  }

  void addAxisCorrection(const Eigen::Matrix4f &correction);

  int ringCount() const {
    return VelodyneSpecification::rings(velodyne_model);
  }

  VelodyneSpecification::Model estimateModel();

  VelodyneSpecification::Model getVelodyneModel() const {
    return velodyne_model;
  }

  void setVelodyneModel(VelodyneSpecification::Model velodyneModel) {
    velodyne_model = velodyneModel;
  }

  void copyTo(VelodynePointCloud &dest) const;

protected:
  VelodynePointCloud discartWeakPoints(float threshold);

private:
  Eigen::Matrix4f axis_correction;
  VelodyneSpecification::Model velodyne_model;
};

class SensorsCalibration {
public:
  SensorsCalibration(void) :
    sensors_poses(1, Eigen::Affine3f::Identity()) {
  }

  SensorsCalibration(const std::string &calibration_file) :
    sensors_poses(KittiUtils::load_kitti_poses(calibration_file)){
  }

  Eigen::Affine3f ofSensor(const int idx) const {
    return sensors_poses[idx];
  }

  int sensorsCount(void) const {
    return sensors_poses.size();
  }

  Eigen::Affine3f getSensorPose(const Eigen::Affine3f system_pose, const int sensor_id) const;

private:
  std::vector<Eigen::Affine3f> sensors_poses;
};

class VelodyneMultiFrame {
public:

  typedef boost::shared_ptr<VelodyneMultiFrame> Ptr;

  VelodyneMultiFrame(const std::vector<std::string> &filenames_,
      const SensorsCalibration &calibration_,
      bool transform_pcd_files_ = false);

  VelodyneMultiFrame(const std::vector<std::string> &filenames_,
          const std::vector<VelodynePointCloud::Ptr> &clouds_,
          const SensorsCalibration &calibration_) : filenames(filenames_), clouds(clouds_), calibration(calibration_) {
  }

  void joinTo(pcl::PointCloud<PointWithSource> &output) const;

  void joinTo(pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &output, bool distinguish_rings = false) const;

  void joinTo(pcl::PointCloud<pcl::PointXYZI> &output) const;

  void joinTo(pcl::PointCloud<pcl::PointXYZ> &output) const;

  void subsample(float ratio);

  std::vector<std::string> filenames;
  std::vector<VelodynePointCloud::Ptr> clouds;
  SensorsCalibration calibration;
};

class VelodyneFileSequence {
public:
  VelodyneFileSequence(const std::vector<std::string> &filenames,
      const SensorsCalibration &calibration_,
      bool transform_pcd_files = false);

  bool hasNext(void);

  VelodyneMultiFrame getNext(void);

  VelodyneMultiFrame::Ptr getNextPtr(void);

  void reset(void);

  void next(void);

  bool hasPrev(void);

  VelodyneMultiFrame getPrev(void);

  VelodyneMultiFrame::Ptr getPrevPtr(void);

    int size(void) const {
    return filenames.size() / calibration.sensorsCount();
  }

  int getIndex() const {
    return index;
  }

  VelodyneMultiFrame::Ptr operator[](const int i) const;

private:
  const std::vector<std::string> filenames;
  const SensorsCalibration calibration;
  const bool transform_pcd_files;
  int index;
};

class PhaseFilter {

public:
  PhaseFilter(const float min_phase_, const float max_phase_) :
    min_phase(min_phase_), max_phase(max_phase_) {
  }

  void filter(VelodynePointCloud &cloud) const {
    for(VelodynePointCloud::iterator p = cloud.begin(); p < cloud.end();) {
      if(min_phase <= p->phase && p->phase < max_phase) {
        p++;
      } else {
        p = cloud.erase(p);
      }
    }
  }

  void filter(const VelodynePointCloud &input, VelodynePointCloud &slice, VelodynePointCloud &rest) const {
    for(VelodynePointCloud::const_iterator p = input.begin(); p < input.end(); p++) {
      if(min_phase <= p->phase && p->phase < max_phase) {
        slice.push_back(*p);
      } else {
        rest.push_back(*p);
      }
    }
  }

  void filter(const VelodynePointCloud &input, VelodynePointCloud &slice) const {
    for(VelodynePointCloud::const_iterator p = input.begin(); p < input.end(); p++) {
      if(min_phase <= p->phase && p->phase < max_phase) {
        slice.push_back(*p);
      }
    }
  }

  void filter(VelodyneMultiFrame &multiframe) const {
    for(std::vector<VelodynePointCloud::Ptr>::iterator cloud = multiframe.clouds.begin();
        cloud < multiframe.clouds.end(); cloud++) {
      filter(**cloud);
    }
  }

  float getMaxPhase() const {
    return max_phase;
  }

  float getMinPhase() const {
    return min_phase;
  }

private:
  float min_phase;
  float max_phase;
};

template <typename T>
void outlier_removal(typename pcl::PointCloud<T>::ConstPtr cloud,
                     pcl::PointCloud<T> &cloud_filtered,
                     const int mean_k, const float std_mul_thresh) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setMeanK(mean_k);
  filter.setStddevMulThresh(std_mul_thresh);

  filter.setInputCloud(cloud);
  filter.filter(cloud_filtered);
}

template <typename T>
void subsample_by_voxel_grid(typename pcl::PointCloud<T>::ConstPtr input, pcl::PointCloud<T> &subsampled,
                             const float leaf_size) {
  pcl::VoxelGrid<T> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);

  int original_size = input->size();
  grid.setInputCloud(input);
  grid.filter(subsampled);

  if(original_size == subsampled.size()) {
    typename pcl::PointCloud<T>::Ptr inliers(new pcl::PointCloud<T>);
    outlier_removal(input, *inliers, 10, 10.0);
    grid.setInputCloud(inliers);
    grid.filter(subsampled);
  }
}

}

#endif /* VELODYNEPOINTCLOUD_H_ */
