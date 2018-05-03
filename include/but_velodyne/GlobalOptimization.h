/*
 * GlobalOptimization.h
 *
 *  Created on: Sep 12, 2017
 *      Author: ivelas
 */

#ifndef GLOBALOPTIMIZATION_H_
#define GLOBALOPTIMIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/PoseGraphEdge.h>

namespace but_velodyne {

template <typename PointT>
void cloud2matrix(const pcl::PointCloud<PointT> &cloud, Eigen::MatrixXf &matrix) {
  matrix = Eigen::MatrixXf(cloud.size(), 3);
  for(int i = 0; i < cloud.size(); i++) {
    matrix(i, 0) = cloud[i].x;
    matrix(i, 1) = cloud[i].y;
    matrix(i, 2) = cloud[i].z;
  }
}

template <typename PointT>
void matrix2cloud(const Eigen::MatrixXf &matrix, pcl::PointCloud<PointT> &cloud) {
  bool hasZ = matrix.cols() > 2;
  cloud.resize(matrix.rows());
  for(int i = 0; i < matrix.rows(); i++) {
    cloud[i].x = matrix(i, 0);
    cloud[i].y = matrix(i, 1);
    if(hasZ) {
      cloud[i].z = matrix(i, 2);
    } else {
      cloud[i].z = 0.0;
    }
  }
}

template <typename PointT>
void reduceWeakestDimension(const pcl::PointCloud<PointT> &input, const Eigen::Matrix3f &covaraince,
    pcl::PointCloud<PointT> &reduced) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covaraince);
  Eigen::MatrixXf pca = eig.eigenvectors().rightCols(2);
  Eigen::MatrixXf pointMatrix;
  cloud2matrix(input, pointMatrix);
  matrix2cloud(pointMatrix*pca, reduced);
}

template <typename PointT>
void getClosestMatches(const typename pcl::PointCloud<PointT>::Ptr points, float query_ratio, std::vector<cv::DMatch> &matches) {
  pcl::IndicesPtr indicies(new std::vector<int>(points->size()));
  for(int i = 0; i < points->size(); i++) {
    indicies->at(i) = i;
  }
  random_shuffle(indicies->begin(), indicies->end());

  int queries_cnt = points->size()*query_ratio;
  std::vector<int>::iterator query_first = indicies->begin();
  std::vector<int>::iterator query_last = indicies->begin()+queries_cnt;
  std::vector<int> query_indicies(query_first, query_last);
  indicies->erase(query_first, query_last);
  matches.resize(queries_cnt);

  pcl::search::KdTree<PointT> kdtree;
  kdtree.setInputCloud(points, indicies);

  std::vector<int> knn_indices(1);
  std::vector<float> distances(1);
  for(int i = 0; i < query_indicies.size(); i++) {
    int qi = query_indicies[i];
    kdtree.nearestKSearch(points->at(qi), 1, knn_indices, distances);
    matches[i].queryIdx = qi;
    matches[i].trainIdx = knn_indices[0];
    matches[i].distance = sqrt(distances[0]);
  }
}

class Origin {
public:
  int pose_id, sensor_id;

  Origin(int pose_id_ = 0, int sensor_id_ = 0) :
    pose_id(pose_id_), sensor_id(sensor_id_) {
  }

  int edgeIdx(int poses_cnt) const {
    return pose_id + sensor_id*poses_cnt;
  }

  static Origin fromPointSource(const int point_src, const int poses_count) {
    return Origin(point_src%poses_count, point_src/poses_count);
  }
};

void printPoseGraphPrefix(const std::vector<Eigen::Affine3f> &poses,
    float covariance_diagonal = 0.1);

template <typename PointT>
void printPoseGraphMatches(const std::vector<Eigen::Affine3f> &poses,
    const pcl::PointCloud<PointT> &points,
    const std::vector<Origin> &origins,
    const std::vector<cv::DMatch> &matches,
    float covariance_diagonal = 0.001) {

  static const cv::Mat LANDMARK_COVARIANCE = cv::Mat::eye(3, 3, CV_32FC1)*covariance_diagonal;

  static int new_vertex = poses.size();
  for(std::vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    Origin o1 = origins[m->trainIdx];
    Origin o2 = origins[m->queryIdx];
    PointT pt1 = transformPoint(points[m->trainIdx], poses[o1.pose_id].inverse());
    PointT pt2 = transformPoint(points[m->queryIdx], poses[o2.pose_id].inverse());
    std::cout << PoseToLandmarkGraphEdge(o1.pose_id, new_vertex, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << std::endl;
    std::cout << PoseToLandmarkGraphEdge(o2.pose_id, new_vertex, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << std::endl;
    new_vertex++;
  }
}

}

#endif /* GLOBALOPTIMIZATION_H_ */
