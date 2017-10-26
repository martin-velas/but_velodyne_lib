/*
 * Clustering.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: ivelas
 */

#include <but_velodyne/Clustering.h>
#include <but_velodyne/NormalsEstimation.h>

#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>

#include <algorithm>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;

namespace but_velodyne {

template <class PointT>
void Clustering<PointT>::clusterKMeans(const pcl::PointCloud<PointT> &points, const pcl::PointCloud<pcl::Normal> &normals,
    const int K, std::vector<int> &indices) {
  cv::Mat data(points.size(), 4, CV_32F);
  for (int i = 0; i < points.size(); i++) {
    getPlaneCoefficients(normals[i], points[i].getVector3fMap(),
        data.at<float>(i, 0),
        data.at<float>(i, 1),
        data.at<float>(i, 2),
        data.at<float>(i, 3));
  }
  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1);
  int attempts = 1;
  cv::kmeans(data, K, labels, termination, attempts, cv::KMEANS_PP_CENTERS);
  labels.copyTo(indices);
}

template <class PointT>
void Clustering<PointT>::clusterKMeans(const pcl::PointCloud<PointT> &points, const int K, std::vector<int> &indices) {
  cv::Mat data(points.size(), 3, CV_32F);
  for (int i = 0; i < points.size(); i++) {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
        data.at<float>(i, 2) = points[i].z;
  }
  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1);
  int attempts = 1;
  cv::kmeans(data, K, labels, termination, attempts, cv::KMEANS_PP_CENTERS);
  labels.copyTo(indices);
}

template <class PointT>
void Clustering<PointT>::clusterEM(const pcl::PointCloud<PointT> &points, const pcl::PointCloud<pcl::Normal> &normals,
    const int K, std::vector<int> &indices, std::vector<float> &clusterProbs, const float train_ratio) {
  assert(train_ratio <= 1.000000001 && train_ratio > 0.0f);

  int train_size = points.size() * train_ratio;
  vector<bool> mask(points.size(), false);
  for(int i = 0; i < train_size; i++) {
    mask[i] = true;
  }
  random_shuffle(mask.begin(), mask.end());

  cv::Mat train_data(train_size, 4, CV_32F);
  cv::Mat all_data(points.size(), 4, CV_32F);
  int train_i = 0;
  for (int points_i = 0; points_i < points.size(); points_i++) {
    getPlaneCoefficients(normals[points_i], points[points_i].getVector3fMap(),
        all_data.at<float>(points_i, 0),
        all_data.at<float>(points_i, 1),
        all_data.at<float>(points_i, 2),
        all_data.at<float>(points_i, 3));
    if(mask[points_i]) {
      all_data.row(points_i).copyTo(
          train_data.row(train_i));
      train_i++;
    }
  }

  cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
  em_model->setClustersNumber(K);
  em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);
  em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1));

  em_model->trainEM(train_data);

  indices.resize(points.size());
  clusterProbs.resize(points.size());
  for(int i = 0; i < points.size(); i++) {
    cv::Mat probs(1, K, CV_64FC1);
    cv::Vec2d likelyhood_idx = em_model->predict2(all_data.row(i), probs);
    indices[i] = likelyhood_idx(1);
    clusterProbs[i] = probs.at<double>(indices[i]);
  }
}

template <class PointT>
void Clustering<PointT>::refineByKMeans(const PointCloud<PointT> &points,// const PointCloud<Normal> &normals,
    const int cluster_size, vector<int> &init_indices, vector<int> &refined_indices) {
  int init_K = (*max_element(init_indices.begin(), init_indices.end())) + 1;
  vector< vector<int> > sub_labels(init_K);
  vector<int> idx_offsets(init_K, 0);
  for(int init_c = 0; init_c < init_K; init_c++) {
    PointCloud<PointT> sub_points;
    //PointCloud<Normal> sub_normals;
    for(int i = 0; i < init_indices.size(); i++) {
      if(init_indices[i] == init_c) {
        sub_points.push_back(points[i]);
        //sub_normals.push_back(normals[i]);
      }
    }
    int K = sub_points.size()/cluster_size;
    //clusterKMeans(sub_points, sub_normals, K, sub_labels[init_c]);
    clusterKMeans(sub_points, K, sub_labels[init_c]);
    if(init_c+1 < init_K) {
      idx_offsets[init_c+1] = idx_offsets[init_c] + K;
    }
  }
  refined_indices.resize(init_indices.size());
  vector<int> sub_labels_idxs(init_K, 0);
  for(int i = 0; i < init_indices.size(); i++) {
    int init_c = init_indices[i];
    refined_indices[i] = idx_offsets[init_c] + sub_labels[init_c][sub_labels_idxs[init_c]];
    sub_labels_idxs[init_c]++;
  }
}


template class Clustering<PointXYZ>;
template class Clustering<PointXYZI>;
template class Clustering<PointXYZRGB>;
template class Clustering<VelodynePoint>;

} /* namespace but_velodyne */
