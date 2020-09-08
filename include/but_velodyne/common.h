/*
 * common.h
 *
 *  Created on: Oct 10, 2017
 *      Author: ivelas
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

namespace but_velodyne {

void random_indices(const int total_count, const int new_count, std::vector<int> &output);

template <typename PointT>
void extract_indices(typename pcl::PointCloud<PointT>::ConstPtr in_cloud,
    pcl::PointIndices::ConstPtr indices,
    pcl::PointCloud<PointT> &out_cloud,
    bool negative = false) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(out_cloud);
}

template <typename T>
void extract_indices(const std::vector<T> &input,
    const std::vector<int> &indices,
    std::vector<T> &output) {
  std::vector<T> raw_output;
  for(std::vector<int>::const_iterator i = indices.begin(); i < indices.end(); i++) {
    raw_output.push_back(input[*i]);
  }
  output = raw_output;
}

template <typename T>
void save_vector(const std::vector<T> &data,
    const std::string &fn) {
  std::ofstream file(fn.c_str());
  for(typename std::vector<T>::const_iterator d = data.begin(); d < data.end(); d++) {
    file << *d << std::endl;
  }
}

template <typename T>
void load_vector_from_file(const std::string &filename, std::vector<T> &output) {
  std::ifstream file(filename.c_str());
  T element;
  while(file >> element) {
    output.push_back(element);
  }
}

template <typename T1, typename T2>
void load_vectors_from_file(const std::string &filename, std::vector<T1> &output1, std::vector<T2> &output2) {
  std::ifstream file(filename.c_str());
  T1 element1;
  T2 element2;
  while(file >> element1 >> element2) {
    output1.push_back(element1);
    output2.push_back(element2);
  }
}

float sq(const float x);

float get_rand(float maxval);

void invert_indices(const std::vector<int> &labels, std::vector< pcl::PointIndices::Ptr > &inverted_indices);

template <typename PointT>
void filter_by_range(const pcl::PointCloud<PointT> &input, pcl::PointCloud<PointT> &output, const float range_threshold) {
  output.resize(input.size());
  typename pcl::PointCloud<PointT>::iterator out_it = output.begin();
  for(typename pcl::PointCloud<PointT>::const_iterator p = input.begin(); p < input.end(); p++) {
    float range = computeRange(*p);
    if(range < range_threshold) {
      *out_it = *p;
      out_it++;
    }
  }
  output.erase(out_it, output.end());
}

float gauss(const float x, const float mean, const float variance);

float harmonicalAverage(const float a, const float b);

template <typename PointT>
float distance_pts(const PointT &p1, const PointT &p2) {
  return (p1.getVector3fMap() - p2.getVector3fMap()).norm();
}

bool endsWith(const std::string &fullString, const std::string &ending);

template <typename T>
std::ostream& operator<<(std::ostream &stream, const std::vector<T> &items) {
  stream << "[ ";
  for(typename std::vector<T>::const_iterator i = items.begin(); i < items.end(); i++) {
    stream << *i << ", ";
  }
  stream << "]" << std::endl;
  return stream;
}

}

#endif /* COMMON_H_ */
