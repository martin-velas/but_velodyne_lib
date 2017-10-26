/*
 * common.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: ivelas
 */

#include <pcl/common/common.h>

#include <but_velodyne/common.h>
#include <algorithm>

using namespace std;
using namespace pcl;

namespace but_velodyne {

void random_indices(const int total_count, const int new_count, vector<int> &output) {
  output.resize(total_count);
  for(int i = 0; i < total_count; i++) {
    output[i] = i;
  }
  random_shuffle(output.begin(), output.end());
  output.resize(new_count);
}

float sq(const float x) {
  return x*x;
}

float get_rand(float maxval) {
  return rand()/(RAND_MAX/maxval/2)-maxval;
}

void invert_indices(const vector<int> &labels, vector< PointIndices::Ptr > &inverted_indices) {
  int max_label = *max_element(labels.begin(), labels.end());
  inverted_indices.resize(max_label+1);
  for(int c = 0; c <= max_label; c++) {
    inverted_indices[c].reset(new PointIndices);
  }
  for(int i = 0; i < labels.size(); i++) {
    inverted_indices[labels[i]]->indices.push_back(i);
  }
}

}

