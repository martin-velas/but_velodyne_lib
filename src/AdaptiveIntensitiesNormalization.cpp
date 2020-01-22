/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 16/01/2020
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

#include <but_velodyne/AdaptiveIntensitiesNormalization.h>
#include <but_velodyne/GlobalOptimization.h>

using namespace std;
using namespace pcl;
using namespace cv;

namespace but_velodyne {

    typedef PointWithSource PointType;

    class BinIndex {
    public:
        int ring, dist;

        BinIndex(const int ring_, const int dist_) :
                ring(ring_), dist(dist_) {
        }

        bool operator=(const BinIndex &o) const {
          return this->ring == o.ring && this->dist == o.dist;
        }

        bool operator<(const BinIndex &o) const {
          if (this->ring == o.ring) {
            return this->dist < o.dist;
          } else {
            return this->ring < o.ring;
          }
        }
    };

    float transform_gauss(float x1, float u1, float s1, float u2, float s2) {
      float x2;
      if (s1 < 0.01) {
        x2 = x1 - u1 + u2;
      } else {
        x2 = ((x1 - u1) / s1) * s2 + u2;
      }

      return MIN(MAX(x2, 0.0), 2 * u2);
    }

    void normalize_intensities(const Mat &data, const PointCloud<PointType> &in_cloud,
                               float expected_mean, float expected_std_dev,
                               PointCloud<PointType>::Ptr &out_cloud) {
      *out_cloud = in_cloud;

      typedef map <BinIndex, vector<float>> GridT;
      GridT desc_grid;
      for (int i = 0; i < data.rows; i++) {
        BinIndex bin(data.at<int>(i, 0), data.at<int>(i, 1));
        desc_grid[bin].push_back(in_cloud[i].intensity);
      }

      for (GridT::iterator bin = desc_grid.begin(); bin != desc_grid.end(); bin++) {
        float mean = accumulate(bin->second.begin(), bin->second.end(), 0.0) / bin->second.size();
        float variance = 0;
        for (vector<float>::iterator x = bin->second.begin(); x < bin->second.end(); x++) {
          variance += pow(*x - mean, 2);
        }
        variance /= bin->second.size();
        bin->second.clear();
        bin->second.push_back(mean);
        bin->second.push_back(sqrt(variance));
      }

      for (int i = 0; i < data.rows; i++) {
        BinIndex bin(data.at<int>(i, 0), data.at<int>(i, 1));
        out_cloud->at(i).intensity = transform_gauss(in_cloud[i].intensity,
                                             desc_grid[bin][0], desc_grid[bin][1],
                                             expected_mean, expected_std_dev);
      }
    }

    int quantize(float value, float resolution) {
      return floor(value / resolution);
    }


    void AdaptiveIntensitiesNormalization::run(PointCloud<PointType>::Ptr sum_cloud,
                                               const SensorsCalibration &calibration,
                                               const vector <Eigen::Affine3f> &poses,
                                               PointCloud<PointType>::Ptr &out_cloud) {

      const int DATA_CHANNELS = 2;
      const float DISTANCE_RES = 0.2;

      PointCloud<PointXYZ> sum_origin_positions;
      sum_origin_positions.resize(sum_cloud->size());
      for (int i = 0; i < sum_cloud->size(); i++) {
        Origin o = Origin::fromPointSource(sum_cloud->at(i).source, poses.size());
        sum_origin_positions[i].getVector3fMap() = calibration.getSensorPose(poses[o.pose_id],
                                                                             o.sensor_id).translation();
      }
      cerr << "Computed " << sum_origin_positions.size() << " origins." << endl;

      cerr << "Compute data for normalization ..." << endl;
      Mat data(sum_cloud->size(), DATA_CHANNELS, CV_32SC1);
      PointCloud<PointType>::Ptr vis_cloud(new PointCloud<PointType>);
      *vis_cloud += *sum_cloud;
      subsample_cloud<PointType>(vis_cloud, 0.01);

      for (int pi = 0; pi < sum_cloud->size(); pi++) {
        PointType pt = sum_cloud->at(pi);
        Eigen::Vector3f pt_to_origin = sum_origin_positions[pi].getVector3fMap() - pt.getVector3fMap();
        float distance = pt_to_origin.norm();

        data.at<int>(pi, 0) = pt.ring;
        data.at<int>(pi, 1) = quantize(distance, DISTANCE_RES);
      }

      cerr << "Intensities normalization ..." << endl;
      normalize_intensities(data, *sum_cloud, expected_mean, expected_std_dev,
              out_cloud);
    }

}
