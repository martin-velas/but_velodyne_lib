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

namespace but_velodyne {

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

}

#endif /* GLOBALOPTIMIZATION_H_ */
