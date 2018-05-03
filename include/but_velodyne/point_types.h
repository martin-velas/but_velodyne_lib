/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __BUT_VELODYNE_POINT_TYPES_H__
#define __BUT_VELODYNE_POINT_TYPES_H__

#include <pcl/point_types.h>

namespace but_velodyne
{
  struct PointXYZILP
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t label;                     ///< point label
    float    prob;                      ///< label probability
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  typedef PointXYZILP LabeledPoint;

  struct PointXYZIRS
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t source;                    ///< source index (pose, sensor, etc.)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  typedef PointXYZIRS PointWithSource;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(but_velodyne::PointXYZILP,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (float, prob, prob));

POINT_CLOUD_REGISTER_POINT_STRUCT(but_velodyne::PointXYZIRS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, source, source));

#endif // __BUT_VELODYNE_POINT_TYPES_H__
