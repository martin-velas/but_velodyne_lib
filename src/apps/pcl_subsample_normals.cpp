/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
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

#include <cstdlib>
#include <cstdio>

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

typedef Normal PointType;

int main(int argc, char** argv) {

  if(argc != 4) {
    cerr << "ERROR, expecting arguments: [in-normals.pcd] [in-indices.txt] [out-normals.pcd]" << endl;
    return EXIT_FAILURE;
  }

  PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
  io::loadPCDFile(argv[1], *cloud);

  PointIndices::Ptr indices(new PointIndices);
  load_vector_from_file(argv[2], indices->indices);

  extract_indices(cloud, indices, *cloud);

  io::savePCDFileBinary(argv[3], *cloud);

  return EXIT_SUCCESS;
}
