/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 03/08/2020
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

#include <but_velodyne/VelodyneMultiFrameSequence.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace pcl;

namespace but_velodyne {

VelodyneMultiFrame::VelodyneMultiFrame(const std::vector<std::string> &filenames_,
                                       const SensorsCalibration &calibration_,
                                       bool transform_pcd_files) :
        filenames(filenames_),
        calibration(calibration_) {
  if(filenames.size() != calibration.sensorsCount()) {
    cerr << "WARNING: different number of files (" << filenames.size() <<
         ") and sensor_poses (" << calibration.sensorsCount() << ")" << endl;
  }
  for(int i = 0; i < filenames.size(); i++) {
    if(endsWith(filenames[i], ".lcd")) {
      line_clouds.push_back(LineCloud::Ptr(new LineCloud));
      LineCloud::fromFile(filenames[i], *line_clouds[i]);
    } else {
      clouds.push_back(VelodynePointCloud::Ptr(new VelodynePointCloud));
      VelodynePointCloud::fromFile(filenames[i], *clouds[i], transform_pcd_files);
    }
  }
}

void VelodyneMultiFrame::transform(const Eigen::Affine3f &pose) {
  for(int i = 0; i < this->calibration.sensorsCount(); i++) {
    const Eigen::Affine3f &C = this->calibration.ofSensor(i);
    const Eigen::Affine3f &T = C.inverse()*pose*C;

    if(this->hasPointClouds()) {
      transformPointCloud(*this->clouds[i], *this->clouds[i], T);
    }

    if(this->hasLineClouds()) {
      this->line_clouds[i]->transform(T.matrix());
    }
  }
}

void VelodyneMultiFrame::save(const string &out_dir) const {
  for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
    boost::filesystem::path cloud_path(filenames[sensor_i]);
    string out_filename = out_dir + "/" + cloud_path.filename().string();
    if(this->hasPointClouds()) {
      io::savePCDFileBinary(out_filename, *clouds[sensor_i]);
    }
    if(this->hasLineClouds()) {
      ofstream out_file(out_filename.c_str());
      out_file << *line_clouds[sensor_i];
    }
  }
}

void VelodyneMultiFrame::joinTo(PointCloud<PointWithSource> &output) const {
  for(int sensor_i = 0; sensor_i < clouds.size(); sensor_i++) {
    PointCloud<PointWithSource> transformed;
    copyPointCloud(*clouds[sensor_i], transformed);
    pcl::transformPointCloud(transformed, transformed, calibration.ofSensor(sensor_i));
    for(int i = 0; i < transformed.size(); i++) {
      transformed[i].source = sensor_i;
    }
    output += transformed;
  }
}

void VelodyneMultiFrame::joinTo(pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &output, bool distinguish_rings) const {
  int rings_count = 0;
  for(int i = 0; i < clouds.size(); i++) {
    VelodynePointCloud transformed;
    pcl::transformPointCloud(*clouds[i], transformed, calibration.ofSensor(i));
    if(distinguish_rings) {
      transformed.estimateModel();
      if(i > 0) {
        for(VelodynePointCloud::iterator pt = transformed.begin(); pt < transformed.end(); pt++) {
          pt->ring += rings_count;
        }
      }
      rings_count += transformed.ringCount();
    }
    output += transformed;
  }
}

void VelodyneMultiFrame::joinTo(PointCloud<PointXYZI> &output) const {
  PointCloud<velodyne_pointcloud::VelodynePoint> mid_output;
  joinTo(mid_output);
  output.resize(mid_output.size());
  for(int i = 0; i < output.size(); i++) {
    output[i].intensity = mid_output[i].intensity;
    copyXYZ(mid_output[i], output[i]);
  }
}

void VelodyneMultiFrame::joinTo(PointCloud<PointXYZ> &output) const {
  PointCloud<velodyne_pointcloud::VelodynePoint> mid_output;
  joinTo(mid_output);
  output.resize(mid_output.size());
  for(int i = 0; i < output.size(); i++) {
    copyXYZ(mid_output[i], output[i]);
  }
}

void VelodyneMultiFrame::joinTo(LineCloud &output) const {
  if(this->hasLineClouds()) {
    for(int i = 0; i < line_clouds.size(); i++) {
      LineCloud transformed;
      line_clouds[i]->transform(calibration.ofSensor(i).matrix(), transformed);
      output += transformed;
    }
  }
}

VelodyneFileSequence::VelodyneFileSequence(const std::vector<std::string> &filenames_,
                                           const SensorsCalibration &calibration_,
                                           bool transform_pcd_files_) :
        filenames(filenames_),
        calibration(calibration_),
        transform_pcd_files(transform_pcd_files_),
        index(0) {
}

bool VelodyneFileSequence::hasNext(void) {
  return (index+1)*calibration.sensorsCount() <= filenames.size();
}

VelodyneMultiFrame::Ptr VelodyneFileSequence::operator[](const int i) const {
  vector<string>::const_iterator first = filenames.begin() + i*calibration.sensorsCount();
  vector<string>::const_iterator last = first + calibration.sensorsCount();
  vector<string> frame_filenames(first, last);
  return VelodyneMultiFrame::Ptr(new VelodyneMultiFrame(frame_filenames, calibration, transform_pcd_files));
}

VelodyneMultiFrame VelodyneFileSequence::getNext(void) {
  assert(hasNext());
  return *getNextPtr();
}

VelodyneMultiFrame::Ptr VelodyneFileSequence::getNextPtr(void) {
  assert(hasNext());
  return (*this)[index++];
}

void VelodyneFileSequence::next(void) {
  index++;
}

void VelodyneFileSequence::reset(void) {
  index = 0;
}

bool VelodyneFileSequence::hasPrev(void) {
  return index - 1 >= 0;
}

VelodyneMultiFrame VelodyneFileSequence::getPrev(void) {
  assert(hasPrev());
  return *getPrevPtr();
}

VelodyneMultiFrame::Ptr VelodyneFileSequence::getPrevPtr(void) {
  assert(hasPrev());
  return (*this)[--index];
}

}
