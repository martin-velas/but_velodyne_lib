/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 08/03/2019
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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    SensorsCalibration &calibration,
    vector<string> &clouds_to_process,
    vector<float> &start_angles, vector<float> &end_angles,
    string &out_dir) {

  string pose_filename, sensors_pose_file;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("start_angle,s", po::value< vector<float> >(&start_angles)->multitoken(), "Beginnings of angle ranges.")
    ("end_angle,e", po::value< vector<float> >(&end_angles)->multitoken(), "Ends of angle ranges.")
    ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.")
    ("calibration,c", po::value<string>(&sensors_pose_file)->default_value(""), "Sensors poses (calibration).")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }
  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  if(!(start_angles.size() == end_angles.size() && end_angles.size() == calibration.sensorsCount())) {
    cerr << "Number of sensors is nor consistent" << endl;
    cerr << desc << endl;
    return false;
  }

  return true;
}

class AngleFilter {
public:
  AngleFilter(const float start_angle_, const float end_angle_) :
    start_angle(start_angle_), end_angle(end_angle_) {
  }

  float filter(const VelodynePointCloud &input, VelodynePointCloud &output) {
    output = input;
    float sum_phase = 0.0;
    for(VelodynePointCloud::iterator p = output.begin(); p < output.end();) {
      const float angle = VelodynePointCloud::horizontalAngle(p->z, p->x);
      if(start_angle < angle && angle < end_angle) {
        p++;
        sum_phase += p->phase;
      } else {
        p = output.erase(p);
      }
    }
    return sum_phase / output.size();
  }

private:
  float start_angle, end_angle;
};

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> slice_poses;
  SensorsCalibration calibration;
  vector<string> clouds_to_process;
  vector<float> start_angles, end_angles;
  string out_dir;
  if(!parse_arguments(argc, argv, calibration, clouds_to_process, start_angles, end_angles, out_dir)) {
    return EXIT_FAILURE;
  }

  vector<AngleFilter> filters;
  for(int i = 0; i < start_angles.size(); i++) {
    AngleFilter filter(start_angles[i], end_angles[i]);
    filters.push_back(filter);
  }
  VelodyneFileSequence file_sequence(clouds_to_process, calibration);
  for(int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      VelodynePointCloud cloud = *multiframe.clouds[sensor_i];
      cout << filters[sensor_i].filter(cloud, cloud) << " ";
      boost::filesystem::path first_cloud(multiframe.filenames[sensor_i]);
      io::savePCDFileBinary(out_dir + "/" + first_cloud.filename().string(), cloud);
    }
    cout << endl;
  }

  return EXIT_SUCCESS;
}
