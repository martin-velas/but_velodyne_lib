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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

inline PointCloud<PointXYZI>& operator += (PointCloud<PointXYZI> &this_cloud, const VelodynePointCloud& other)
{
  // Make the resultant point cloud take the newest stamp
  if (other.header.stamp > this_cloud.header.stamp)
    this_cloud.header.stamp = other.header.stamp;

  size_t nr_points = this_cloud.points.size ();
  this_cloud.points.resize (nr_points + other.points.size ());
  for (size_t i = nr_points; i < this_cloud.points.size (); ++i) {
    copyXYZ(other.points[i - nr_points], this_cloud.points[i]);
    this_cloud.points[i].intensity = other.points[i - nr_points].intensity;
  }

  this_cloud.width    = static_cast<uint32_t>(this_cloud.points.size ());
  this_cloud.height   = 1;
  if (other.is_dense && this_cloud.is_dense)
    this_cloud.is_dense = true;
  else
    this_cloud.is_dense = false;
  return this_cloud;
}

void normalizeIntensity(const PointCloud<PointXYZI> &in, PointCloud<PointXYZI> &out) {
  out.resize(in.size());
  float mean = 0;
  for(int i = 0; i < in.size(); i++) {
    copyXYZ(in[i], out[i]);
    mean += in[i].intensity;
  }

  mean /= in.size();

  for(int i = 0; i < in.size(); i++) {
    if(mean < 0.5) {
      out[i].intensity = in[i].intensity/mean*0.5;
    } else {
      out[i].intensity = (in[i].intensity-mean)/(1-mean)*0.5 + 0.5;
    }
  }
}

enum Colorization {
  GREYSCALE_BY_INTENSITY,
  COLOR_BY_INTENSITY,
  RANDOM_COLORS
};

std::istream& operator>> (std::istream &in, Colorization &c) {
  string token;
  in >> token;
  boost::to_upper(token);
  if (token == "GREYSCALE_BY_INTENSITY") {
    c = GREYSCALE_BY_INTENSITY;
  } else if (token == "COLOR_BY_INTENSITY") {
    c = COLOR_BY_INTENSITY;
  } else if (token == "RANDOM_COLORS") {
    c = RANDOM_COLORS;
  } else {
      throw boost::program_options::validation_error(
          boost::program_options::validation_error::invalid_option_value, "colorization");
  }
  return in;
}

bool parse_arguments(int argc, char **argv,
                     int &preserve_ratio,
                     int &subsampling_rate,
                     float &subsampling_resolution,
                     bool &normalize_intensity,
                     Colorization &colorization,
                     vector<Eigen::Affine3f> &poses,
                     vector<string> &clouds_to_process,
                     string &output_file) {
  string pose_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->default_value(""), "KITTI poses file.")
      ("preserve_ratio", po::value<int>(&preserve_ratio)->default_value(preserve_ratio), "Preserve only each Nth cloud for visualization")
      ("subsampling_rate", po::value<int>(&subsampling_rate)->default_value(subsampling_rate), "Run VoxelGrid filter after each Nth frame")
      ("subsampling_resolution", po::value<float>(&subsampling_resolution)->default_value(subsampling_resolution), "Minimal VoxelGrid leaf size.")
      ("normalize_intensity", po::bool_switch(&normalize_intensity), "Normalize intensity values.")
      ("colorization", po::value<Colorization>(&colorization)->default_value(colorization), "Colorization of the cloud [GREYSCALE_BY_INTENSITY|COLOR_BY_INTENSITY|RANDOM_COLORS]")
      ("output_file", po::value<string>(&output_file)->default_value(""), "Output PCD file")
   ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1) {
        std::cerr << desc << std::endl;
        return false;
    }

    try {
        po::notify(vm);
    } catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if(!pose_filename.empty()) {
      poses = KittiUtils::load_kitti_poses(pose_filename);
    }

    return true;
}


int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  int preserve_ratio = 1;
  int subsampling_rate = 100;
  float subsampling_resolution = 0.1;
  bool normalize_intensity = false;
  Colorization colorization = GREYSCALE_BY_INTENSITY;
  string output_pcd_file;

  if(!parse_arguments(argc, argv,
      preserve_ratio, subsampling_rate, subsampling_resolution,
      normalize_intensity, colorization,
      poses, filenames,
      output_pcd_file)) {
    return EXIT_FAILURE;
  }

  Visualizer3D visualizer;
  VelodynePointCloud cloud;
  PointCloud<PointXYZI> sum_cloud;
  int cloud_i;
  for (cloud_i = 0; cloud_i < filenames.size(); cloud_i++) {

    if(cloud_i%preserve_ratio != 0) {
      continue;
    }

    std::cerr << "Processing KITTI file: " << filenames[cloud_i] << std::endl << std::flush;
    if (filenames[cloud_i].find(".pcd") != std::string::npos) {
      pcl::io::loadPCDFile(filenames[cloud_i], cloud);
      //cloud.setImageLikeAxisFromKitti();
    } else {
      VelodynePointCloud::fromKitti(filenames[cloud_i], cloud);
    }

    if (!poses.empty()) {
      transformPointCloud(cloud, cloud, poses[cloud_i]);
      float x, y, z, rx, ry, rz;
      getTranslationAndEulerAngles(poses[cloud_i], x, y, z, rx, ry, rz);
      cout << "Pose of the cloud #" << cloud_i << " " << x << " " << y << " " << z << " " <<
          rx << " " << ry << " " << rz  << endl;
    }

    sum_cloud += cloud;

    if ((cloud_i % subsampling_rate == 0 && cloud_i > 0) || cloud_i == filenames.size()-1) {
      cerr << "Subsampling by VoxelGrid" << endl;
      pcl::VoxelGrid<PointXYZI> grid;
      grid.setLeafSize(subsampling_resolution, subsampling_resolution, subsampling_resolution);
      grid.setInputCloud(sum_cloud.makeShared());
      grid.filter(sum_cloud);
    }
  }

  if(normalize_intensity) {
    normalizeIntensity(sum_cloud, sum_cloud);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
  if(colorization == COLOR_BY_INTENSITY) {
    rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, false);
  } else {
    rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);
  }

  if(!output_pcd_file.empty()) {
    io::savePCDFileBinary(output_pcd_file, *rgb_cloud);
  }

  visualizer.addColorPointCloud(rgb_cloud);

  if(!poses.empty()) {
    //poses.erase(poses.begin()+cloud_i, poses.end());
    visualizer.setColor(0, 100, 200).addPoses(poses);
  }
  visualizer.show();

  return EXIT_SUCCESS;
}
