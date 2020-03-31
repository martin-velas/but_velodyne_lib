/*
 * Odometry estimation by collar line segments of Velodyne scan.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
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
#include <libgen.h>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


class CLSMap {

public:
  LineCloud all_lines;

  PointCloud<pcl::PointXYZ>::Ptr getMiddlesPtr(void) {
    return all_lines.getMiddles();
  }

  int size(void) {
    return all_lines.size();
  }

  void add(const LineCloud &lines, const int frame_id) {
    all_lines.append(lines, frame_id);
    cerr << "all: " << all_lines.size() << "; new: " << lines.size() << endl;
  }

  void prune(const float ratio) {
    random_shuffle(all_lines.begin(), all_lines.end());
    all_lines.erase(all_lines.begin() + all_lines.size() * (1.0 - ratio),
        all_lines.end());
  }
};


class CollarLinesRegistrationToMap {
public:
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;

  CollarLinesRegistrationToMap(CollarLinesRegistrationPipeline::Parameters pipeline_params_,
                               CollarLinesRegistration::Parameters registration_params_,
                               const float prune_ratio_, const bool &visualization) :
                                 filter(pipeline_params_.linesPerCellPreserved),
                                 params(pipeline_params_),
                                 registration_params(registration_params_),
                                 prune_ratio(prune_ratio_), indexed(false), last_frame_id(-1) {
    if(visualization) {
      vis = Visualizer3D::getCommonVisualizer();
    }
  }

  void addToMap(const std::vector<VelodynePointCloud::Ptr> &point_clouds,
      const SensorsCalibration &calibration, const Eigen::Affine3f &pose) {
    PolarGridOfClouds polar_grid(point_clouds, calibration);
    LineCloud line_cloud(polar_grid, params.linesPerCellGenerated, filter);
    addToMap(line_cloud, pose);
  }

  Eigen::Affine3f runMapping(const VelodyneMultiFrame &multiframe,
      const SensorsCalibration &calibration, const Eigen::Affine3f &init_pose,
      vector<CLSMatch> &matches) {
    PointCloud<PointXYZ> target_cloud_vis;
    if(vis) {
      multiframe.joinTo(target_cloud_vis);
      vis->keepOnlyClouds(0).setColor(200,0,200).addPointCloud(target_cloud_vis, init_pose.matrix())
          .setColor(150,150,150).addPointCloud(*lines_map.all_lines.getMiddles()).show();
    }

    if(!indexed) {
      buildKdTree();
    }
    PolarGridOfClouds target_polar_grid(multiframe.clouds, calibration);
    LineCloud target_line_cloud(target_polar_grid, params.linesPerCellGenerated, filter);
    Eigen::Affine3f refined_pose = registerLineCloud(target_line_cloud, init_pose, matches);
    addToMap(target_line_cloud, refined_pose);
    if(lines_map.size()*prune_ratio > target_line_cloud.size()) {
      cerr << "[DEBUG] Before pruning: " << lines_map.size() << endl;
      lines_map.prune(prune_ratio);
      indexed = false;
      cerr << "[DEBUG] After pruning: " << lines_map.size() << endl;
    }

    if(vis) {
      vis->keepOnlyClouds(1).setColor(0,200,0)
          .addPointCloud(target_cloud_vis, refined_pose.matrix()).show();
    }

    return refined_pose;
  }

protected:

  void addToMap(const LineCloud &line_cloud,
                const Eigen::Affine3f &pose) {
    LineCloud line_cloud_transformed;
    line_cloud.transform(pose.matrix(), line_cloud_transformed);
    lines_map.add(line_cloud_transformed, ++last_frame_id);
    indexed = false;
  }

  void buildKdTree(void) {
    map_kdtree.setInputCloud(lines_map.getMiddlesPtr());
    indexed = true;
  }

  Eigen::Affine3f registerLineCloud(const LineCloud &target,
      const Eigen::Affine3f &initial_transformation, vector<CLSMatch> &matches) {
    Termination termination(params.term_params);
    Eigen::Matrix4f transformation = initial_transformation.matrix();
    CollarLinesRegistration icl_fitting(lines_map.all_lines, map_kdtree, target,
                                        registration_params, transformation);
    while (!termination()) {
      float error = icl_fitting.refine();
      termination.addNewError(error);
      transformation = icl_fitting.getTransformation();
    }
    icl_fitting.getLastMatches(matches);
    return Eigen::Affine3f(transformation);
  }

private:
  float prune_ratio;
  CollarLinesFilter filter;
  CLSMap lines_map;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree;
  bool indexed;
  Visualizer3D::Ptr vis;
  int last_frame_id;
};

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    vector<Eigen::Affine3f> &init_poses, SensorsCalibration &calibration,
    vector<string> &clouds_to_process, float &prune_ratio, bool &visualization,
    string &matches_output_file, float &matches_portion) {
  string pose_file, sensors_pose_file;

  po::options_description desc("Collar Lines Registration of Velodyne scans against the map\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  registration_parameters.prepareForLoading(desc);
  pipeline_parameters.prepareForLoading(desc);
  desc.add_options()
    ("help,h", "produce help message")
    ("poses", po::value<string>(&pose_file)->required(),
      "File with initial poses")
    ("sensors_pose_file", po::value<string>(&sensors_pose_file)->default_value(""),
        "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
    ("prune_ratio", po::value<float>(&prune_ratio)->default_value(0.1),
        "How many lines (portion) should be discarted from the map.")
    ("visualize", po::bool_switch(&visualization),
        "Run visualization")
    ("matches_output", po::value<string>(&matches_output_file)->default_value(""),
        "Output file for the matches used in final registration.")
    ("matches_portion", po::value<float>(&matches_portion)->default_value(0.9),
        "Keep this portion of best matches (by the distance)")
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
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }

  init_poses = KittiUtils::load_kitti_poses(pose_file);

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  return true;
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  vector<Eigen::Affine3f> init_poses;
  vector<string> clouds_filenames;
  SensorsCalibration calibration;
  float prune_ratio;
  bool visualization;
  string matches_output_filename;
  float matches_portion;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters, init_poses, calibration,
      clouds_filenames, prune_ratio, visualization, matches_output_filename, matches_portion)) {
    return EXIT_FAILURE;
  }

  CollarLinesRegistrationToMap registration(
      pipeline_parameters, registration_parameters, prune_ratio, visualization);

  ofstream matches_output;
  if(!matches_output_filename.empty()) {
    matches_output.open(matches_output_filename.c_str());
  }
  CLSMatchByCoeffComparator clsMatchByCoeffComparator;

  VelodyneFileSequence sequence(clouds_filenames, calibration);
  vector<Eigen::Affine3f> refined_poses;
  for (int frame_i = 0; sequence.hasNext(); frame_i++) {

    VelodyneMultiFrame multiframe = sequence.getNext();

    Eigen::Affine3f pose;
    if(frame_i == 0) {
      pose = init_poses.front().matrix();
      registration.addToMap(multiframe.clouds, calibration, pose);

    } else {
      Eigen::Affine3f init_pose = refined_poses.back() * (init_poses[frame_i-1].inverse() * init_poses[frame_i]);
      vector<CLSMatch> matches;
      pose = registration.runMapping(multiframe, calibration, init_pose, matches);
      cerr << "Matches (unfiltered): " << matches.size() << endl;

      sort(matches.begin(), matches.end(), clsMatchByCoeffComparator);
      matches.erase(matches.begin() + matches_portion*matches.size(), matches.end());
      cerr << "Matches (filtered " << matches_portion*100 << "%): " << matches.size() << endl;

      if(matches_output.is_open()) {
        for(vector<CLSMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
          PointXYZ real_src_point = transformPoint(m->src, refined_poses[m->src_frame_id].inverse());
          matches_output
            << m->src_frame_id << " "
            << real_src_point.x << " " << real_src_point.y << " " << real_src_point.z << " "
            << frame_i << " "
            << m->trg.x << " " << m->trg.y << " " << m->trg.z << endl;
        }
      }
    }
    refined_poses.push_back(pose);
    std::cout << pose << endl;
  }

  return EXIT_SUCCESS;
}
