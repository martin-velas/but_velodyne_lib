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

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>

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

class ManualSubseqRegistration {
public:
  ManualSubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
      const Eigen::Affine3f &init_transform_,
      CollarLinesRegistrationPipeline::Parameters &params_,
      CollarLinesRegistration::Parameters &registration_params_) :
    src_lines(src_lines_), trg_lines(trg_lines_),
    params(params_), registration_params(registration_params_),
    split_idx(src_lines_.line_cloud.size()),
    estimated_transform(init_transform_) {

    pclVis = visualizer.getViewer();
    pclVis->registerPointPickingCallback(&ManualSubseqRegistration::pickPointCallback, *this);
    pclVis->registerKeyboardCallback(&ManualSubseqRegistration::keyCallback, *this);
  }

  Eigen::Affine3f run() {
    setDataToVisualizer();
    visualizer.show();
    return estimated_transform;
  }

protected:

  void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*) {
    int idx = event.getPointIndex();
    if (idx == -1)
      return;

    if(idx < split_idx) {
      if(src_indices.size() < trg_indices.size()) {
        src_indices.push_back(idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked source point but expected target - ignoring\n");
      }
    } else {
      if(src_indices.size() == trg_indices.size()) {
        trg_indices.push_back(idx-split_idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked target point but expected source - ignoring\n");
      }
    }
  }

  void setDataToVisualizer() {
    PointCloud<PointXYZRGB>::Ptr sum_cloud(new PointCloud<PointXYZRGB>);
    *sum_cloud += *Visualizer3D::colorizeCloud(src_lines.line_middles, 255, 0, 0);
    PointCloud<PointXYZ> trg_cloud_transformed;
    transformPointCloud(trg_lines.line_middles, trg_cloud_transformed, estimated_transform);
    *sum_cloud += *Visualizer3D::colorizeCloud(trg_cloud_transformed, 0, 0, 255);
    pclVis->removeAllShapes();
    pclVis->removeAllPointClouds();
    visualizer.addColorPointCloud(sum_cloud);
//    visualizer.setColor(255, 0, 0).addLines(*src_lines);
//    visualizer.setColor(0, 0, 255).addLines(*trg_lines);
    for(int i = 0; i < src_indices.size(); i++) {
      visualizer.addArrow(PointCloudLine(src_lines.line_middles[src_indices[i]],
                                         trg_cloud_transformed[trg_indices[i]]));
    }
    if(src_indices.size() < trg_indices.size()) {
      pclVis->addSphere(trg_cloud_transformed[trg_indices.back()], 0.1, "sphere");
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "s") {
        PCL_DEBUG("Running transformation estimation using SVD ...\n");
        estimateManualTransform();
        setDataToVisualizer();
      } else if(event.getKeySym() == "u") {
        if(src_indices.size() < trg_indices.size() && !trg_indices.empty()) {
          trg_indices.pop_back();
        } else if(!src_indices.empty()) {
          src_indices.pop_back();
        }
        setDataToVisualizer();
      } else if(event.getKeySym() == "a") {
        runAutomaticRegistration(CollarLinesRegistration::NO_THRESHOLD);
        setDataToVisualizer();
      } else if(event.getKeySym() == "m") {
        runAutomaticRegistration(CollarLinesRegistration::MEDIAN_THRESHOLD);
        setDataToVisualizer();
      }
    }
  }

  void estimateManualTransform() {
    if(src_indices.size() > 2) {
      pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> tfe;
      Eigen::Matrix4f t;
      tfe.estimateRigidTransformation(trg_lines.line_middles, trg_indices,
          src_lines.line_middles, src_indices, t);
      estimated_transform = Eigen::Affine3f(t);
    } else {
      PCL_WARN("Unable to estimate transformation with less than 3 matches - skipping.\n");
    }
  }

  void runAutomaticRegistration(CollarLinesRegistration::Threshold th_type) {
    PCL_DEBUG("Running automatic transformation estimation using CLS ...\n");
    registration_params.distance_threshold = th_type;
    estimated_transform = Eigen::Affine3f(registerLineClouds(src_lines, trg_lines,
        estimated_transform.matrix(), registration_params, params));
  }

  Eigen::Matrix4f registerLineClouds(
      const LineCloud &source, const LineCloud &target,
      const Eigen::Matrix4f &initial_transformation,
      CollarLinesRegistration::Parameters registration_params,
      CollarLinesRegistrationPipeline::Parameters pipeline_params) {
    Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations,
        pipeline_params.maxTimeSpent, pipeline_params.significantErrorDeviation,
        pipeline_params.targetError);
    Eigen::Matrix4f transformation = initial_transformation;
    while (!termination()) {
      CollarLinesRegistration icl_fitting(source, target, registration_params,
          transformation);
      float error = icl_fitting.refine();
      termination.addNewError(error);
      transformation = icl_fitting.getTransformation();
    }
    return transformation;
  }


private:
  int split_idx;
  Visualizer3D visualizer;
  pcl::visualization::PCLVisualizer::Ptr pclVis;
  vector<int> src_indices;
  vector<int> trg_indices;

  Eigen::Affine3f estimated_transform;

  LineCloud src_lines, trg_lines;
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;
};

void read_lines(const string &fn, vector<string> &lines) {
  ifstream file(fn.c_str());
  string line;
  while (getline(file, line)) {
    lines.push_back(line);
  }
}

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    Eigen::Affine3f &init_transform,
    vector<Eigen::Affine3f> &src_poses, vector<string> &src_clouds_filenames,
    vector<Eigen::Affine3f> &trg_poses, vector<string> &trg_clouds_filenames) {
  string source_poses_file, target_poses_file;
  string source_clouds_list, target_clouds_list;
  string init_transform_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("matching_threshold", po::value<CollarLinesRegistration::Threshold>(&registration_parameters.distance_threshold)->default_value(registration_parameters.distance_threshold),
      "How the value of line matching threshold is estimated (mean/median/... of line pairs distance). Possible values: MEDIAN_THRESHOLD|MEAN_THRESHOLD|NO_THRESHOLD")
    ("line_weightning", po::value<CollarLinesRegistration::Weights>(&registration_parameters.weighting)->default_value(registration_parameters.weighting),
      "How the weights are assigned to the line matches - prefer vertical lines, close or treat matches as equal. Possible values: DISTANCE_WEIGHTS|VERTICAL_ANGLE_WEIGHTS|NO_WEIGHTS")
    ("shifts_per_match", po::value<int>(&registration_parameters.correnspPerLineMatch)->default_value(registration_parameters.correnspPerLineMatch),
      "[Experimental] How many shift vectors (for SVD) are generated per line match - each is amended by small noise")
    ("shifts_noise_sigma", po::value<float>(&registration_parameters.lineCorrenspSigma)->default_value(registration_parameters.lineCorrenspSigma),
      "[Experimental] Deviation of noise generated for shift vectors (see above)")
    ("lines_per_bin_generated,g", po::value<int>(&pipeline_parameters.linesPerCellGenerated)->default_value(pipeline_parameters.linesPerCellGenerated),
      "How many collar lines are generated per single polar bin")
    ("lines_per_bin_preserved,p", po::value<int>(&pipeline_parameters.linesPerCellPreserved)->default_value(pipeline_parameters.linesPerCellPreserved),
      "How many collar lines are preserved per single polar bin after filtering")
    ("min_iterations", po::value<int>(&pipeline_parameters.minIterations)->default_value(pipeline_parameters.minIterations),
      "Minimal number of registration iterations (similar to ICP iterations)")
    ("max_iterations", po::value<int>(&pipeline_parameters.maxIterations)->default_value(pipeline_parameters.maxIterations),
      "Maximal number of registration iterations")
    ("max_time_for_registration", po::value<int>(&pipeline_parameters.maxTimeSpent)->default_value(pipeline_parameters.maxTimeSpent),
      "Maximal time for registration [sec]")
    ("target_error", po::value<float>(&pipeline_parameters.targetError)->default_value(pipeline_parameters.targetError),
      "Minimal error (average distance of line matches) causing termination of registration")
    ("significant_error_deviation", po::value<float>(&pipeline_parameters.significantErrorDeviation)->default_value(pipeline_parameters.significantErrorDeviation),
      "If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated")
    ("source_clouds_list", po::value<string>(&source_clouds_list)->required(),
      "List of source point clouds")
    ("target_clouds_list", po::value<string>(&target_clouds_list)->required(),
      "List of target point clouds")
    ("source_poses_file", po::value<string>(&source_poses_file)->required(),
      "File with source poses")
    ("target_poses_file", po::value<string>(&target_poses_file)->required(),
      "File with poses of target clouds for initialisation")
    ("nearest_neighbors", po::value<int>(&registration_parameters.nearestNeighbors)->default_value(registration_parameters.nearestNeighbors),
      "How many nearest neighbors (matches) are found for each line of source frame.")
    ("init_transform,i", po::value<string>(&init_transform_filename)->default_value(""),
      "Transform for initialisation (pose file)")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

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

  src_poses = KittiUtils::load_kitti_poses(source_poses_file);
  trg_poses = KittiUtils::load_kitti_poses(target_poses_file);
  read_lines(source_clouds_list, src_clouds_filenames);
  read_lines(target_clouds_list, trg_clouds_filenames);

  if(!init_transform_filename.empty()) {
    init_transform = KittiUtils::load_kitti_poses(init_transform_filename).front();
  } else {
    init_transform = Eigen::Affine3f::Identity();
  }

  return true;
}

void buildLineCloud(const vector<string> &cloud_files,
    const vector<Eigen::Affine3f> &poses,
    int lines_per_cell_gen, int lines_per_cell_preserve,
    LineCloud &out_lines,
    VelodynePointCloud &out_cloud) {
  CollarLinesFilter filter(lines_per_cell_preserve);
  for(int i = 0; i < cloud_files.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(cloud_files[i], cloud, false);
    PolarGridOfClouds polar_grid(cloud);
    LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
    line_cloud.transform(poses[i].matrix());
    out_lines += line_cloud;
    transformPointCloud(cloud, cloud, poses[i]);
    out_cloud += cloud;
  }
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  Eigen::Affine3f init_transform;
  vector<Eigen::Affine3f> src_poses, trg_poses;
  vector<string> src_clouds_filenames, trg_clouds_filenames;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters,
      init_transform,
      src_poses, src_clouds_filenames, trg_poses, trg_clouds_filenames)) {
    return EXIT_FAILURE;
  }

  LineCloud src_lines, trg_lines;
  VelodynePointCloud src_cloud, trg_cloud;
  buildLineCloud(src_clouds_filenames, src_poses,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      src_lines, src_cloud);
  buildLineCloud(trg_clouds_filenames, trg_poses,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      trg_lines, trg_cloud);

  ManualSubseqRegistration registration(src_lines, trg_lines,
      init_transform,
      pipeline_parameters,
      registration_parameters);
  Eigen::Affine3f t = registration.run();
  KittiUtils::printPose(std::cout, t.matrix());

  return EXIT_SUCCESS;
}
