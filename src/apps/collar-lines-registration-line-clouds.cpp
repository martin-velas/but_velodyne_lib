/*
 * Registration of the LineClouds by Collar Line Segments.
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

#include <pcl/common/eigen.h>
#include <boost/program_options.hpp>
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     SensorsCalibration &calibration, Eigen::Affine3f &init_transform,
                     vector<string> &src_filenames, vector<string> &trg_filenames,
                     bool &visualization,
                     string &matches_output, bool &visualize_output_matches, bool &compute_inv_reg_error) {
  string sensors_pose_file, init_poses_file;
  bool index_by_cloud_name;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
  ;

  registration_parameters.prepareForLoading(desc);

  pipeline_parameters.prepareForLoading(desc);

  desc.add_options()
    ("sensors_pose_file", po::value<string>(&sensors_pose_file)->default_value(""),
        "Extrinsic calibration parameters, when multiple Velodyne LiDARs are used")
    ("init_poses", po::value<string>(&init_poses_file)->required(),
        "Initial poses of the frames.")
    ("visualize", po::bool_switch(&visualization),
        "Run visualization")
    ("matches_output", po::value<string>(&matches_output)->default_value(""),
        "Save matches of CLS to this file")
    ("index_by_cloud_name", po::bool_switch(&index_by_cloud_name),
         "Get cloud index from filename.")
    ("visualize_output_matches", po::bool_switch(&visualize_output_matches),
         "Visualize output matches.")
    ("compute_inv_reg_error", po::bool_switch(&compute_inv_reg_error),
         "Estimate error by inverse registration (source and target frames are swapped).")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_to_process;
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  try {
      po::notify(vm);
  }
  catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }

  if(sensors_pose_file.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensors_pose_file);
  }

  if (clouds_to_process.size() != 2*calibration.sensorsCount()) {
    cerr << "ERROR, expected " << 2*calibration.sensorsCount() << " point clouds, found " <<
        clouds_to_process.size() << endl;
    return false;
  }
  src_filenames = vector<string>(clouds_to_process.begin(), clouds_to_process.begin()+calibration.sensorsCount());
  trg_filenames = vector<string>(clouds_to_process.begin()+calibration.sensorsCount(), clouds_to_process.end());

  vector<Eigen::Affine3f> init_poses = KittiUtils::load_kitti_poses(init_poses_file);
  if(init_poses.size() != 2 && !index_by_cloud_name) {
    cerr << "ERROR, expected exactly two initial poses." << endl;
    return false;
  } else if(index_by_cloud_name) {
    int src_i = KittiUtils::kittiNameToIndex(src_filenames.front());
    int trg_i = KittiUtils::kittiNameToIndex(trg_filenames.front());
    cerr << "Using initial transformation: poses[" << src_i << "]^-1 * poses[" << trg_i << "]" << endl;
    init_transform = init_poses[src_i].inverse() * init_poses[trg_i];
  } else {
    init_transform = init_poses[0].inverse() * init_poses[1];
  }

  return true;
}

void show_output_matches(const PointCloud<PointXYZ> &src_cloud,
                         const PointCloud<PointXYZ> &trg_cloud,
                         const Eigen::Affine3f &transformation,
                         const vector<DMatch> &matches) {
  Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
  vis->keepOnlyClouds(0);
  vis->getViewer()->removeAllShapes();
  vis->setColor(255, 220, 220);
  vis->addPointCloud(src_cloud);
  vis->setColor(220, 220, 255);
  vis->addPointCloud(trg_cloud);

  for (vector<DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    PointCloudLine line(src_cloud[m->trainIdx], trg_cloud[m->queryIdx]);
    vis->addLine(line);
  }
  vis->show();
}

void load_line_multiframe(const vector<string> &filenames, const SensorsCalibration &calibration,
                          LineCloud::Ptr output_line_cloud) {
  assert(filenames.size() == calibration.sensorsCount());

  for(int i = 0; i < filenames.size(); i++) {
    LineCloud part;
    ifstream line_cloud_file(filenames[i].c_str());
    line_cloud_file >> part;
    cerr << "Loaded " << part.size() << " points from " << filenames[i] << endl;

    part.transform(calibration.ofSensor(i).matrix());

    *output_line_cloud += part;
  }
}

void registerLineClouds(const LineCloud &source_line_cloud, const LineCloud &target_line_cloud,
                        const Eigen::Matrix4f &initial_transformation,
                        const CollarLinesRegistration::Parameters &registration_params,
                        const CollarLinesRegistrationPipeline::Parameters &pipeline_params,
                        RegistrationOutcome &output_result, std::vector<DMatch> &matches) {
  CollarLinesRegistration::Parameters registration_params_effective = registration_params;
  registration_params_effective.verbose = pipeline_params.verbose;

  CollarLinesRegistration cls_fitting(source_line_cloud, target_line_cloud,
                                      registration_params_effective, initial_transformation.matrix());
  Termination termination(pipeline_params.term_params);
  float best_error = 1e9;
  for(int sampling_it = 0;
      (sampling_it < pipeline_params.term_params.iterationsPerSampling) && !termination();
      sampling_it++) {
    float fitting_error = cls_fitting.refine();
    Eigen::Matrix4f transformation = cls_fitting.getTransformation();
    termination.addNewError(fitting_error);
    RegistrationOutcome current_result;
    current_result.transformation = Eigen::Affine3f(transformation);
    current_result.error = fitting_error;
    current_result.error_deviation = termination.getErrorDeviation();
    if(pipeline_params.verbose) {
      cerr << "Current result: " << current_result << endl;
    }
    if(!pipeline_params.pick_by_lowest_error || current_result.error < best_error) {
      output_result = current_result;
      best_error = current_result.error;
      if(pipeline_params.verbose) {
        cerr << "Was picked as BEST." << endl;
      }
    }
  }
  output_result.term_reason = termination.why();
  matches.clear();
  const vector<cv::DMatch> &last_matches = cls_fitting.getMatches();
  for(vector<cv::DMatch>::const_iterator m = last_matches.begin(); m < last_matches.end(); m++) {
    matches.push_back(*m);
  }
}

int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  SensorsCalibration calibration;
  Eigen::Affine3f init_transform;
  vector<string> src_filenames, trg_filenames;
  bool visualization, visualize_output_matches;
  string matches_output_fn;
  bool compute_inv_reg_error;

  if (!parse_arguments(argc, argv, registration_parameters, pipeline_parameters,
      calibration, init_transform, src_filenames, trg_filenames, visualization,
      matches_output_fn, visualize_output_matches, compute_inv_reg_error)) {
    return EXIT_FAILURE;
  }

  LineCloud::Ptr src_line_cloud(new LineCloud), trg_line_cloud(new LineCloud);
  load_line_multiframe(src_filenames, calibration, src_line_cloud);
  load_line_multiframe(trg_filenames, calibration, trg_line_cloud);

  Visualizer3D::Ptr vis;
  PointCloud<PointXYZ>::Ptr src_cloud, trg_cloud;
  src_cloud = src_line_cloud->getMiddles();
  trg_cloud = trg_line_cloud->getMiddles();

  if(visualization) {
    vis = Visualizer3D::getCommonVisualizer();
    vis->setColor(255, 0, 0).addPointCloud(*src_cloud)
        .setColor(0, 0, 255).addPointCloud(*trg_cloud, init_transform.matrix()).show();
    vis->keepOnlyClouds(0);
  }

  RegistrationOutcome result;
  vector<cv::DMatch> matches;
  registerLineClouds(*src_line_cloud, *trg_line_cloud,
                     init_transform.matrix(), registration_parameters, pipeline_parameters,
                    result, matches);

  if(compute_inv_reg_error) {
    RegistrationOutcome inv_result;
    vector<cv::DMatch> inv_matches;
    registerLineClouds(*trg_line_cloud, *src_line_cloud,
                       init_transform.inverse().matrix(), registration_parameters, pipeline_parameters,
                       inv_result, inv_matches);
    const float t_error = tdiff(result.transformation, inv_result.transformation.inverse(), *trg_cloud);
    cout << result << " inv_reg_error:" << t_error << endl;
    if(visualization) {
      vis->setColor(255, 0, 255).addPointCloud(*trg_cloud, inv_result.transformation.inverse().matrix());
    }
  } else {
    cout << result << endl;
  }

  if(!matches_output_fn.empty()) {
    ofstream matches_stream(matches_output_fn.c_str());
    for(vector<DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
      matches_stream << m->trainIdx << " " << m->queryIdx << endl;
    }

    if(visualize_output_matches) {
      show_output_matches(*src_cloud, *trg_cloud, result.transformation, matches);
    }
  }

  if(visualization) {
    vis->setColor(255, 0, 0).addPointCloud(*src_cloud)
        .setColor(0, 0, 255).addPointCloud(*trg_cloud, result.transformation.matrix())
        .addPoses(vector<Eigen::Affine3f>(1, init_transform)).show();
  }

  return EXIT_SUCCESS;
}
