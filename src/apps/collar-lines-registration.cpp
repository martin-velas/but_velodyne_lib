/*
 * Registration of the Velodyne point clouds by Collar Line Segments.
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
                     VelodyneMultiFrame::Ptr &src_frame, VelodyneMultiFrame::Ptr &trg_frame,
                     bool &visualization, string &matches_output, float &matches_portion,
                     bool &visualize_output_matches, bool &compute_inv_reg_error) {
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
    ("matches_portion", po::value<float>(&matches_portion)->default_value(0.9),
         "Keep this portion of best matches (by the distance)")
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
  vector<string> src_clouds(clouds_to_process.begin(), clouds_to_process.begin()+calibration.sensorsCount());
  src_frame.reset(new VelodyneMultiFrame(src_clouds, calibration));

  vector<string> trg_clouds(clouds_to_process.begin()+calibration.sensorsCount(), clouds_to_process.end());
  trg_frame.reset(new VelodyneMultiFrame(trg_clouds, calibration));

  vector<Eigen::Affine3f> init_poses = KittiUtils::load_kitti_poses(init_poses_file);
  if(init_poses.size() != 2 && !index_by_cloud_name) {
    cerr << "ERROR, expected exactly two initial poses." << endl;
    return false;
  } else if(index_by_cloud_name) {
    int src_i = KittiUtils::kittiNameToIndex(src_clouds.front());
    int trg_i = KittiUtils::kittiNameToIndex(trg_clouds.front());
    cerr << "Using initial transformation: poses[" << src_i << "]^-1 * poses[" << trg_i << "]" << endl;
    init_transform = init_poses[src_i].inverse() * init_poses[trg_i];
  } else {
    init_transform = init_poses[0].inverse() * init_poses[1];
  }

  return true;
}

int phaseToInt(const float phase) {
  return MIN(phase*100, 99);
}

void show_output_matches(const PointCloud<PointXYZ> &src_cloud,
                         const PointCloud<PointXYZ> &trg_cloud,
                         const Eigen::Affine3f &transformation,
                         const vector<CLSMatch> &matches) {
  Visualizer3D::Ptr vis = Visualizer3D::getCommonVisualizer();
  vis->keepOnlyClouds(0);
  vis->getViewer()->removeAllShapes();
  vis->setColor(255, 220, 220);
  vis->addPointCloud(src_cloud);
  vis->setColor(220, 220, 255);
  vis->addPointCloud(trg_cloud);

  for (vector<CLSMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    if (!isnan(m->getSourceLine().phase) && !isnan(m->getTargetLine().phase)) {
      PointCloudLine line(m->getSrcPt(), m->getTrgPt());
      vis->addLine(line);
    }
  }
  vis->show();
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  SensorsCalibration calibration;
  Eigen::Affine3f init_transform;
  VelodyneMultiFrame::Ptr src_frame, trg_frame;
  bool visualization, visualize_output_matches;
  string matches_output_fn;
  float matches_portion;
  bool compute_inv_reg_error;

  if (!parse_arguments(argc, argv, registration_parameters, pipeline_parameters,
      calibration, init_transform, src_frame, trg_frame, visualization,
      matches_output_fn, matches_portion, visualize_output_matches, compute_inv_reg_error)) {
    return EXIT_FAILURE;
  }

  LinearMoveEstimator null_estimator(0);
  ofstream null_file("/dev/null");
  CollarLinesRegistrationPipeline registration(null_estimator, null_file,
      pipeline_parameters, registration_parameters);

  PolarGridOfClouds src_grid(src_frame->clouds, calibration);
  PolarGridOfClouds trg_grid(trg_frame->clouds, calibration);

  Visualizer3D::Ptr vis;
  PointCloud<PointXYZ> src_cloud, trg_cloud;
  src_frame->joinTo(src_cloud);
  trg_frame->joinTo(trg_cloud);

  if(visualization) {
    vis = Visualizer3D::getCommonVisualizer();
    vis->setColor(255, 0, 0).addPointCloud(src_cloud)
        .setColor(0, 0, 255).addPointCloud(trg_cloud, init_transform.matrix()).show();
    vis->keepOnlyClouds(0);
  }

  RegistrationOutcome result;
  registration.registerTwoGrids(src_grid, trg_grid, init_transform.matrix(), result);

  if(compute_inv_reg_error) {
    RegistrationOutcome inv_result;
    registration.registerTwoGrids(trg_grid, src_grid, init_transform.inverse().matrix(), inv_result);
    const float t_error = tdiff(result.transformation, inv_result.transformation.inverse(), trg_cloud);
    cout << result << " inv_reg_error:" << t_error << endl;
    if(visualization) {
      vis->setColor(255, 0, 255).addPointCloud(trg_cloud, inv_result.transformation.inverse().matrix());
    }
  } else {
    cout << result << endl;
  }

  CLSMatchByCoeffComparator clsMatchByCoeffComparator;

  if(!matches_output_fn.empty()) {
    vector<CLSMatch> matches = registration.getLastMatches();
    cerr << "Matches (unfiltered): " << matches.size() << endl;

    sort(matches.begin(), matches.end(), clsMatchByCoeffComparator);
    matches.erase(matches.begin() + matches_portion*matches.size(), matches.end());
    cerr << "Matches (filtered " << matches_portion*100 << "%): " << matches.size() << endl;

    ofstream matches_stream(matches_output_fn.c_str());
    int not_nan_matches = 0;
    for(vector<CLSMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
      const CLS &l_src = m->getSourceLine();
      const CLS &l_trg = m->getTargetLine();
      if(!isnan(l_src.phase) && !isnan(m->getTargetLine().phase)) {
        const PointXYZ &src_pt = m->getSrcPt();
        const PointXYZ &trg_pt = m->getTrgPt();
        matches_stream << phaseToInt(l_src.phase) << " " << src_pt.x << " " << src_pt.y << " " << src_pt.z << " " <<
                          phaseToInt(l_trg.phase) << " " << trg_pt.x << " " << trg_pt.y << " " << trg_pt.z << " " <<
                          m->getSrcQ() << " " << m->getTrgQ() << endl;
        not_nan_matches++;
      }
    }
    cerr << "Matches (no NaN phases): " << not_nan_matches << endl;

    if(visualize_output_matches) {
      show_output_matches(src_cloud, trg_cloud, result.transformation, matches);
    }
  }

  if(visualization) {
    vis->setColor(255, 0, 0).addPointCloud(src_cloud)
        .setColor(0, 0, 255).addPointCloud(trg_cloud, result.transformation.matrix())
        .addPoses(vector<Eigen::Affine3f>(1, init_transform)).show();
  }

  return EXIT_SUCCESS;
}
