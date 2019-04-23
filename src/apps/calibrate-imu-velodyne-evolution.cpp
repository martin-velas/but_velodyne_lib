/*
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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/InterpolationSE3.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &P,
                     vector<Eigen::Affine3f> &R,
                     Eigen::Affine3f &calibration) {
  string init_calibration_filename, imu_poses_filename, registrations_filename;

  po::options_description desc("IMU Velodyne calibration with evolution strategy\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
  ;

  desc.add_options()
    ("init_calibration,i", po::value<string>(&init_calibration_filename)->required(), "Initial calibration.")
    ("imu_poses,p", po::value<string>(&imu_poses_filename)->required(), "IMU poses differences")
    ("registrations,r", po::value<string>(&registrations_filename)->required(), "Velodyne registrations.")
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

  P = KittiUtils::load_kitti_poses(imu_poses_filename);
  R = KittiUtils::load_kitti_poses(registrations_filename);
  calibration = KittiUtils::load_kitti_poses(init_calibration_filename).front();

  return true;
}

class Calibration {

public:
  Calibration(const Eigen::Affine3f &t_,
      const vector<Eigen::Affine3f> &P,
      const vector<Eigen::Affine3f> &R) : t(t_) {
    error = evaluate(P, R);
  }

  Calibration(const Calibration &c1, const Calibration &c2,
      const vector<Eigen::Affine3f> &P,
      const vector<Eigen::Affine3f> &R) : error(-1.0) {
    LinearInterpolationSE3 interpolation(c1.t, c2.t);
    t = c1.t * interpolation.estimate(0.5);
    mutation();
    error = evaluate(P, R);
  }

  bool operator < (const Calibration &c) const {
    return this->error < c.error;
  }

  Eigen::Affine3f getTransformation(void) const {
    return t;
  }

  float getError(void) const {
    return error;
  }

protected:

  void mutation(void) {
    cv::RNG& rng = cv::theRNG();
    float max_t = 0.0;
    float max_R = degToRad(0.5);
    if(rng.uniform(0.0, 1.0) < 0.05) {
      Eigen::Affine3f delta = pcl::getTransformation(
          rng.uniform(-max_t, max_t), rng.uniform(-max_t, max_t), rng.uniform(-max_t, max_t),
          rng.uniform(-max_R, max_R), rng.uniform(-max_R, max_R), rng.uniform(-max_R, max_R));
      t = delta * t;
    }
  }

  float evaluate(const vector<Eigen::Affine3f> &P, const vector<Eigen::Affine3f> &R) {
    error = 0.0;
    for(int i = 0; i < P.size(); i++) {
      //error += ((P[i]*t).matrix() - (t*R[i]).matrix()).norm();
      error += fabs(Eigen::AngleAxisf( ((P[i]*t).inverse()*(t*R[i])).rotation() ).angle());
    }
    return error;
  }

private:
  Eigen::Affine3f t;
  float error;
};

class EvolutionStrategy {

public:
  EvolutionStrategy(const vector<Eigen::Affine3f> &P_,
      const vector<Eigen::Affine3f> &R_, const Eigen::Affine3f &init_,
      const float max_t_, const float max_R_,
      const size_t population_size) :
    P(P_), R(R_), init(init_), max_t(max_t_), max_R(max_R_), rng(cv::theRNG()) {

    cerr << "Initial error: " << Calibration(init, P, R).getError() << endl;

    for(size_t i = 0; i < population_size; i++) {
      Eigen::Affine3f delta = getTransformation(
          rng.uniform(-max_t, max_t), rng.uniform(-max_t, max_t), rng.uniform(-max_t, max_t),
          rng.uniform(-max_R, max_R), rng.uniform(-max_R, max_R), rng.uniform(-max_R, max_R));
      Calibration c(delta * init, P, R);
      population.push_back(c);
    }
  }

  Eigen::Affine3f run(const int iterations) {
    for(int it = 0; it < iterations; it++) {
      int target_population_size = population.size();
      for(int i = 0; i < target_population_size*0.1; i++) {
        const Calibration &c1 = population[rng.uniform(0, population.size())];
        const Calibration &c2 = population[rng.uniform(0, population.size())];
        Calibration c(c1, c2, P, R);
        population.push_back(c);
      }
      sort(population.begin(), population.end());
      population.erase(population.begin()+target_population_size, population.end());
      cerr << "Best error: " << population.front().getError()
           << " for " << population.front().getTransformation() << endl;
    }
    return population.front().getTransformation();
  }

private:
  vector<Calibration> population;
  vector<Eigen::Affine3f> P, R;
  cv::RNG& rng;
  Eigen::Affine3f init;
  float max_t, max_R;
};

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> P, R;
  Eigen::Affine3f init_calibration;
  if(!parse_arguments(argc, argv, P, R, init_calibration)) {
    return EXIT_FAILURE;
  }

  EvolutionStrategy evolution(P, R, init_calibration, 0.0, degToRad(1.0), 100);
  cout << evolution.run(1000000000) << endl;

  return EXIT_SUCCESS;
}
