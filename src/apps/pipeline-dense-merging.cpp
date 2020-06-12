/*
 * Pipeline for progressive dense clouds merging.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Date: 27/04/2020
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

#include <boost/program_options.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/DenseCloudRegistration.h>
#include <but_velodyne/Overlap.h>
#include <but_velodyne/SlamPlusPlus.h>
#include <but_velodyne/EigenUtils.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

class VelodyneDenseCloud {

public:

    typedef PointXYZ PointT;
    typedef boost::shared_ptr<VelodyneDenseCloud> Ptr;
    typedef boost::shared_ptr<const VelodyneDenseCloud> ConstPtr;

    VelodyneDenseCloud(const string &pcd_filename_, const int index_) :
          pcd_filename(pcd_filename_), index(index_), transformation(Eigen::Affine3f::Identity()) {
    }

    template <class T>
    void getFullCloud(PointCloud<T> &full_cloud) const {
      io::loadPCDFile(pcd_filename, full_cloud);
      transformPointCloud(full_cloud, full_cloud, this->transformation);
    }

    void transform(const Eigen::Affine3f &delta) {
      transformation = delta * transformation;
    }

    void setTransformation(const Eigen::Affine3f &new_transformation) {
      this->transformation = new_transformation;
    }

    const Eigen::Affine3f& getTransformation(void) const {
      return this->transformation;
    }

    const string pcd_filename;
    const int index;

private:

    Eigen::Affine3f transformation;
};

class CloudsMerge {

    typedef PointCloud<VelodyneDenseCloud::PointT> CloudT;

public:

    explicit CloudsMerge(const VelodyneDenseCloud::Ptr &first_item) {
      merge.push_back(first_item);
    }

    void mergeWith(const CloudsMerge &other) {
      this->merge.insert(this->merge.end(), other.merge.begin(), other.merge.end());
      voxel_cloud.reset();
    }

    void transform(const Eigen::Affine3f &t) {
      for(vector<VelodyneDenseCloud::Ptr>::iterator c = merge.begin(); c < merge.end(); c++) {
        (*c)->transform(t);
      }
      voxel_cloud.reset();
    }

    typedef vector<VelodyneDenseCloud::Ptr>::const_iterator const_iterator;

    const_iterator begin(void) const {
      return merge.begin();
    }

    const_iterator end(void) const {
      return merge.end();
    }

    float overlapWith(CloudsMerge &other, DenseCloudOverlap::Parameters overlaping_params) {
      CloudT::ConstPtr src_cloud = this->getVoxelCloud(overlaping_params.leaf_size);
      CloudT::ConstPtr trg_cloud = other.getVoxelCloud(overlaping_params.leaf_size);
      overlaping_params.leaf_size = -1.0;

      DenseCloudOverlap overlap_estimation(overlaping_params);
      float absolute_overlap, relative_overlap;
      overlap_estimation.compute(src_cloud, trg_cloud, absolute_overlap, relative_overlap);
      return relative_overlap;
    }

    Eigen::Affine3f registerWith(CloudsMerge &other, DenseCloudRegistration::Parameters registration_params) {
      CloudT::ConstPtr src_cloud = this->getVoxelCloud(registration_params.leaf_size);
      CloudT::ConstPtr trg_cloud = other.getVoxelCloud(registration_params.leaf_size);
      registration_params.leaf_size = -1.0;

      DenseCloudRegistration registration(src_cloud, trg_cloud, registration_params);
      return registration.run();
    }

    friend ostream& operator<<(ostream &stream, const CloudsMerge &merge);

    CloudT::Ptr getVoxelCloud(const float leaf_size) {
      if(voxel_cloud) {
        return voxel_cloud;
      } else {
        CloudT::Ptr full_cloud(new CloudT);
        for(vector<VelodyneDenseCloud::Ptr>::const_iterator c = merge.begin(); c < merge.end(); c++) {
          PointCloud<VelodyneDenseCloud::PointT> part;
          (*c)->getFullCloud(part);
          *full_cloud += part;
        }
        voxel_cloud.reset(new CloudT);
        subsample_by_voxel_grid(full_cloud, *voxel_cloud, leaf_size);
        return voxel_cloud;
      }
    }

private:
    vector<VelodyneDenseCloud::Ptr> merge;
    CloudT::Ptr voxel_cloud;
};

ostream& operator<<(ostream &stream, const CloudsMerge &merge) {
  stream << "[";
  for(vector<VelodyneDenseCloud::Ptr>::const_iterator c = merge.merge.begin(); c < merge.merge.end(); c++) {
    stream << (*c)->index;
    if(c+1 < merge.merge.end()) {
      stream << ", ";
    }
  }
  stream << "]";
  return stream;
}

bool parse_arguments(const int argc, char *argv[],
                     string &output_dir, string &output_poses,
                     DenseCloudRegistration::Parameters &registration_params,
                     DenseCloudOverlap::Parameters &overlaping_params,
                     vector<CloudsMerge> &merges) {
  po::options_description desc("Merge sequences of the Velodyne frames\n"
                               "======================================\n"
                               " * Allowed options");
  desc.add_options()
          ("help,h", "produce help message")
          ("out_dir", po::value<string>(&output_dir)->required(), "Output storage directory.")
          ("output_poses", po::value<string>(&output_poses)->required(), "Output refined poses.")
          ;
  registration_params.loadFrom(desc);

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  vector<string> clouds_fn = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }
  try {
    po::notify(vm);
  } catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  for(int i = 0; i < clouds_fn.size(); i++) {
    VelodyneDenseCloud::Ptr vdc(new VelodyneDenseCloud(clouds_fn[i], i));
    merges.push_back(CloudsMerge(vdc));
  }
  overlaping_params.leaf_size = registration_params.leaf_size;
  overlaping_params.max_match_distance = 2.0 * registration_params.max_match_distance;
  overlaping_params.visualization = registration_params.visualization;

  return true;
}

class Registration {
public:
    Registration(const int src_i_, const int trg_i_, const Eigen::Affine3f t_) :
            src_i(src_i_), trg_i(trg_i_), t(t_) {
    }

    const int src_i, trg_i;
    const Eigen::Affine3f t;
};

void compute_registrations(vector<CloudsMerge> &merges,
                           const DenseCloudRegistration::Parameters &registration_params,
                           vector<Registration> &registrations) {
  for(int i = 0; i < merges.size(); i++) {
    int src_i = i;
    int trg_i = (i+1) % merges.size();
    if(src_i > trg_i) {
      swap(src_i, trg_i);
    }
    Eigen::Affine3f t = merges[src_i].registerWith(merges[trg_i], registration_params);
    registrations.push_back(Registration(src_i, trg_i, t));
  }
}

void compute_poses(const vector<Registration> &registrations,
                   vector<Eigen::Affine3f> &poses) {
  SlamPlusPlus slampp;
  Eigen::Matrix<double, 6, 1> inf_diagonal;
  inf_diagonal << 100, 100, 100, 10000, 10000, 10000;
  Eigen::Matrix<double, 6, 6> inf_matrix = inf_diagonal.asDiagonal();
  for(vector<Registration>::const_iterator r = registrations.begin(); r < registrations.end(); r++) {
    slampp.addEdge(r->src_i, r->trg_i, r->t, inf_matrix);
  }
  slampp.optimize(poses);
}

class Overlap {
public:
    int src_i, trg_i;
    float overlap;

    Overlap(const int src_i_, const int trg_i_, const float overlap_) :
        src_i(src_i_), trg_i(trg_i_), overlap(overlap_) {
    }

    bool operator<(const Overlap &other) const {
      return this->overlap < other.overlap;
    }
};

void compute_overlaps(vector<CloudsMerge> &merges,
                      const DenseCloudOverlap::Parameters &overlaping_params,
                      vector<Overlap> &overlaps) {
  if(merges.size() < 2) {
    return;
  }
  for(int i = 0; i < merges.size(); i++) {
    int src_i = i;
    int trg_i = (i+1) % merges.size();
    if(src_i > trg_i) {
      swap(src_i, trg_i);
    }
    const float overlap_value = merges[src_i].overlapWith(merges[trg_i], overlaping_params);
    overlaps.push_back(Overlap(src_i, trg_i, overlap_value));
  }
}

bool merge_sets(vector<CloudsMerge> &merges,
                const DenseCloudRegistration::Parameters &registration_params,
                const DenseCloudOverlap::Parameters &overlaping_params,
                const string &output_dir) {
  static int merges_performed = 0;
  vector<Registration> registrations;
  compute_registrations(merges, registration_params, registrations);

  vector<Eigen::Affine3f> closed_poses;
  compute_poses(registrations, closed_poses);
  for(int i = 0; i < merges.size(); i++) {
    merges[i].transform(closed_poses[i]);
  }

  vector<Overlap> overlaps;
  compute_overlaps(merges, overlaping_params, overlaps);
  sort(overlaps.begin(), overlaps.end());
  reverse(overlaps.begin(), overlaps.end());

  vector<bool> delete_mask(merges.size(), false);
  vector<bool> merged_mask(merges.size(), false);
  bool merge_done = false;

  cerr << " ===== Merging ===== " << endl;
  stringstream ss;
  ss << output_dir << "/" << KittiUtils::getKittiFrameName(merges_performed, ".txt");
  ofstream debug_file(ss.str());
  for(vector<Overlap>::const_iterator o = overlaps.begin(); o < overlaps.end(); o++) {
    if(!merged_mask[o->src_i] && !merged_mask[o->trg_i]) {
      Eigen::Affine3f src_to_trg = merges[o->src_i].registerWith(merges[o->trg_i], registration_params);
      Eigen::Affine3f trg_to_src = merges[o->trg_i].registerWith(merges[o->src_i], registration_params);

      const float error = tdiff(src_to_trg, trg_to_src.inverse(), 10.0);

      debug_file << o->src_i << ":" << merges[o->src_i] << "; " << o->trg_i << ":" << merges[o->trg_i]  << "; " << error << endl;
      cerr << "Trying to merge: " << o->src_i << " with " << o->trg_i << ", overlap: " << o->overlap << ", forward-backward error: " << error << endl;
      stringstream ss_src, ss_trg;
      ss_src << output_dir << "/" << KittiUtils::getKittiFrameName(merges_performed, "") << "-" << o->src_i << ".pcd";
      io::savePCDFileBinary(ss_src.str(), *merges[o->src_i].getVoxelCloud(registration_params.leaf_size));
      ss_trg << output_dir << "/" << KittiUtils::getKittiFrameName(merges_performed, "") << "-" << o->trg_i << ".pcd";
      io::savePCDFileBinary(ss_trg.str(), *merges[o->trg_i].getVoxelCloud(registration_params.leaf_size));

      if(error < 0.05) {
        merges[o->trg_i].transform(src_to_trg);
        merges[o->src_i].mergeWith(merges[o->trg_i]);
        delete_mask[o->trg_i] = true;
        merged_mask[o->src_i] = merged_mask[o->trg_i] = true;
        merge_done = true;
      }
    }
  }

  cerr << "Final merges: " << endl;
  vector<CloudsMerge>::iterator merges_it = merges.begin();
  for(vector<bool>::iterator d = delete_mask.begin(); d < delete_mask.end(); d++) {
    if(*d) {
      merges_it = merges.erase(merges_it);
    } else {
      cerr << " * " << *merges_it << endl;
      merges_it++;
    }
  }

  merges_performed++;
  return merge_done;
}

int main(int argc, char *argv[]) {

  string output_dir, output_poses;
  DenseCloudRegistration::Parameters registration_params;
  DenseCloudOverlap::Parameters overlaping_params;
  vector<CloudsMerge> merges;

  if(!parse_arguments(argc, argv, output_dir, output_poses, registration_params, overlaping_params, merges)) {
    return EXIT_FAILURE;
  }
  size_t poses_to_compute = merges.size();

  // merge itself
  while(merge_sets(merges, registration_params, overlaping_params, output_dir))
    ;

  // output
  vector<Eigen::Affine3f> final_poses(poses_to_compute);
  for(vector<CloudsMerge>::iterator m = merges.begin(); m < merges.end(); m++) {
    for(CloudsMerge::const_iterator c = m->begin(); c < m->end(); c++) {
      final_poses[(*c)->index] = (*c)->getTransformation();
    }
  }
  ofstream output_poses_file(output_poses);
  for(vector<Eigen::Affine3f>::const_iterator p = final_poses.begin(); p < final_poses.begin(); p++) {
    output_poses_file << *p << endl;
  }

  return EXIT_SUCCESS;
}
