#include <iostream>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace but_velodyne;
namespace po = boost::program_options;
using namespace pcl;

bool parse_arguments(int argc, char **argv,
    string &ref_file, string &align_file, bool &visualize) {

  po::options_description desc("GICP\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("reference_cloud,r", po::value<string>(&ref_file)->required(), "Reference point cloud file *.pcd")
    ("aligned_cloud,a", po::value<string>(&align_file)->required(), "Aligned point cloud file *.pcd")
    ("visualize,v", po::bool_switch(&visualize), "Turn on visualization")
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
    } catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}

int main (int argc, char** argv) {

  string ref_filename, align_filename;
  bool visualize;
  if(!parse_arguments(argc, argv, ref_filename, align_filename, visualize)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZ>::Ptr ref_cloud (new PointCloud<PointXYZ>);
  io::loadPCDFile(ref_filename, *ref_cloud);
  PointCloud<PointXYZ>::Ptr align_cloud (new PointCloud<PointXYZ>);
  io::loadPCDFile(align_filename, *align_cloud);

  Visualizer3D::Ptr vis;
  PointCloud<PointXYZ>::Ptr ref_cloud_subsampled(new PointCloud<PointXYZ>);
  if(visualize) {
    *ref_cloud_subsampled += *ref_cloud;
    subsample_cloud<PointXYZ>(ref_cloud_subsampled, 0.01);

    vis.reset(new Visualizer3D);
    cerr << "Before" << endl;
    vis->addPointCloud(*ref_cloud_subsampled)
        .addPointCloud(*align_cloud).show();
  }

  GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;
  gicp.setInputSource(align_cloud);
  gicp.setInputTarget(ref_cloud);
  gicp.align(*align_cloud);
  cerr << "GICP has converged: " << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << endl;

  Eigen::Matrix4f t = gicp.getFinalTransformation().inverse();
  std::cerr << t << std::endl;
  KittiUtils::printPose(cout, t);

  if(visualize) {
    vis->keepOnlyClouds(0)
        .addPointCloud(*ref_cloud_subsampled)
        .addPointCloud(*align_cloud).show();
  }

  return EXIT_SUCCESS;
}
