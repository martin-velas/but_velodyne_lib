#define PCL_NO_PRECOMPILE

#include <iostream>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace but_velodyne;
using namespace pcl;
namespace po = boost::program_options;

typedef velodyne_pointcloud::VelodynePoint PointT;

bool parse_arguments(int argc, char **argv,
                     string &input_file, string &output_file, float &leaf_size) {
  string pose_file;
  string transformation;
  po::options_description desc("Transform point cloud\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_cloud,i", po::value<string>(&input_file)->required(), "Input point cloud file *.pcd")
    ("output_cloud,o", po::value<string>(&output_file)->required(), "Output point cloud file *.pcd")
    ("leaf_size,l", po::value<float>(&leaf_size)->default_value(0.1), "Leaf size")
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

  string in_filename, out_filename;
  float leaf_size;
  if(!parse_arguments(argc, argv, in_filename, out_filename, leaf_size)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointT>::Ptr input(new PointCloud<PointT>);
  io::loadPCDFile(in_filename, *input);

  PointCloud<PointT> output;
  subsample_by_voxel_grid(input, output, leaf_size);

  pcl::io::savePCDFileBinary(out_filename, output);
  cerr << "Output saved to " << out_filename <<
    "; reduction from " << input->size() << " to " << output.size() << " points" << endl;

  return EXIT_SUCCESS;
}
