/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 30/09/2020
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

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/program_options.hpp>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/LineCloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/CollarLinesRegistration.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace but_velodyne;
using namespace velodyne_pointcloud;
namespace po = boost::program_options;

class PointLineMatch {
public:
    PointXYZ pt;
    PointCloudLine line;

    PointLineMatch(const PointXYZ &pt_, const PointCloudLine &line_) : pt(pt_), line(line_) {
    }
};

class PointsMatch {
public:
    PointXYZ pt1, pt2;

    PointsMatch(const PointXYZ &pt1_, const PointXYZ &pt2_) : pt1(pt1_),  pt2(pt2_) {
    }
};

class EquirectangularCorrespPicker {

public:

    typedef PointXYZI PointT;
    typedef PointCloud<PointT> CloudT;

    struct Point2i_cmp {
      bool operator()(const Point2i &p1, const Point2i &p2) const {
        if(p1.x == p2.x) {
          return p1.y < p2.y;
        } else {
          return p1.x < p2.x;
        }
      }
    };

    EquirectangularCorrespPicker(const cv::Mat &pano_image,
                                 const Eigen::Affine3f camera_pose,
                                 const CloudT &cloud,
                                 const float distance_threshold_) :
        distance_threshold(distance_threshold_),
        window_name("Picking correspondences"), cloud_point_was_picked(false), scale(0.25) {
      pano_image.copyTo(drawing_image);
      this->preparePointCloudSlice(camera_pose, cloud);

      namedWindow(window_name, 1);
      setMouseCallback(window_name, &EquirectangularCorrespPicker::onMouse, this);
      this->draw3DPointsToImage();
    }

    static void onMouse(int event, int x, int y, int flags, void* param) {
      ((EquirectangularCorrespPicker*) param)->onMouse(event, x, y, flags);
    }

    vector<PointLineMatch> pickCorrespondences(void) {
      this->show(scale);
      return correspondences;
    }

    static float horizontalAngle(const float to_font, const float to_right) {
      float heading = fabs(360.0 - VelodynePointCloud::horizontalAngle(to_font, to_right));
      heading += 180.0;
      if(heading >= 360.0) {
        heading -= 360;
      }
      return heading;
    }

    /**
     * @returns horizontal angle, vertical angle, range (angles are relative 0.0-1.0)
     */
    static Vec3f pointToSpherical(const PointT &pt) {
      Vec3f spherical;
      spherical(0) = horizontalAngle(pt.y, pt.x) / 360.0;
      const float range = computeRange(pt);
      spherical(1) = (-asin(pt.z / range) + M_PI_2) / M_PI;
      spherical(2) = range;
      return spherical;
    }

protected:

    bool findClosest3DPoint(const Point2i &pixel, PointT &out_pt) const {
      map<Point2i, int>::const_iterator pt_found = pixel_map_of_cloud.find(pixel);
      if(pt_found != pixel_map_of_cloud.end()) {
        out_pt = cloud_slice[pt_found->second];
        return true;
      } else {
        return false;
      }
    }

    void onMouse(int event, int x, int y, int flags) {
      if(event == EVENT_LBUTTONUP) {
        Point2i px(x / scale, y / scale);
        if(!cloud_point_was_picked) {
          if(this->findClosest3DPoint(px, cloud_pt_picked)) {
            cloud_point_was_picked = true;
            circle(drawing_image, px, 10, CV_RGB(220, 0, 220), 10);
          }
        } else {
          image_px_picked = px;
          cloud_point_was_picked = false;
          circle(drawing_image, px, 10, CV_RGB(220, 0, 220), 20);
          line(drawing_image, px, last_px_clicked, CV_RGB(0, 100, 250), 8);
          this->recordCorrespondence();
        }
        last_px_clicked = px;
        this->show(scale);
      }
    }

    void recordCorrespondence(void) {

      PointT origin, endpoint;
      origin.x = origin.y = origin.z = 0.0;
      endpoint = this->pixelTo3DPoint(image_px_picked);

      PointXYZ pt;
      copyXYZ(cloud_pt_picked, pt);
      printf("New correspondence: [%f, %f, %f] -> [%f, %f, %f]; [%f, %f, %f]\n",
             pt.x, pt.y, pt.z,
             origin.x, origin.y, origin.z,
             endpoint.x, endpoint.y, endpoint.z);
      PointCloudLine line(origin, endpoint);
      correspondences.push_back(PointLineMatch(pt, line));

      // checking:
      Point2i check_px = anglesToPixel(pointToSpherical(endpoint));
      circle(drawing_image, check_px, 10, CV_RGB(0, 255, 0), 20);
    }

    /*
     * x = right; y = front; z = up
     */
    PointT pixelTo3DPoint(const Point2i &px) const {
      PointT pt;
      float range = 2 * distance_threshold;

      // 0rad = up; pi = down:
      float pitch = px.y / ((float) drawing_image.rows) * M_PI;
      // pi/2 = up; -pi/2 = down:
      pitch = M_PI_2 - pitch;
      pt.z = sin(pitch) * range;

      // projection of the range to XY plane
      float horizontal_range = cos(pitch) * range;

      // 0rad = back; pi/2 = left; pi = front; 3/2 pi = right:
      float heading = px.x / ((float) drawing_image.cols) * 2.0*M_PI;
      heading = 3*M_PI_2 - heading;
      pt.x = cos(heading) * horizontal_range;
      pt.y = sin(heading) * horizontal_range;

      return pt;
    }

    void draw3DPointsToImage(void) {
      int idx = 0;
      for(CloudT::const_iterator pt = cloud_slice.begin(); pt < cloud_slice.end(); pt++, idx++) {
        Vec3f spherical = pointToSpherical(*pt);
        const Point2i px = anglesToPixel(spherical);
        this->drawPointToImage(px, spherical(2) / distance_threshold);
        pixel_map_of_cloud[px] = idx;
        for(int dx = -4; dx <= 4; dx++) {
          for(int dy = -4; dy <= 4; dy++) {
            const Point2i px_shifted(px.x + dx, px.y + dy);
            pixel_map_of_cloud[px_shifted] = idx;
          }
        }
      }
    }

    Point2i anglesToPixel(const Vec3f &angles) const {
      Point2i px;
      px.x = angles(0) * drawing_image.cols;
      px.y = angles(1) * drawing_image.rows;
      return px;
    }

    void drawPointToImage(const Point2i &px, const float intensity) {
      uchar r, g, b;
      Visualizer3D::colorizeIntensity(intensity, r, g, b);
      circle(drawing_image, px, 1, CV_RGB(r, g, b), 1);
    }

    void preparePointCloudSlice(const Eigen::Affine3f camera_pose,
                                const CloudT &cloud) {
      CloudT::Ptr cloud_transformed(new CloudT);
      transformPointCloud(cloud, *cloud_transformed, camera_pose.inverse());

      CropBox<PointT> crop_box;
      crop_box.setInputCloud(cloud_transformed);
      Eigen::Vector4f min_coords, max_coords;
      min_coords.x() = min_coords.y() = min_coords.z() = -distance_threshold;
      max_coords.x() = max_coords.y() = max_coords.z() = +distance_threshold;
      crop_box.setMin(min_coords);
      crop_box.setMax(max_coords);
      crop_box.filter(cloud_slice);
    }

    int show(float scale) {
      Mat drawingImageScaled;
      resize(drawing_image, drawingImageScaled, Size(), scale, scale, INTER_NEAREST);
      imshow(window_name, drawingImageScaled);
      return waitKey();
    }

private:

    Mat drawing_image;
    CloudT cloud_slice;
    const float distance_threshold;

    const string window_name;
    map<Point2i, int, Point2i_cmp> pixel_map_of_cloud;
    bool cloud_point_was_picked;
    Point2i last_px_clicked;
    Point2i image_px_picked;
    PointT cloud_pt_picked;
    const float scale;

    vector<PointLineMatch> correspondences;
};

bool parse_arguments(int argc, char **argv,
                     PointCloud<PointXYZI> &cloud,
                     cv::Mat &pano_img,
                     Eigen::Affine3f &camera_pose,
                     float &dist_thresh,
                     bool &rotation_only) {
  string cloud_fn, pano_img_fn, camera_pose_fn;

  po::options_description desc("Visualization for picking LB-Velodyne correspondences\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("point_cloud,c", po::value<string>(&cloud_fn)->required(), "Point cloud from LiDAR.")
    ("pano_image,i", po::value<string>(&pano_img_fn)->required(), "Camera equirectangular image.")
    ("camera_pose,p", po::value<string>(&camera_pose_fn)->required(), "Camera pose file.")
    ("distance_thresh", po::value<float>(&dist_thresh)->default_value(50), "Distance threshold.")
    ("rotation_only", po::bool_switch(&rotation_only), "Estimate rotation only.")
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

  io::loadPCDFile(cloud_fn, cloud);
  pano_img = cv::imread(pano_img_fn);
  camera_pose = KittiUtils::load_kitti_poses(camera_pose_fn).front();

  return true;
}

PointXYZ closestPoint(const PointCloudLine &line, const PointXYZ &pt) {
  const Eigen::Vector3f P  = pt.getVector3fMap();
  const Eigen::Vector3f P0 = line.getBeginPoint().getVector3fMap();
  Eigen::Vector3f d = line.orientation;
  d.normalize();

  const Eigen::Vector3f w = P - P0;

  PointXYZ Pc;
  Pc.getVector3fMap() = P0 + (d * w.dot(d));
  return Pc;
}

void getPointsMatches(const vector<PointLineMatch> &correspondences,
                      vector<PointsMatch> &out_matches) {
  for(vector<PointLineMatch>::const_iterator c = correspondences.begin(); c < correspondences.end(); c++) {
    const PointXYZ Pc = closestPoint(c->line, c->pt);
    out_matches.push_back(PointsMatch(c->pt, Pc));
  }
}

typedef CollarLinesRegistration::MatrixOfPoints MatrixOfPoints;
typedef CollarLinesRegistration::TPoint3D TPoint3D;

Eigen::Affine3f estimateTransformation(const MatrixOfPoints &source_coresp_points,
                                       const MatrixOfPoints &target_coresp_points,
                                       const bool rotation_only) {
  // Define Column vector using definition of TPoint3D
  TPoint3D centroid_0, centroid_1;

  // Translate the point cloud 0 to the coordinates of point cloud 1
  MatrixOfPoints target_coresp_points_translated(target_coresp_points.rows(), target_coresp_points.cols());
  MatrixOfPoints source_coresp_points_translated(source_coresp_points.rows(), source_coresp_points.cols());

  if(rotation_only) {
    centroid_0 << 0.0, 0.0, 0.0;
    centroid_1 << 0.0, 0.0, 0.0;
    target_coresp_points_translated = target_coresp_points;
    source_coresp_points_translated = source_coresp_points;
  } else {
    // Lets compute the translation
    centroid_0 << target_coresp_points.row(0).sum(),
            target_coresp_points.row(1).sum(),
            target_coresp_points.row(2).sum();
    centroid_0 /= target_coresp_points.cols();

    centroid_1 << source_coresp_points.row(0).sum(),
            source_coresp_points.row(1).sum(),
            source_coresp_points.row(2).sum();
    centroid_1 /= source_coresp_points.cols();

    Eigen::Matrix<TPoint3D::Scalar, 1, Eigen::Dynamic> identity_vec =
            Eigen::Matrix<TPoint3D::Scalar, 1, Eigen::Dynamic>::Ones(1, target_coresp_points.cols()); //setOnes();

    // Create matrix with repeating values in columns
    MatrixOfPoints translate_0_mat = centroid_0 * identity_vec;
    MatrixOfPoints translate_1_mat = centroid_1 * identity_vec;

    // Translation of source_coresp_points to the target_coresp_points (Remember this is opposite of camera movement)
    // ie if camera is moving forward, the translation of target_coresp_points to source_coresp_points is opposite
    // TPoint3D t = (centroid_1 - centroid_0);

    target_coresp_points_translated = target_coresp_points - translate_0_mat;
    source_coresp_points_translated = source_coresp_points - translate_1_mat;
  }

  // Compute the Covariance matrix of these two pointclouds moved to the origin
  // This is not properly covariance matrix as there is missing the 1/N
  // 1/N is important for computing eigenvalues(scale), not the eigenvectors(directions) - as we are interested in eigenvectors

  Eigen::Matrix3f A = target_coresp_points_translated * source_coresp_points_translated.transpose();

  // Compute the SVD upon A = USV^t
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Compute the determinant of V*U^t - to find out in what direction the rotation is
  float det = (svd.matrixV() * svd.matrixU().transpose()).determinant();

  // Fix the right hand/left hand rotation : assuming we would like the right hand rotation
  Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
  E(2, 2) = (det >= 0) ? 1.0f : -1.0f;

  // Compute the rotation as R = VEU^t
  // R is the rotation of point_0_translated to fit the source_coresp_points_translated
  Eigen::Matrix3f R = svd.matrixV() * E * (svd.matrixU().transpose());

  typedef Eigen::Matrix<TPoint3D::Scalar, 4, 1> _TyVector4;
  Eigen::Matrix4f transformation = _TyVector4::Ones().asDiagonal();

  transformation.block(0, 0, 3, 3) = R;
  // The translation must be computed as centroid_1 - rotated centroid_0
  transformation.block(0, 3, 3, 1) = centroid_1 - (R * centroid_0);

  return Eigen::Affine3f(transformation);
}

void shiftCorrespondences(const vector<PointLineMatch> &correspondences,
                          const Eigen::Affine3f &T,
                          vector<PointLineMatch> &correspondences_shifted) {
  for(vector<PointLineMatch>::const_iterator c = correspondences.begin(); c < correspondences.end(); c++) {
    const PointXYZ pt_shifted = transformPoint(c->pt, T);
    correspondences_shifted.push_back(PointLineMatch(pt_shifted, c->line));
  }
}

// p1 = LiDAR pt, p2 = point on the camera ray
Eigen::Affine3f getCameraCalibration(const vector<PointLineMatch> &correspondences,
                                     const bool rotation_only) {

  Eigen::Affine3f calibration = Eigen::Affine3f::Identity();

  for(int iteration = 0; iteration < 1000; iteration++) {
    vector<PointLineMatch> correspondences_shifted;
    shiftCorrespondences(correspondences, calibration.inverse(), correspondences_shifted);

    vector<PointsMatch> pt_matches;
    getPointsMatches(correspondences_shifted, pt_matches);

    CollarLinesRegistration::MatrixOfPoints src_camera_points(3, correspondences_shifted.size());
    CollarLinesRegistration::MatrixOfPoints trg_lidar_points(3, correspondences_shifted.size());
    int i = 0;
    for(vector<PointsMatch>::const_iterator m = pt_matches.begin(); m < pt_matches.end(); m++, i++) {
      src_camera_points.block(0, i, 3, 1) = m->pt1.getVector3fMap();
      trg_lidar_points.block(0, i, 3, 1)  = m->pt2.getVector3fMap();
    }

    const Eigen::Affine3f delta = estimateTransformation(src_camera_points, trg_lidar_points, rotation_only);

    const float delta_t = delta.translation().norm();
    const float delta_R = Eigen::AngleAxisf(delta.rotation()).angle();
    if(delta_t < 0.01 && delta_R < 0.0001) {
      cerr << "Finished after " << iteration << " iterations" << endl;
      break;
    }

    calibration = calibration * delta;
  }

  return calibration;
}


int main(int argc, char** argv) {

  bool rotation_only = false;
  if(argc != 2) {
    PointCloud<PointXYZI> cloud;
    cv::Mat pano_img;
    Eigen::Affine3f camera_pose_initial;
    float distance_threshold;
    if(!parse_arguments(argc, argv, cloud, pano_img, camera_pose_initial, distance_threshold,
            rotation_only)) {
      return EXIT_FAILURE;
    }

    vector<PointLineMatch> correspondences;
    Eigen::Affine3f camera_calibration = Eigen::Affine3f::Identity();
    do {
      const Eigen::Affine3f camera_pose = camera_pose_initial * camera_calibration;
      EquirectangularCorrespPicker picker(pano_img, camera_pose, cloud, distance_threshold);
      correspondences = picker.pickCorrespondences();

      cerr << "Picked " << correspondences.size() << " correspondences" << endl;
      if(correspondences.size() > 0) {
        camera_calibration = camera_calibration * getCameraCalibration(correspondences, rotation_only);
        cout << camera_calibration << endl;
      }
    } while(correspondences.size() != 0);
  } else {
    ifstream corresp_file(argv[1]);
    vector<PointLineMatch> correspondences;
    const PointXYZ origin(0, 0, 0);
    while(true) {
      PointXYZ pt_lidar;
      PointXYZ pt_camera;

      corresp_file >> pt_lidar.x >> pt_lidar.y >> pt_lidar.z;
      corresp_file >> pt_camera.x >> pt_camera.y >> pt_camera.z;

      if(corresp_file.eof()) {
        break;
      } else {
        PointCloudLine line;
        correspondences.push_back(PointLineMatch(pt_lidar, PointCloudLine(origin, pt_camera)));
      }
    }
    cout << getCameraCalibration(correspondences, rotation_only) << endl;
  }

  return EXIT_SUCCESS;
}
