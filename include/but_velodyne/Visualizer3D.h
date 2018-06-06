/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 23/01/2015
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

#ifndef VISUALISER3D_H_
#define VISUALISER3D_H_

#include <iostream>

#include <cv.h>

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/Correspondence.h>
#include <but_velodyne/Regular2DGrid.h>
#include <but_velodyne/Regular2DGridGenerator.h>
#include <but_velodyne/KittiUtils.h>

using namespace std;

namespace but_velodyne
{

/**!
 * Wrapper of PCL visualizer for displaying 3D data. Visualizer follows
 * builder design pattern. Used should add everything (s)he wants to visualize
 * including metadata (colors, ...) and then trigger the visualization
 * calling show() method.
 */
class Visualizer3D
{
public:
  typedef boost::shared_ptr<Visualizer3D> Ptr;

  Visualizer3D();

  ~Visualizer3D();

  /**!
   * Prints rotation and translation coefficients of [R|t] transformation
   *
   * @param transformation [R|t] transformation
   */
  static void printRT(const Eigen::Matrix4f &transformation) {
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f affineTransf(transformation);
    pcl::getTranslationAndEulerAngles(affineTransf, x, y, z, roll, pitch, yaw);
    cerr << "t: [" << x << ", " << y << ", " << z << "]\t" <<
        "R: [" << roll << ", " << pitch << ", " << yaw << "]" << endl;
  }

  /**!
   * Add new point cloud of arbitrary point type into the visualization. Cloud
   * is optionally transformed. The color for whole cloud is randomly generated
   * or the user defined color is used if was previously set.
   *
   * @param cloud point cloud to visualized
   * @param transformation optional 3D transformation of the cloud before the visualization
   * @return *this instance with the new point cloud ready to be visualized too
   */
  template<typename PointT>
  Visualizer3D& addPointCloud(const pcl::PointCloud<PointT> &cloud,
                              const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity(),
                              const int viewport = 0) {
    pcl::PointCloud<PointT> cloud_transformed;
    transformPointCloud(cloud, cloud_transformed, transformation);
    if(!transformation.isIdentity()) {
      cerr << "Transformation:" << endl << transformation.matrix() << endl;
      printRT(transformation);
    }
    return addColorPointCloud(colorizeCloud(cloud_transformed, rngU(), rngU(), rngU()), Eigen::Matrix4f::Identity(), viewport);
  }

  template<typename PointT>
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(const pcl::PointCloud<PointT> &cloud, uchar r, uchar g, uchar b) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (typename pcl::PointCloud<PointT>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++)
    {
      pcl::PointXYZRGB color_pt(r, g, b);
      color_pt.x = pt->x;
      color_pt.y = pt->y;
      color_pt.z = pt->z;
      color_cloud->push_back(color_pt);
    }
    return color_cloud;
  }

  template<typename PointT>
  static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorizeCloud(const pcl::PointCloud<PointT> &cloud, uchar r, uchar g, uchar b, uchar a) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    color_cloud->resize(cloud.size());
    for(int i = 0; i < cloud.size(); i++)
    {
      pcl::PointXYZRGBA &color_pt = color_cloud->at(i);
      copyXYZ(cloud[i], color_pt);
      color_pt.r = r;
      color_pt.g = g;
      color_pt.b = b;
      color_pt.a = a;
    }
    return color_cloud;
  }

  static void colorizeIntensity(float normalized_intensity, uchar &r, uchar &g, uchar &b) {
    // magic by http://ros-users.122217.n3.nabble.com/RVIZ-PointCloud-display-coloring-based-on-height-td981630.html
    float h = normalized_intensity * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if ( !(i&1) ) f = 1 - f; // if i is even
    uchar n = (1 - f)*255;

    if      (i <= 1) r = n,   g = 0,   b = 255;
    else if (i == 2) r = 0,   g = n,   b = 255;
    else if (i == 3) r = 0,   g = 255, b = n;
    else if (i == 4) r = n,   g = 255, b = 0;
    else if (i >= 5) r = 255, g = n,   b = 0;
  }

  template<typename PointT>
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(const pcl::PointCloud<PointT> &cloud, float intensity) {
    uchar r, g, b;
    colorizeIntensity(intensity, r, g, b);
    return colorizeCloud(cloud, r, g, b);
  }

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloudByPhase(const VelodynePointCloud &cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored->resize(cloud.size());
    for(int i = 0; i < cloud.size(); i++) {
      const velodyne_pointcloud::VelodynePoint &pt = cloud[i];
      pcl::PointXYZRGB &colored_pt = colored->at(i);
      copyXYZ(pt, colored_pt);
      colorizeIntensity(pt.phase, colored_pt.r, colored_pt.g, colored_pt.b);
    }
    return colored;
  }

  template<typename PointT>
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(const pcl::PointCloud<PointT> &cloud, bool grayscale = false) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (typename pcl::PointCloud<PointT>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      float intensity = pt->intensity;
      uchar r, g, b;
      if(grayscale) {
        r = g = b = 255*intensity;
      } else {
        colorizeIntensity(intensity, r, g, b);
      }

      pcl::PointXYZRGB rgb_pt;
      copyXYZ(*pt, rgb_pt);
      rgb_pt.r = r;
      rgb_pt.g = g;
      rgb_pt.b = b;
      rgb_cloud->push_back(rgb_pt);
    }
    return rgb_cloud;
  }

  static void getMinMaxIntensities(const pcl::PointCloud<pcl::PointXYZI> &in,
      float &min_intensity, float &max_intensity, float skiprate) {
    vector<float> intensities;
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator pt = in.begin(); pt < in.end(); pt++) {
      intensities.push_back(pt->intensity);
    }
    sort(intensities.begin(), intensities.end());
    min_intensity = intensities[intensities.size()*skiprate];
    max_intensity = intensities[intensities.size()*(1-skiprate)];
  }

  static void normalizeMinMaxIntensity(const pcl::PointCloud<pcl::PointXYZI> &in,
      pcl::PointCloud<pcl::PointXYZI> &out,
      float found_min, float found_max, float expected_min, float expected_max) {

    out.resize(in.size());
    for (int i = 0; i < in.size(); i++) {
      copyXYZ(in[i], out[i]);
      out[i].intensity = (in[i].intensity-found_min)/(found_max-found_min) * (expected_max-expected_min) + expected_min;
      out[i].intensity = MIN(MAX(out[i].intensity, expected_min), expected_max);
    }
  }

  static void normalizeMinMaxIntensity(const pcl::PointCloud<pcl::PointXYZI> &in,
      pcl::PointCloud<pcl::PointXYZI> &out,
      float skiprate, float expected_min, float expected_max) {
    float min_intensity, max_intensity;
    getMinMaxIntensities(in, min_intensity, max_intensity, skiprate);
    normalizeMinMaxIntensity(in, out, min_intensity, max_intensity, expected_min, expected_max);
  }

  /**!
   * Add new point cloud of arbitrary point type into the visualization. Cloud
   * is optionally transformed. The color of each point is estimated according to its height.
   *
   * @param cloud point cloud to visualized
   * @param transformation optional 3D transformation of the cloud before the visualization
   * @return *this instance with the new point cloud ready to be visualized too
   */
  template<typename PointT>
  Visualizer3D& addCloudColoredByHeight(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transformation =
                                            Eigen::Matrix4f::Identity()) {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);

    float min = min_pt.y;
    float max = max_pt.y;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (typename pcl::PointCloud<PointT>::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      float normalized_height = 1 - (pt->y - min) / (max - min);        // height is decreasing with increasing Y-coordinate

      uchar r, g, b;
      colorizeIntensity(normalized_height, r, g, b);

      pcl::PointXYZRGB rgb_pt;
      copyXYZ(*pt, rgb_pt);
      rgb_pt.r = r;
      rgb_pt.g = g;
      rgb_pt.b = b;
      rgb_cloud->push_back(rgb_pt);
    }

    return this->addColorPointCloud(rgb_cloud, transformation);
  }

  template<typename PointT>
  Visualizer3D& addCloudColoredByIntensity(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transformation =
                                            Eigen::Matrix4f::Identity()) {
    return this->addColorPointCloud(colorizeCloud(cloud), transformation);
  }

	Visualizer3D& addCloudColoredByRing(const VelodynePointCloud &cloud,
			const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity());

  template<typename PointT>
  Visualizer3D& addCloudColoredByAngle(const pcl::PointCloud<PointT> &cloud,
			const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity()) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      uchar r, g, b;
      r = VelodynePointCloud::horizontalAngle(pt->z, pt->x) / 360.0 * 255;
      g = b = 50;

      pcl::PointXYZRGB rgb_pt;
      copyXYZ(*pt, rgb_pt);
      rgb_pt.r = r;
      rgb_pt.g = g;
      rgb_pt.b = b;
      rgb_cloud->push_back(rgb_pt);
    }

    return this->addColorPointCloud(rgb_cloud, transformation);
  }

  /**!
   * Add new point cloud of XYZRGB point type into the visualization. Cloud
   * is optionally transformed. The original color of all points is preserved.
   *
   * @param cloud point cloud to visualized
   * @param transformation optional 3D transformation of the cloud before the visualization
   * @return *this instance with the new point cloud ready to be visualized too
   */
  Visualizer3D& addColorPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity(),
      int viewport = 0);

  Visualizer3D& addColorPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
      const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity(),
      int viewport = 0);

  /**!
   * Add new point clouds of arbitrary point type into the visualization. The color for
   * whole cloud is randomly generated or the user defined color is used if was previously set.
   *
   * @param clouds point clouds to visualized
   * @return *this instance with the new point clouds ready to be visualized too
   */
  template<typename PointT>
  Visualizer3D& addPointClouds(const std::vector< pcl::PointCloud<PointT> > &clouds) {
    for(typename std::vector< pcl::PointCloud<PointT> >::const_iterator cloud = clouds.begin();
        cloud < clouds.end(); cloud++) {
      addPointCloud(*cloud);
    }
    return *this;
  }

  template<typename PointT>
  Visualizer3D& addEvenOddColoredGrid(const Regular2DGrid<pcl::PointCloud<PointT> > &grid) {
    for(int r = 0; r < grid.rows; r++) {
      for(int c = 0; c < grid.cols; c++) {
        if((r+c)%2 == 0) {
          setColor(200, 0, 0);
        } else {
          setColor(0, 0, 200);
        }
        addPointCloud(*grid.at(r, c));
      }
    }
    return *this;
  }

  Visualizer3D& addHeightMap(const Regular2DGrid<float> &height_map,
                             const Regular2DGridGenerator::Parameters &params,
                             const float min_height = VelodyneSpecification::KITTI_HEIGHT);

  /**!
   * Add line segments bound by the corresponding 3D points. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param correspondences corresponding 3D points (endpoints of the line segments)
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<Correspondence3D> &correspondences);

  /**!
   * Add line segments into the visualization. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param lines new lines to be added
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines);

  /**!
   * Add line segments into the visualization using specific color.
   *
   * @param lines new lines to be added
   * @param r red channel of the color used for visualization
   * @param g green channel of the color used for visualization
   * @param b blue channel of the color used for visualization
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines,float r, float g, float b);

  /**!
   * Add line segments of the cloud into the visualization. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param lineCloud new lines of line cloud to be added
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const LineCloud &lineCloud);

  /**!
   * Add line segment bound by the corresponding 3D points. The color for the line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param line corresponding 3D point (endpoint of the line segment)
   * @return *this instance with line segment added for visualization
   */
  Visualizer3D& addLine(const Correspondence3D &line);

  /**!
   * Add line segment into the visualization. The color for the line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param line new line to be added
   * @return *this instance with line segment added for visualization
   */
  Visualizer3D& addLine(const PointCloudLine &line);

  /**!
   * Add line segment into the visualization using specific color.
   *
   * @param line new line to be added
   * @param r red channel of the color used for visualization
   * @param g green channel of the color used for visualization
   * @param b blue channel of the color used for visualization
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLine(const PointCloudLine &line, float r, float g, float b);

  /**!
   * Add the arrow into the visualization. The color for the arrow
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param arrow new arrow to be added
   * @return *this instance with arrow added for visualization
   */
  Visualizer3D& addArrow(const PointCloudLine &arrow);

  Visualizer3D& addArrow(const PointCloudLine &line, string &id);

  /**!
   * Add the sensor representation into the visualization (currently the yellow sphere)
   *
   * @param position position of the sensor
   * @return *this instance with sensor representation added
   */
  Visualizer3D& addSenzor(pcl::PointXYZ position = pcl::PointXYZ(0, 0, 0));

  /**!
   * Add the visualization of 3D matches. The color for each match
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param matches matches to be added
   * @param source_pts source points of matches (indexed by trainIdx)
   * @param target_pts target points of matches (indexed by queryIdx)
   * @return *this instance with matches added for visualization
   */
  template<typename PointT>
  Visualizer3D& addMatches(const std::vector<cv::DMatch> &matches,
                           const pcl::PointCloud<PointT> source_pts,
                           const pcl::PointCloud<PointT> target_pts) {
    int i = 0;
    for(std::vector<cv::DMatch>::const_iterator m = matches.begin();
        m < matches.end(); m++) {
      assert(m->trainIdx < source_pts.size() && m->queryIdx < target_pts.size());
      assert(m->trainIdx >= 0 && m->queryIdx >= 0);
      viewer->addArrow(source_pts[m->trainIdx], target_pts[m->queryIdx],
                       rngU(), rngU(), rngU(), false, getId("arrow"));
    }
    return *this;
  }

  static pcl::PointCloud<pcl::PointXYZ> posesToPoints(const std::vector<Eigen::Affine3f> &poses) {
    pcl::PointCloud<pcl::PointXYZ> poses_cloud;
    for(vector<Eigen::Affine3f>::const_iterator p = poses.begin(); p < poses.end(); p++) {
      poses_cloud.push_back(KittiUtils::positionFromPose(*p));
    }
    return poses_cloud;
  }

  /**!
   * Add visualization of 3D poses (one point per pose)
   *
   * @param poses poses to visualize (sequence of 3D poses of sensor/vehicle expected)
   * @return *this instance with poses added for visualization
   */
  Visualizer3D& addPosesDots(const vector<Eigen::Affine3f> &poses, int viewport = 0);

  Visualizer3D& addPoses(const vector<Eigen::Affine3f> &poses, float axis_size = 0.1, int viewport = 0);

  /**!
   * Add visualization of visual loops between 3D poses
   *
   * @param poses sequence of 3D poses of sensor/vehicle expected
   * @param visual loops detected
   * @return *this instance with visual loops added for visualization
   */
  Visualizer3D& addPosesLoops(const vector<Eigen::Affine3f> &poses,
                              std::vector<cv::DMatch> matches = std::vector<cv::DMatch>());

  /**!
   * Shows the interactive visualization of the all elements added.
   */
  void show() {
    viewer->spin();
  }

  /**!
   * Shows the single frame of visualization of the all elements added.
   *
   * @param time duration of visualization in miliseconds
   */
  void showOnce(int time = 1) {
    viewer->spinOnce(time);
  }

  /**!
   * Saves the single frame of visualization of the all elements added into the image file.
   *
   * @param filename destination image file
   */
  void saveSnapshot(const std::string &filename);

  /**!
   * Close the visualization.
   * !!! There is a bug on Ubuntu-like systems and visualization can not be stopped.
   * This can result in SIGSEGV when multiple visualizations are launched. !!!
   */
  void close() {
    viewer->close();
  }

  /**!
   * Discards all point clouds from visualization except the last N clouds.
   *
   * @param count number of clouds preserved in the visualization
   * @return *this instance with point clouds discarded
   */
  Visualizer3D& keepOnlyClouds(int count);

  /**!
   * Set the color for visualization of the next element (or set of elements in case of point clouds).
   *
   * @param r red channel of the color used for next visualization
   * @param g green channel of the color used for next visualization
   * @param b blue channel of the color used for next visualization
   */
  Visualizer3D& setColor(unsigned r, unsigned g, unsigned b);

  /**!
   * @return the encapsulated PCLVisualizer instance for specific setups.
   */
  const boost::shared_ptr<pcl::visualization::PCLVisualizer>& getViewer() const
  {
    return viewer;
  }

  static boost::shared_ptr<Visualizer3D> getCommonVisualizer() {
    if(!commonVisualizer) {
      commonVisualizer.reset(new Visualizer3D);
    }
    return commonVisualizer;
  }

  Visualizer3D& addRingColoredCloud(const VelodynePointCloud &cloud);

  int getPointSize () const {
    return point_size;
  }

  void setPointSize (int pointSize) {
    point_size = pointSize;
  }

  std::string getId(const string &what);

  double rngF();

  unsigned rngU();

protected:
  cv::RNG& rng;
  int color_index;
  vector<unsigned> color_stack;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  int identifier;
  vector<string> all_identifiers;
  int point_size;

  static boost::shared_ptr<Visualizer3D> commonVisualizer;
};

} /* namespace but_velodyne */

#endif /* VISUALISER3D_H_ */
