/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 29/04/2019
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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/point_types.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace but_velodyne;
using namespace cv;


void getNonzeroMinMax(const Mat &img, int &minX, int &minY, int &maxX, int &maxY) {
  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  gray.convertTo(gray, CV_32FC1);

  Mat rowsum = Mat::ones(img.cols, 1, gray.type()) * (1.0/img.cols);
  Mat colsum = Mat::ones(img.rows, 1, gray.type()) * (1.0/img.rows);
  rowsum = gray * rowsum;
  colsum = gray.t() * colsum;

  for(minX = 0; minX < img.cols && colsum.at<float>(minX) > 240; minX++) {
  }
  minX--;
  for(minY = 0; minY < img.rows && rowsum.at<float>(minY) > 240; minY++) {
  }
  minY--;
  for(maxX = img.cols-1; maxX >=0  && colsum.at<float>(maxX) > 240; maxX--) {
  }
  maxX++;
  for(maxY = img.rows-1; maxY >=0  && rowsum.at<float>(maxY) > 240; maxY--) {
  }
  maxY++;
}

int main(int argc, char** argv) {

  for(int argi = 1; argi < argc; argi++) {
    Mat whole = imread(argv[argi]);

    vector<Rect> rois;
    rois.push_back(Rect(0, 0,
        whole.cols/2, whole.rows/2));
    rois.push_back(Rect(whole.cols/2, 0,
        whole.cols/2, whole.rows/2));
    rois.push_back(Rect(0, whole.rows/2,
        whole.cols/2, whole.rows/2));
    rois.push_back(Rect(whole.cols/2, whole.rows/2,
        whole.cols/2, whole.rows/2));

    vector<Mat> tiles(4);
    int minX, minY, maxX, maxY;
    minX = maxX = whole.cols/4;
    minY = maxY = whole.rows/4;
    for(int i = 0; i < 4; i++) {
      tiles[i] = whole(rois[i]);
      int current_minX, current_minY, current_maxX, current_maxY;
      getNonzeroMinMax(tiles[i], current_minX, current_minY, current_maxX, current_maxY);
      minX = MIN(minX, current_minX);
      minY = MIN(minY, current_minY);
      maxX = MAX(maxX, current_maxX);
      maxY = MAX(maxY, current_maxY);
      cerr << "minmax: " << minX << " " << " " << minY << " " << maxX << " " << maxY << endl;
      cerr << "current: " << current_minX << " " << current_minY << " " << current_maxX << " " << current_maxY << endl;
    }

    Rect final_roi(minX, minY, maxX-minX, maxY-minY);

    for(int i = 0; i < 4; i++) {
      stringstream fn;
      fn << argv[argi] << "." << i << ".png";
      imwrite(fn.str(), tiles[i](final_roi));
    }
  }

  return EXIT_SUCCESS;
}
