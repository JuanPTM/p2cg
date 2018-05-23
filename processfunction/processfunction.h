#ifndef _PROCESSFUNCTION_H
#define _PROCESSFUNCTION_H

#include <opencv2/core/core.hpp>
#include "types/types.h"
#include <iostream>
#include <stdlib.h>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// using namespace cv;
using namespace std;

namespace processfunction
{
  void FILTER(cv::Mat &input);
  std::vector< int > dijk(const int &A, const int &B, const std::vector<point> &points, const int &value);
  std::vector<point> reduceListPoints(const std::vector<point> &pointsToWay, const int &value);
  std::map<float,rgb_color> loadPaleta();
  rgb_color ucharize(const float &v, const std::map<float,rgb_color> &paleta);
  long mymap(const long &x, const long &in_min, const long &in_max, const long &out_min, const long &out_max);
  void  normalizeFloat(float &v);

}

#endif
