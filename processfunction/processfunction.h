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
  cv::Mat FILTER(cv::Mat input);
  std::vector< int > dijk(int A, int B, const std::vector<point> points, int value);
  std::vector<point> reduceListPoints(std::vector<point> pointsToWay, int value);
  std::map<float,rgb_color> loadPaleta();
  rgb_color ucharize(float v, std::map<float,rgb_color> paleta);
  long mymap(long x, long in_min, long in_max, long out_min, long out_max);
  float normalizeFloat(float v);

}

#endif
