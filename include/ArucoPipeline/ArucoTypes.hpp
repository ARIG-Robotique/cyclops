#pragma once

#include <array>
#include <opencv2/core.hpp>

#define ARUCO_CORNERS_PER_TAG 4
#define ARUCO_DICT_SIZE 100

typedef std::vector<cv::Point2f> ArucoCornerArray;