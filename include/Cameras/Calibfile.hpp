#include <string>
#include <opencv2/core.hpp>


bool readCameraParameters(std::string description, cv::Mat& camMatrix, cv::Mat& distCoeffs, cv::Size Resolution);

void writeCameraParameters(std::string description, cv::Mat camMatrix, cv::Mat distCoeffs, cv::Size Resolution);